// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "encoder_decoder_tester.hpp"

#include <gtest/gtest.h>

#include <sensor_msgs/image_encodings.hpp>

using namespace std::placeholders;

sensor_msgs::msg::Image::SharedPtr EncoderDecoderTester::makeTestMessage()
{
  auto img = std::make_shared<Image>();
  img->encoding = original_ros_encoding_;
  img->width = width_;
  img->height = height_;
  img->step = (sensor_msgs::image_encodings::bitDepth(img->encoding) / 8) * img->width *
              sensor_msgs::image_encodings::numChannels(img->encoding);
  img->header.frame_id = frame_id_;
  img->is_bigendian = false;
  return (img);
}

int byte_depth(const std::string & enc)
{
  return (
    (sensor_msgs::image_encodings::bitDepth(enc) * sensor_msgs::image_encodings::numChannels(enc)) /
    8);
}

void EncoderDecoderTester::runTest()
{
  enc_.setQMax(10);
  enc_.setBitRate(8242880);
  enc_.setGOPSize(2);
  enc_.setFrameRate(100, 1);

  if (!enc_.initialize(
        width_, height_,
        std::bind(&EncoderDecoderTester::packetReady, this, _1, _2, _3, _4, _5, _6, _7, _8, _9),
        original_ros_encoding_)) {
    std::cerr << "failed to initialize encoder!" << std::endl;
    return;
  }
  std::cout << "original encoding " << original_ros_encoding_ << " has "
            << byte_depth(original_ros_encoding_) << " bytes/pixel" << std::endl;

  for (size_t i = 0; i < num_frames_; i++) {
    auto img = makeTestMessage();
    img->header.stamp = rclcpp::Time(i + 1, RCL_SYSTEM_TIME);
    img->data.resize(img->step * img->height, static_cast<uint8_t>(i));
    addImage(img);
    enc_.encodeImage(*img);
  }
  enc_.flush();
  dec_.flush();
}

void EncoderDecoderTester::check()
{
  const size_t n = num_frames_;
  EXPECT_EQ(getTsSum(), (n * (n + 1)) / 2);
  EXPECT_EQ(getPtsSum(), (n * (n - 1)) / 2);
  EXPECT_EQ(getPacketCounter(), n);
  EXPECT_EQ(getFrameCounter(), n);
}

void EncoderDecoderTester::packetReady(
  const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
  uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz)
{
  pts_sum_ += pts;
  ffmpeg_image_transport_msgs::msg::FFMPEGPacket msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.encoding = codec;
  msg.flags = flags;
  msg.pts = pts;
  msg.width = width;
  msg.height = height;
  msg.data.reserve(sz);
  std::copy(data, data + sz, std::back_inserter(msg.data));
  decodePacket(msg);
}

void EncoderDecoderTester::addImage(const Image::ConstSharedPtr & img)
{
  time_to_image_.insert({rclcpp::Time(img->header.stamp).nanoseconds(), img});
}

void EncoderDecoderTester::decodePacket(const FFMPEGPacket & msg)
{
  EXPECT_EQ(msg.header.frame_id, frame_id_);
  EXPECT_EQ(msg.encoding, expected_packet_encoding_);
  EXPECT_EQ(msg.width, width_);
  EXPECT_EQ(msg.height, height_);

  if (!dec_.isInitialized()) {
    dec_.initialize(
      msg.encoding, std::bind(&EncoderDecoderTester::imageCallback, this, _1, _2, _3),
      decoder_name_);
  }
  if (!dec_.decodePacket(
        msg.encoding, &msg.data[0], msg.data.size(), msg.pts, msg.header.frame_id,
        msg.header.stamp)) {
    std::cerr << "error decoding packet!" << std::endl;
    throw(std::runtime_error("error decoding packet!"));
  }
  packet_counter_++;
}

void EncoderDecoderTester::imageCallback(
  const Image::ConstSharedPtr & img, bool /* isKeyFrame */, const std::string & av_pix_fmt)
{
  const auto stamp = rclcpp::Time(img->header.stamp).nanoseconds();
  ts_sum_ += stamp;
  const auto it = time_to_image_.find(stamp);
  if (it == time_to_image_.end()) {
    std::cerr << "cannot find image from time stamp " << stamp << std::endl;
    throw(std::runtime_error("image time stamp not found"));
  }
  const auto & orig = it->second;
  EXPECT_EQ(av_pix_fmt, decoder_av_pix_fmt_);
  EXPECT_EQ(img->header.frame_id, orig->header.frame_id);
  EXPECT_EQ(img->header.stamp, orig->header.stamp);
  EXPECT_EQ(img->encoding, orig->encoding);
  EXPECT_EQ(img->is_bigendian, orig->is_bigendian);
  EXPECT_EQ(img->width, orig->width);
  EXPECT_EQ(img->height, orig->height);
  EXPECT_EQ(img->step, orig->step);
  EXPECT_EQ(img->data.size(), orig->data.size());
  EXPECT_EQ(img->step, orig->step);
  EXPECT_EQ(img->step * img->height, orig->data.size());
  EXPECT_GE(img->data.size(), 1);
  const int num_channels = sensor_msgs::image_encodings::numChannels(img->encoding);
  const int num_orig_channels = sensor_msgs::image_encodings::numChannels(orig->encoding);
  if (num_channels != num_orig_channels) {
    std::cerr << "num channel mismatch! orig: " << num_orig_channels << " vs now: " << num_channels
              << std::endl;
    throw(std::runtime_error("num channel mismatch!"));
  }
  bool data_equal{true};
  int last_err = 0;
  for (size_t row = 0; row < img->height; row++) {
    for (size_t col = 0; col < img->width; col++) {
      for (int channel = 0; channel < num_channels; channel++) {
        const uint8_t * p_orig = &(orig->data[row * orig->step + col * num_channels + channel]);
        const uint8_t * p = &(img->data[row * orig->step + col * num_channels + channel]);
        int err = static_cast<int>(*p) - static_cast<int>(*p_orig);
        if (err && (err != last_err || col == 0 || col == img->width - 1)) {
          std::cout << "mismatch image # " << rclcpp::Time(img->header.stamp).nanoseconds()
                    << " at line " << row << " col: " << col << " chan: " << channel
                    << ", orig: " << static_cast<int>(*p_orig) << " now: " << static_cast<int>(*p)
                    << std::endl;
          data_equal = false;
          last_err = err;
        }
      }
    }
  }
  EXPECT_TRUE(data_equal);
  frame_counter_++;
}
