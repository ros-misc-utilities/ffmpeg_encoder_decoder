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

#include <gtest/gtest.h>
#include <unistd.h>

#include <ffmpeg_encoder_decoder/decoder.hpp>
#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>

using namespace std::placeholders;
using sensor_msgs::msg::Image;
using EncoderCallback = ffmpeg_encoder_decoder::Encoder::Callback;
using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;

class EncoderTester
{
public:
  EncoderTester(const std::string & frame_id, const std::string & codec, uint32_t w, uint32_t h)
  : expected_frame_id_(frame_id), expected_codec_(codec), expected_width_(w), expected_height_(h)
  {
  }
  void setCallback(EncoderCallback cb) { callback_ = cb; }
  const auto & getCodec() const { return (expected_codec_); }
  const auto & getFrameId() const { return (expected_frame_id_); }
  const auto & getWidth() const { return (expected_width_); }
  const auto & getHeight() const { return (expected_height_); }
  const auto & getPacketSum() const { return (packet_sum_); }
  const auto & getPtsSum() const { return (pts_sum_); }
  const auto & getTsSum() const { return (ts_sum_); }
  const auto & getPacketCounter() const { return (packet_counter_); }

  void packetReady(
    const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
    uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz)
  {
    (void)flags;
    (void)pts;
    EXPECT_EQ(frame_id, expected_frame_id_);
    EXPECT_EQ(codec, expected_codec_);
    EXPECT_EQ(width, expected_width_);
    EXPECT_EQ(height, expected_height_);
    for (size_t i = 0; i < sz; i++) {
      packet_sum_ += data[i];
    }
    pts_sum_ += pts;
    ts_sum_ += stamp.nanoseconds();
    packet_counter_++;
    if (callback_) {
      callback_(frame_id, stamp, codec, width, height, pts, flags, data, sz);
    }
  }

private:
  std::string expected_frame_id_;
  std::string expected_codec_;
  uint32_t expected_width_{0};
  uint32_t expected_height_{0};
  uint64_t packet_sum_{0};
  uint64_t pts_sum_{0};
  int64_t ts_sum_{0};
  uint64_t packet_counter_{0};
  std::unordered_map<int, Image::ConstSharedPtr> time_to_image_;
  EncoderCallback callback_{nullptr};
};

class DecoderTester
{
public:
  DecoderTester(const std::string & frame_id, const std::string & encoding, uint32_t w, uint32_t h)
  : expected_frame_id_(frame_id),
    expected_encoding_(encoding),
    expected_width_(w),
    expected_height_(h)
  {
  }
  void setDecoder(const std::string & decoder) { decoder_name_ = decoder; }
  const auto & getEncoding() const { return (expected_encoding_); }
  const auto & getFrameId() const { return (expected_frame_id_); }
  const auto & getWidth() const { return (expected_width_); }
  const auto & getHeight() const { return (expected_height_); }
  const auto & getImageSum() const { return (image_sum_); }
  const auto & getTsSum() const { return (ts_sum_); }
  const auto & getFrameCounter() const { return (frame_counter_); }
  const auto & getPacketCounter() const { return (packet_counter_); }

  // entry point for the encoded packet (passthrough from EncoderTester)
  void packetReady(
    const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
    uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz)
  {
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

  void addImage(const Image::ConstSharedPtr & img)
  {
    time_to_image_.insert({rclcpp::Time(img->header.stamp).nanoseconds(), img});
  }

  void finalize() { decoder_.flush(); }

private:
  void decodePacket(const FFMPEGPacket & msg)
  {
    EXPECT_EQ(msg.header.frame_id, expected_frame_id_);
    EXPECT_EQ(msg.encoding, expected_encoding_);
    EXPECT_EQ(msg.width, expected_width_);
    EXPECT_EQ(msg.height, expected_height_);

    if (!decoder_.isInitialized()) {
      decoder_.initialize(
        msg.encoding, std::bind(&DecoderTester::imageCallback, this, _1, _2), decoder_name_);
    }
    if (!decoder_.decodePacket(
          msg.encoding, &msg.data[0], msg.data.size(), msg.pts, msg.header.frame_id,
          msg.header.stamp)) {
      std::cerr << "error decoding packet!" << std::endl;
      throw(std::runtime_error("error decoding packet!"));
    }
    packet_counter_++;
  }

  void imageCallback(const Image::ConstSharedPtr & img, bool /* isKeyFrame */)
  {
    const auto stamp = rclcpp::Time(img->header.stamp).nanoseconds();
    const auto it = time_to_image_.find(stamp);
    if (it == time_to_image_.end()) {
      std::cerr << "cannot find image from time stamp " << stamp << std::endl;
      ;
      throw(std::runtime_error("image time stamp not found"));
    }
    const auto & orig = it->second;
    EXPECT_EQ(img->header.frame_id, orig->header.frame_id);
    EXPECT_EQ(img->header.stamp, orig->header.stamp);
    EXPECT_EQ(img->encoding, orig->encoding);
    EXPECT_EQ(img->is_bigendian, orig->is_bigendian);
    EXPECT_EQ(img->width, orig->width);
    EXPECT_EQ(img->height, orig->height);
    EXPECT_EQ(img->step, orig->step);
    EXPECT_EQ(img->data.size(), orig->data.size());
    EXPECT_EQ(img->step, img->width * 3);
    EXPECT_EQ(img->step * img->height, orig->data.size());
    EXPECT_GE(img->data.size(), 1);
    bool data_equal{true};
    for (size_t i = 0; i < img->data.size(); i++) {
      if (img->data[i] != orig->data[i]) {
        std::cout << "mismatch at " << i << ", orig: " << static_cast<int>(orig->data[i])
                  << " now: " << static_cast<int>(img->data[i]) << std::endl;
        data_equal = false;
      }
    }
    EXPECT_TRUE(data_equal);
    frame_counter_++;
  }

  std::string expected_frame_id_;
  std::string expected_encoding_;
  uint32_t expected_width_{0};
  uint32_t expected_height_{0};
  uint64_t image_sum_{0};
  int64_t ts_sum_{0};
  uint64_t frame_counter_{0};
  uint64_t packet_counter_{0};
  std::unordered_map<int, Image::ConstSharedPtr> time_to_image_;
  std::string decoder_name_;
  ffmpeg_encoder_decoder::Decoder decoder_;
};

void test_encoder_msg(int numFrames, const std::string & encoder, EncoderTester * tester)
{
  ffmpeg_encoder_decoder::Encoder enc;
  enc.setEncoder(encoder);
  enc.addAVOption("profile", "main");
  enc.addAVOption("preset", "slow");
  enc.setQMax(10);
  enc.setBitRate(8242880);
  enc.setGOPSize(2);
  enc.setFrameRate(100, 1);

  if (!enc.initialize(
        tester->getWidth(), tester->getHeight(),
        std::bind(&EncoderTester::packetReady, tester, _1, _2, _3, _4, _5, _6, _7, _8, _9))) {
    std::cerr << "failed to initialize encoder!" << std::endl;
    return;
  }
  Image image;
  image.encoding = "bgr8";
  image.width = tester->getWidth();
  image.height = tester->getHeight();
  image.step = image.width * 3;  // 3 bytes per pixel
  image.header.frame_id = tester->getFrameId();
  image.is_bigendian = false;

  for (int64_t i = 0; i < numFrames; i++) {
    image.header.stamp = rclcpp::Time(i + 1, RCL_SYSTEM_TIME);
    image.data.resize(image.step * image.height, static_cast<uint8_t>(i));
    enc.encodeImage(image);
  }
  enc.flush();
}

void test_encoder_decoder_msg(
  int numFrames, const std::string & encoder, const std::string & decoder,
  EncoderTester * enc_tester, DecoderTester * dec_tester)
{
  dec_tester->setDecoder(decoder);
  ffmpeg_encoder_decoder::Encoder enc;
  enc.setEncoder(encoder);
  enc.addAVOption("x265-params", "lossless=1");
  enc.addAVOption("crf", "0");  // may not be needed for lossless
  enc.setAVSourcePixelFormat("gray");
  enc.setCVBridgeTargetFormat("mono8");
  enc_tester->setCallback(
    std::bind(&DecoderTester::packetReady, dec_tester, _1, _2, _3, _4, _5, _6, _7, _8, _9));

  if (!enc.initialize(
        enc_tester->getWidth(), enc_tester->getHeight(),
        std::bind(&EncoderTester::packetReady, enc_tester, _1, _2, _3, _4, _5, _6, _7, _8, _9))) {
    std::cerr << "failed to initialize encoder!" << std::endl;
    return;
  }

  for (int64_t i = 0; i < numFrames; i++) {
    auto image = std::make_shared<Image>();
    image->encoding = "bgr8";
    image->width = enc_tester->getWidth();
    image->height = enc_tester->getHeight();
    image->step = image->width * 3;  // 3 bytes per pixel
    image->header.frame_id = enc_tester->getFrameId();
    image->is_bigendian = false;

    image->header.stamp = rclcpp::Time(i + 1, RCL_SYSTEM_TIME);
    image->data.resize(image->step * image->height, static_cast<uint8_t>(i));
    dec_tester->addImage(image);
    enc.encodeImage(*image);
  }
  enc.flush();
  dec_tester->finalize();
}

TEST(ffmpeg_encoder_decoder, encoder_msg)
{
  const int n = 10;
  EncoderTester tester("frame_id", "h264", 640, 480);
  test_encoder_msg(n, "libx264", &tester);
  // EXPECT_EQ(tester.getPacketSum(), 115852); varies between ffmpeg versions
  EXPECT_EQ(tester.getTsSum(), (n * (n + 1)) / 2);
  EXPECT_EQ(tester.getPtsSum(), (n * (n - 1)) / 2);
  EXPECT_EQ(tester.getPacketCounter(), n);
}

TEST(ffmpeg_encoder_decoder, encoder_decoder_msg)
{
  const int n = 10;
  const int w = 640;
  const int h = 480;
  EncoderTester enc_tester("frame_id", "hevc", w, h);
  DecoderTester dec_tester("frame_id", "hevc", w, h);
  test_encoder_decoder_msg(n, "libx265", "hevc", &enc_tester, &dec_tester);
  // EXPECT_EQ(enc_tester.getPacketSum(), 266674); differs with ffmpeg version
  EXPECT_EQ(enc_tester.getTsSum(), (n * (n + 1)) / 2);
  EXPECT_EQ(enc_tester.getPtsSum(), (n * (n - 1)) / 2);
  EXPECT_EQ(enc_tester.getPacketCounter(), n);
  EXPECT_EQ(dec_tester.getPacketCounter(), n);
  EXPECT_EQ(dec_tester.getFrameCounter(), n);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
