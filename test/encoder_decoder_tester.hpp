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

#ifndef ENCODER_DECODER_TESTER_HPP_
#define ENCODER_DECODER_TESTER_HPP_
#include <ffmpeg_encoder_decoder/decoder.hpp>
#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <unordered_map>

class EncoderDecoderTester
{
public:
  using Image = sensor_msgs::msg::Image;
  using EncoderCallback = ffmpeg_encoder_decoder::Encoder::Callback;
  using FFMPEGPacket = ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
  explicit EncoderDecoderTester(size_t n = 10, uint32_t width = 640, uint32_t height = 480)
  : num_frames_(n), width_(width), height_(height)
  {
  }
  // ------- encoder side options
  void setNumFrames(int n) { num_frames_ = n; }
  void setOriginalROSEncoding(const std::string & s) { original_ros_encoding_ = s; }
  void setFinalROSEncoding(const std::string & s) { final_ros_encoding_ = s; }
  void setWidth(uint32_t n) { width_ = n; }
  void setHeight(uint32_t n) { height_ = n; }
  void setEncoder(const std::string & n) { enc_.setEncoder(n); }
  void setAVSourcePixelFormat(const std::string & f) { enc_.setAVSourcePixelFormat(f); }
  void setCVBridgeTargetFormat(const std::string & f) { enc_.setCVBridgeTargetFormat(f); }
  void addAVOption(const std::string & k, const std::string & v) { enc_.addAVOption(k, v); }
  void setExpectedPacketEncoding(const std::string & s) { expected_packet_encoding_ = s; }
  // ------- decoder side options
  void setDecoder(const std::string & s) { decoder_name_ = s; }
  void setDecoderExpectedAVPixFmt(const std::string & s) { decoder_av_pix_fmt_ = s; }

  // ------- getters
  const auto & getImageSum() const { return (image_sum_); }
  const auto & getTsSum() const { return (ts_sum_); }
  const auto & getFrameCounter() const { return (frame_counter_); }
  const auto & getPacketCounter() const { return (packet_counter_); }
  const auto & getPtsSum() const { return (pts_sum_); }

  void runTest();
  void check();

private:
  void packetReady(
    const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
    uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz);
  void addImage(const Image::ConstSharedPtr & img);
  void decodePacket(const FFMPEGPacket & msg);
  void imageCallback(
    const Image::ConstSharedPtr & img, bool /* isKeyFrame */, const std::string & av_pix_fmt);
  sensor_msgs::msg::Image::SharedPtr makeTestMessage();
  // ----------------- variables
  // --- encoder related
  ffmpeg_encoder_decoder::Encoder enc_;
  size_t num_frames_{10};
  std::string frame_id_{"frame_id"};
  std::string original_ros_encoding_;
  std::string encoder_name_;
  uint32_t width_{0};
  uint32_t height_{0};
  // --- decoder related
  ffmpeg_encoder_decoder::Decoder dec_;
  std::string decoder_name_;
  std::string final_ros_encoding_;
  std::string decoder_av_pix_fmt_;
  std::string expected_packet_encoding_;
  // --- statistics
  uint64_t image_sum_{0};
  int64_t ts_sum_{0};
  uint64_t frame_counter_{0};
  uint64_t packet_counter_{0};
  uint64_t pts_sum_{0};
  // --- misc
  std::unordered_map<int, Image::ConstSharedPtr> time_to_image_;
};

#endif  // ENCODER_DECODER_TESTER_HPP_
