// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

static const char * g_codec = "libx264";
static const char * g_frame_id = "frame_id";
static const int g_width = 1920;  // must be mult of 64 for some codecs!
static const int g_height = 1080;

static int g_frame_counter(0);

void packetReady(
  const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
  uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz)
{
  (void)stamp;
  (void)pts;
  (void)flags;
  (void)data;
  (void)sz;
  EXPECT_EQ(static_cast<int>(width), g_width);
  EXPECT_EQ(static_cast<int>(height), g_height);
  EXPECT_EQ(frame_id, g_frame_id);
  EXPECT_EQ(codec, g_codec);
  g_frame_counter++;
}

void test_encoder(int numFrames, const std::string & codec)
{
  ffmpeg_encoder_decoder::Encoder enc;
  enc.setEncoder(codec);
  enc.setProfile("main");
  enc.setPreset("slow");
  enc.setQMax(10);
  enc.setBitRate(8242880);
  enc.setGOPSize(2);
  enc.setFrameRate(100, 1);

  if (!enc.initialize(g_width, g_height, packetReady)) {
    std::cerr << "failed to initialize encoder!" << std::endl;
    return;
  }
  std_msgs::msg::Header header;
  header.frame_id = g_frame_id;
  for (int i = 0; i < numFrames; i++) {
    cv::Mat mat = cv::Mat::zeros(g_height, g_width, CV_8UC3);  // clear image
    cv::putText(
      mat, std::to_string(i), cv::Point(mat.cols / 2, mat.rows / 2), cv::FONT_HERSHEY_COMPLEX,
      2 /* font size */, cv::Scalar(255, 0, 0) /* col */, 2 /* weight */);
    const rclcpp::Time t = rclcpp::Clock().now();
    header.stamp = t;
    enc.encodeImage(mat, header, t);
  }
  enc.flush(header);
}

TEST(ffmpeg_encoder_decoder, encoder)
{
  test_encoder(10, g_codec);
  EXPECT_EQ(g_frame_counter, 10);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
