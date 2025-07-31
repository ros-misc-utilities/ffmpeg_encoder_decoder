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

#include "encoder_decoder_tester.hpp"

TEST(ffmpeg_encoder_decoder, encoder_decoder_bgr8)
{
  const std::string orig_enc = "bgr8";
  EncoderDecoderTester tester;
  // encoding options
  tester.setOriginalROSEncoding(orig_enc);
  tester.setEncoder("libx264rgb");
  tester.setCVBridgeTargetFormat(orig_enc);
  // encoder only supports bgr0, bgr24, rgb24
  // tester.setAVSourcePixelFormat("bgr24"); // automatically picked
  tester.addAVOption("crf", "0");  // needed for lossless
  // decoding options
  tester.setExpectedPacketEncoding("h264;bgr24;" + orig_enc + ";" + orig_enc);
  tester.setFinalROSEncoding(orig_enc);
  tester.setDecoder("h264");                  // codec == encoder in this case
  tester.setDecoderExpectedAVPixFmt("gbrp");  // picked by decoder
  tester.runTest();
  tester.check();
}

TEST(ffmpeg_encoder_decoder, encoder_decoder_mono)
{
  const std::string orig_enc = "mono8";
  EncoderDecoderTester tester;
  // encoding options
  tester.setOriginalROSEncoding(orig_enc);
  tester.setEncoder("libx265");
  tester.setCVBridgeTargetFormat(orig_enc);
  tester.setAVSourcePixelFormat("gray");
  tester.addAVOption("x265-params", "lossless=1");
  tester.setExpectedPacketEncoding("hevc;gray;" + orig_enc + ";" + orig_enc);
  tester.addAVOption("crf", "0");  // may not be needed for lossless
  // decoding options
  tester.setFinalROSEncoding(orig_enc);
  tester.setDecoder("hevc");
  tester.setDecoderExpectedAVPixFmt("gray");
  tester.runTest();
  tester.check();
}

TEST(ffmpeg_encoder_decoder, encoder_decoder_bayer)
{
  // tests the special hacks required for bayer encoding
  const std::string orig_enc = "bayer_rggb8";
  EncoderDecoderTester tester;
  // --- encoding options
  tester.setOriginalROSEncoding(orig_enc);
  tester.setEncoder("libx265");
  // setting target format no conversion
  tester.setCVBridgeTargetFormat(orig_enc);
  // encoder will automatically pick yuv420p,
  // then recognize that this requires single-channel to
  // color conversion, and do this losslessly.
  // tester.setAVSourcePixelFormat("yuv420p"); // not needed
  tester.addAVOption("x265-params", "lossless=1");
  tester.setExpectedPacketEncoding("hevc;yuv420p;" + orig_enc + ";" + orig_enc);
  // --- decoding options
  tester.setFinalROSEncoding(orig_enc);
  tester.setDecoder("hevc");
  tester.setDecoderExpectedAVPixFmt("yuv420p");
  tester.runTest();
  tester.check();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
