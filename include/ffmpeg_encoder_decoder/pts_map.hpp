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

#ifndef FFMPEG_ENCODER_DECODER__PTS_MAP_HPP_
#define FFMPEG_ENCODER_DECODER__PTS_MAP_HPP_

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

namespace ffmpeg_encoder_decoder
{
struct PTSMapEntry
{
  PTSMapEntry(const rclcpp::Time & t, const std::string & f) : time(t), frame_id(f) {}
  rclcpp::Time time;
  std::string frame_id;
};

using PTSMap = std::unordered_map<int64_t, PTSMapEntry>;
}  // namespace ffmpeg_encoder_decoder
#endif  // FFMPEG_ENCODER_DECODER__PTS_MAP_HPP_
