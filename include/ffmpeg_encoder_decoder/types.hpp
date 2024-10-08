// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef FFMPEG_ENCODER_DECODER__TYPES_HPP_
#define FFMPEG_ENCODER_DECODER__TYPES_HPP_

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace ffmpeg_encoder_decoder
{
using Header = std_msgs::msg::Header;
using Image = sensor_msgs::msg::Image;
using ImagePtr = Image::SharedPtr;
using ImageConstPtr = Image::ConstSharedPtr;
}  // namespace ffmpeg_encoder_decoder

#endif  // FFMPEG_ENCODER_DECODER__TYPES_HPP_
