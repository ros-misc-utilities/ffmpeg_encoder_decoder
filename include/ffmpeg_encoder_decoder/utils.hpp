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

#ifndef FFMPEG_ENCODER_DECODER__UTILS_HPP_
#define FFMPEG_ENCODER_DECODER__UTILS_HPP_

#include <map>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
}

namespace ffmpeg_encoder_decoder
{
namespace utils
{
/**
 * \brief Convert av pixel format to clear text string.
 * \return string with name of pixel format
 */
std::string pix(const AVPixelFormat & f);

/**
 * \brief Long descriptor of av pixel format
 * \return string with long name of pixel format
 */
std::string pix_long(AVPixelFormat const & f);

/**
 * \brief Convert av error number to string.
 * \param errnum libav error number
 * \return clear text string with error message
 */
std::string err(int errnum);

/**
 * \brief throws runtime_error() with decoded av error string
 * \param msg message (free form text)
 * \param errnum libav error number
 */
void throw_err(const std::string & msg, int errnum);

/**
 * \brief checks for error and throws runtime_error() with av error string
 * \param msg error message (free form)
 * \param errnum libav error number
 */
void check_for_err(const std::string & msg, int errnum);

/**
 * \brief finds hardware configuration.
 *
 * Finds hardware configuration, in particular the target pixel format
 * and whether the encoder uses hardware frame upload.
 * \param usesHWFrames will be set to true if hw frames are used
 * \param hwDevType the hardware device type to probe get the config for
 * \param codec the codec to get the config for
 * \return the hardware pixel format to use, or AV_PIX_FMT_NONE
 */
enum AVPixelFormat find_hw_config(
  bool * usesHWFrames, enum AVHWDeviceType hwDevType, const AVCodec * codec);

/**
 * \brief Gets all pixel formats that can be transferred to the hardware device
 * \param hwframe_ctx the hardware frame context for which transfer is intended
 * \return vector of allowed pixel formats
 */
std::vector<enum AVPixelFormat> get_hwframe_transfer_formats(
  AVBufferRef * hwframe_ctx, enum AVHWFrameTransferDirection direction);

/**
 * \brief finds all formats that the encoder supports.
 *
 * Note that for VAAPI, this will justreturn AV_PIX_FMT_VAAPI since it uses hardware frames.
 * \param @return vector with pixel formats supported by codec in this context
 */
std::vector<enum AVPixelFormat> get_encoder_formats(
  const AVCodecContext * context, const AVCodec * avctx);

/**
 * \brief gets the preferred pixel format from list.
 * \param useHWFormat if true only return hardware accelerated formats
 * \param fmts vector of formats to choose from
 * \return preferred pixel format
 */
enum AVPixelFormat get_preferred_pixel_format(
  bool useHWFormat, const std::vector<AVPixelFormat> & fmts);

/**
 * \brief finds the names of all available decoders
 *        for a given codec (or encoder)
 * \param codec the codec / encoding to find decoders for
 * \param hw_decoders (output) non-null ptr to hardware decoders
 * \param sw_decoders (output) non-null ptr to software decoders
 */
void find_decoders(
  const std::string & codec, std::vector<std::string> * hw_decoders,
  std::vector<std::string> * sw_decoders);

/**
 * \brief finds the names of all available decoders
 *        for a given codec (or encoder)
 * \param codec the codec / encoding to find decoders for
 * \return string with comma separated list of libav decoder names
 */

std::string find_decoders(const std::string & codec);
/**
 * \brief filters a string with comma-separated decoders and
 *
 * \param codec the codec / encoding to filter for
 * \param decoders string with comma-separated list of decoder names
 * \return string with comma separated list of libav decoder names
 */
std::string filter_decoders(const std::string & codec, const std::string & decoders);

/**
 * \brief gets list of names of all libav supported device types
 *
 * This is not the list of devices present on the host machine, just
 * the ones that libav supports, whether they are present or not.
 * \return list of all libav supported device types
 */
std::vector<std::string> get_hwdevice_types();

/**
 * \brief find codec for a given encoder
 * \param encoder name of libav encoder
 * \return name of codec
 */
std::string find_codec(const std::string & encoder);

/**
 * \brief gets hardware device type for specific codec
 * \param codec pointer to libav codec
 * \param return associated hardware device type
 */
enum AVHWDeviceType find_hw_device_type(const AVCodec * codec);

/**
 * \brief finds AVPixelFormat corresponding to ROS encoding
 * \param ros_pix_fmt ros encoding name, e.g. "bgr8"
 * \return corresponding AV pixel format
 * \throws std::runtime_error exception when no match found.
 */
enum AVPixelFormat ros_to_av_pix_format(const std::string & ros_pix_fmt);

/**
 * \brief splits string by character
 * \param str_list character-separated list of tokens
 * \param sep separator character
 * \return vector of separated strings
 */
std::vector<std::string> split_by_char(const std::string & str_list, const char sep);

/**
 * \brief determine if given ROS single-channel encoding fits into libav color format
 * \param encoding ROS message encoding, e.g. bayer_rggb8
 * \param fmt libav color format to test for
 * \return true if ROS encoding is single channel and color format is like NV12/yuv420p
 */
bool encode_single_channel_as_color(const std::string & encoding, enum AVPixelFormat fmt);

/**
 * \brief splits a string with a list of decoders
 * \param decoder_list comma-separated list of decoders
 * \return vector of separated strings
 */
std::vector<std::string> split_decoders(const std::string & decoder_list);

/**
 * \brief splits a string with an encoding ("codec;fmt;fmt;fmt")
 * \param encoding string with encoding information
 * \return vector of separated strings
 */
std::vector<std::string> split_encoding(const std::string & encoding);

}  // namespace utils
}  // namespace ffmpeg_encoder_decoder
#endif  // FFMPEG_ENCODER_DECODER__UTILS_HPP_
