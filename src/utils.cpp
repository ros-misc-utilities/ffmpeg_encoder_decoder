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

#include <algorithm>
#include <ffmpeg_encoder_decoder/utils.hpp>
#include <iostream>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <set>
#include <unordered_map>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/imgutils.h>
}

namespace ffmpeg_encoder_decoder
{
namespace utils
{
std::string pix(AVPixelFormat const & f)
{
  if (f == AV_PIX_FMT_NONE) {
    return (std::string("NONE"));
  }
  const char * name = av_get_pix_fmt_name(f);
  if (!name) {
    return (std::string("UNKNOWN"));
  }
  return (std::string(name));
}

std::string pix_long(AVPixelFormat const & f)
{
  char buf[64];
  buf[63] = 0;
  av_get_pix_fmt_string(buf, sizeof(buf) - 1, f);
  if (f == AV_PIX_FMT_NONE) {
    return (std::string("NONE"));
  }
  return (std::string(buf));
}

// solution from https://github.com/joncampbell123/composite-video-simulator/issues/5
std::string err(int errnum)
{
  char str[AV_ERROR_MAX_STRING_SIZE];
  return (av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum));
}

void throw_err(const std::string & msg, int errnum)
{
  throw(std::runtime_error(msg + ": " + err(errnum)));
}

void check_for_err(const std::string & msg, int errnum)
{
  if (errnum < 0) {
    throw_err(msg, errnum);
  }
}

enum AVHWDeviceType find_hw_device_type(const AVCodec * codec)
{
  if (codec->capabilities & AV_CODEC_CAP_HARDWARE) {
    const AVCodecHWConfig * config = avcodec_get_hw_config(codec, 0);
    if (config) {
      return (config->device_type);
    }
  }
  return (AV_HWDEVICE_TYPE_NONE);
}

enum AVPixelFormat find_hw_config(
  bool * usesHWFrames, enum AVHWDeviceType hwDevType, const AVCodec * codec)
{
  *usesHWFrames = false;
  for (int i = 0;; i++) {
    const AVCodecHWConfig * config = avcodec_get_hw_config(codec, i);
    if (!config) {
      return (AV_PIX_FMT_NONE);
    }
    if (
      ((config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) ||
       (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX)) &&
      config->device_type == hwDevType) {
      *usesHWFrames = (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX);
      return (config->pix_fmt);
    }
  }
  return (AV_PIX_FMT_NONE);
}

static bool has_format(const std::vector<AVPixelFormat> & fmts, const AVPixelFormat & f)
{
  return (std::find(fmts.begin(), fmts.end(), f) != fmts.end());
}

enum AVPixelFormat get_preferred_pixel_format(
  bool useHWFormat, const std::vector<AVPixelFormat> & fmts)
{
  if (useHWFormat) {
    // the hardware encoders typically use nv12.
    if (has_format(fmts, AV_PIX_FMT_NV12)) {
      return (AV_PIX_FMT_NV12);
    }
    if (has_format(fmts, AV_PIX_FMT_YUV420P)) {
      return (AV_PIX_FMT_YUV420P);
    }
    if (fmts.size() > 0) {
      return (fmts[0]);
    }
    return (AV_PIX_FMT_NONE);
  }
  if (has_format(fmts, AV_PIX_FMT_BGR24)) {
    return (AV_PIX_FMT_BGR24);  // fastest, needs no copy
  }
  if (has_format(fmts, AV_PIX_FMT_YUV420P)) {
    return (AV_PIX_FMT_YUV420P);  // needs no interleaving
  }
  if (has_format(fmts, AV_PIX_FMT_NV12)) {
    return (AV_PIX_FMT_NV12);  // needs interleaving :()
  }
  return (AV_PIX_FMT_NONE);
}

std::vector<enum AVPixelFormat> get_encoder_formats(
  const AVCodecContext * context, const AVCodec * c)
{
  std::vector<enum AVPixelFormat> formats;
  if (c) {
    const enum AVPixelFormat * pix_fmts = nullptr;
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(61, 13, 100)
    pix_fmts = c->pix_fmts;
    (void)context;
#else
    avcodec_get_supported_config(
      context, c, AV_CODEC_CONFIG_PIX_FORMAT, 0, (const void **)&pix_fmts, NULL);
#endif
    if (pix_fmts) {
      for (const auto * p = pix_fmts; *p != AV_PIX_FMT_NONE; ++p) {
        formats.push_back(*p);
      }
    }
  }
  return (formats);
}

std::vector<enum AVPixelFormat> get_hwframe_transfer_formats(
  AVBufferRef * hwframe_ctx, enum AVHWFrameTransferDirection direction)
{
  std::vector<enum AVPixelFormat> formats;
  AVPixelFormat * fmts{nullptr};
  int ret = av_hwframe_transfer_get_formats(hwframe_ctx, direction, &fmts, 0);
  if (ret >= 0) {
    for (const auto * f = fmts; *f != AV_PIX_FMT_NONE; f++) {
      formats.push_back(*f);
    }
  }
  if (fmts) {
    av_free(fmts);
  }
  return (formats);
}

static const std::unordered_map<std::string, enum AVPixelFormat> ros_to_av_pix_map = {
  {"bayer_rggb8", AV_PIX_FMT_BAYER_RGGB8},
  {"bayer_bggr8", AV_PIX_FMT_BAYER_BGGR8},
  {"bayer_gbrg8", AV_PIX_FMT_BAYER_GBRG8},
  {"bayer_grbg8", AV_PIX_FMT_BAYER_GRBG8},
  {"bayer_rggb16", AV_PIX_FMT_BAYER_RGGB16LE},  // map to little endian :(
  {"bayer_bggr16", AV_PIX_FMT_BAYER_BGGR16LE},
  {"bayer_gbrg16", AV_PIX_FMT_BAYER_GBRG16LE},
  {"bayer_grbg16", AV_PIX_FMT_BAYER_GRBG16LE},
  {"rgb8", AV_PIX_FMT_RGB24},
  {"rgba8", AV_PIX_FMT_RGBA},
  {"rgb16", AV_PIX_FMT_RGB48LE},
  {"rgba16", AV_PIX_FMT_RGBA64LE},
  {"bgr8", AV_PIX_FMT_BGR24},
  {"bgra8", AV_PIX_FMT_BGRA},
  {"bgr16", AV_PIX_FMT_BGR48LE},
  {"bgra16", AV_PIX_FMT_BGRA64LE},
  {"mono8", AV_PIX_FMT_GRAY8},
  {"mono16", AV_PIX_FMT_GRAY16LE},
  {"yuv422", AV_PIX_FMT_YUV422P},           // deprecated, not sure correct
  {"uyvy", AV_PIX_FMT_UYVY422},             // not sure that is correct
  {"yuyv", AV_PIX_FMT_YUYV422},             // not sure that is correct
  {"yuv422_yuy2", AV_PIX_FMT_YUV422P16LE},  // deprecated, probably wrong
  {"nv21", AV_PIX_FMT_NV21},
  {"nv24", AV_PIX_FMT_NV24},
  {"nv12", AV_PIX_FMT_NV12}  // not an official ROS encoding!!
};

enum AVPixelFormat ros_to_av_pix_format(const std::string & ros_pix_fmt)
{
  const auto it = ros_to_av_pix_map.find(ros_pix_fmt);
  if (it == ros_to_av_pix_map.end()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("encoder/decoder"),
      "no AV pixel format known for ros format " << ros_pix_fmt);
    throw(std::runtime_error("no matching pixel format found for: " + ros_pix_fmt));
  }
  return (it->second);
}

// find codec by name
static const AVCodec * find_by_name(const std::string & name)
{
  const AVCodec * c;
  void * iter = nullptr;
  while ((c = av_codec_iterate(&iter))) {
    if (c->name == name) {
      return (c);
    }
  }
  return (nullptr);
}

static void find_decoders(
  AVCodecID encoding, std::vector<std::string> * decoders, bool with_hw_support)
{
  const AVCodec * c;
  void * iter = nullptr;
  while ((c = av_codec_iterate(&iter))) {
    if (av_codec_is_decoder(c) && c->id == encoding) {
      if (with_hw_support) {
        if ((c->capabilities & AV_CODEC_CAP_HARDWARE) && (avcodec_get_hw_config(c, 0) != nullptr)) {
          decoders->push_back(c->name);
        }
      } else {
        if (!(c->capabilities & AV_CODEC_CAP_HARDWARE)) {
          decoders->push_back(c->name);
        }
      }
    }
  }
}

std::vector<std::string> split_by_char(const std::string & str_list, const char sep)
{
  std::stringstream ss(str_list);
  std::vector<std::string> split;
  for (std::string s; ss.good();) {
    getline(ss, s, sep);
    if (!s.empty()) {
      split.push_back(s);
    }
  }
  return (split);
}

std::vector<std::string> split_decoders(const std::string & decoder_list)
{
  return (split_by_char(decoder_list, ','));
}

std::vector<std::string> split_encoding(const std::string & encoding)
{
  return (split_by_char(encoding, ';'));
}

// This function finds the encoding that is the target of a given encoder.

static AVCodecID find_id_for_encoder_or_encoding(const std::string & encoder)
{
  // first look for encoder with that name
  const AVCodec * c = find_by_name(encoder);
  if (!c) {
    const AVCodecDescriptor * desc = NULL;
    while ((desc = avcodec_descriptor_next(desc)) != nullptr) {
      if (desc->name == encoder) {
        return (desc->id);
      }
    }
    throw(std::runtime_error("unknown encoder/encoding: " + encoder));
  }
  return (c->id);
}

void find_decoders(
  const std::string & encoding, std::vector<std::string> * hw_decoders,
  std::vector<std::string> * sw_decoders)
{
  // in case the passed in encoding is actually an encoder...
  const auto real_encoding = find_id_for_encoder_or_encoding(encoding);
  // first use hw accelerated codecs, then software
  find_decoders(real_encoding, hw_decoders, true);
  find_decoders(real_encoding, sw_decoders, false);
}

std::string find_decoders(const std::string & codec)
{
  std::string decoders;
  std::vector<std::string> hw_dec;
  std::vector<std::string> sw_dec;
  find_decoders(codec, &hw_dec, &sw_dec);
  for (const auto & s : hw_dec) {
    decoders += (decoders.empty() ? "" : ",") + s;
  }
  for (const auto & s : sw_dec) {
    decoders += (decoders.empty() ? "" : ",") + s;
  }
  return decoders;
}

std::string filter_decoders(const std::string & codec, const std::string & decoders)
{
  std::string filtered;
  const auto valid_decoders = split_decoders(find_decoders(codec));
  for (const auto & dec : split_decoders(decoders)) {
    bool found = false;
    for (const auto & valid : valid_decoders) {
      if (dec == valid) {
        filtered += (filtered.empty() ? "" : ",") + dec;
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("encoder/decoder"),
        "ignoring invalid decoder " << dec << " for codec " << codec);
    }
  }
  return (filtered);
}

std::string find_codec(const std::string & encoder)
{
  const AVCodec * c = find_by_name(encoder);
  if (!c) {
    throw(std::runtime_error("unknown encoder: " + encoder));
  }
  const AVCodecDescriptor * desc = NULL;
  while ((desc = avcodec_descriptor_next(desc)) != nullptr) {
    if (desc->id == c->id) {
      return (desc->name);
    }
  }
  throw(std::runtime_error("weird ffmpeg config error???"));
}

static bool isNV12LikeFormat(enum AVPixelFormat fmt)
{
  // any format where the top of the image is a full-resolution
  // luminance image, and the bottom 1/3 holds the color channels
  // should work
  return (fmt == AV_PIX_FMT_NV12 || fmt == AV_PIX_FMT_YUV420P);
}

bool encode_single_channel_as_color(const std::string & encoding, enum AVPixelFormat fmt)
{
  return (sensor_msgs::image_encodings::numChannels(encoding) == 1 && isNV12LikeFormat(fmt));
}

std::vector<std::string> get_hwdevice_types()
{
  std::vector<std::string> types;
  for (enum AVHWDeviceType type = av_hwdevice_iterate_types(AV_HWDEVICE_TYPE_NONE);
       type != AV_HWDEVICE_TYPE_NONE; type = av_hwdevice_iterate_types(type)) {
    types.push_back(av_hwdevice_get_type_name(type));
  }
  return (types);
}
}  // namespace utils
}  // namespace ffmpeg_encoder_decoder
