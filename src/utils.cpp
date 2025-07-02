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
#include <set>

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
  char buf[64];
  buf[63] = 0;
  av_get_pix_fmt_string(buf, sizeof(buf) - 1, f);
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
  const std::string & encoder, const std::vector<AVPixelFormat> & fmts)
{
  // the only format that worked for vaapi was NV12
  if (encoder.find("vaapi") != std::string::npos) {
    return (has_format(fmts, AV_PIX_FMT_NV12) ? AV_PIX_FMT_NV12 : AV_PIX_FMT_NONE);
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

std::vector<enum AVPixelFormat> get_encoder_formats(AVCodecContext * context, const AVCodec * c)
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

std::vector<enum AVPixelFormat> get_hwframe_transfer_formats(AVBufferRef * hwframe_ctx)
{
  std::vector<enum AVPixelFormat> formats;
  AVPixelFormat * fmts{nullptr};
  int ret =
    av_hwframe_transfer_get_formats(hwframe_ctx, AV_HWFRAME_TRANSFER_DIRECTION_FROM, &fmts, 0);
  if (ret >= 0) {
    for (const auto * f = fmts; *f != AV_PIX_FMT_NONE; f++) {
      formats.push_back(*f);
    }
  }
  return (formats);
}

// This function finds all encodings that are the target of a given
// encoder. So if encoder == "hevc_nvenc", it will return the set
// of {id_of(hevc)}

static std::set<AVCodecID> find_encodings_for_encoder(const std::string & encoder)
{
  std::set<AVCodecID> encodings;
  const AVCodecDescriptor * desc = NULL;
  while ((desc = avcodec_descriptor_next(desc)) != nullptr) {
    if (desc->name == encoder) {
      encodings.insert(desc->id);
    }
    const AVCodec * c;
    void * iter = nullptr;
    while ((c = av_codec_iterate(&iter))) {
      if (av_codec_is_encoder(c) && c->name == encoder && c->id == desc->id) {
        encodings.insert(c->id);
      }
    }
  }
  return (encodings);
}

static void find_decoders(
  const std::set<AVCodecID> & encodings, std::vector<std::string> * decoders, bool with_hw_support)
{
  const AVCodec * c;
  void * iter = nullptr;
  while ((c = av_codec_iterate(&iter))) {
    if (av_codec_is_decoder(c) && encodings.find(c->id) != encodings.end()) {
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

void find_decoders(
  const std::string & encoding, std::vector<std::string> * hw_decoders,
  std::vector<std::string> * sw_decoders)
{
  // in case the passed in encoding is actually an encoder...
  const auto encodings = find_encodings_for_encoder(encoding);
  // first use hw accelerated codecs, then software
  find_decoders(encodings, hw_decoders, true);
  find_decoders(encodings, sw_decoders, false);
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
