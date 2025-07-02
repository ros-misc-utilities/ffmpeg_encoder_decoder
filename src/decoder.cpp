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

#include <ffmpeg_encoder_decoder/decoder.hpp>
#include <ffmpeg_encoder_decoder/utils.hpp>
#include <fstream>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <unordered_map>

namespace ffmpeg_encoder_decoder
{

Decoder::Decoder() : logger_(rclcpp::get_logger("Decoder")) {}

Decoder::~Decoder() { reset(); }

void Decoder::reset()
{
  if (codecContext_) {
    avcodec_free_context(&codecContext_);
    codecContext_ = NULL;
  }
  if (swsContext_) {
    sws_freeContext(swsContext_);
    swsContext_ = NULL;
  }
  if (hwDeviceContext_) {
    av_buffer_unref(&hwDeviceContext_);
  }
  av_free(decodedFrame_);
  decodedFrame_ = NULL;
  av_free(cpuFrame_);
  cpuFrame_ = NULL;
  av_free(colorFrame_);
  colorFrame_ = NULL;
  hwPixFormat_ = AV_PIX_FMT_NONE;
}

bool Decoder::initialize(
  const std::string & encoding, Callback callback, const std::string & decoder)
{
  return (initialize(
    encoding, callback,
    decoder.empty() ? std::vector<std::string>() : std::vector<std::string>{decoder}));
}

bool Decoder::initialize(
  const std::string & encoding, Callback callback, const std::vector<std::string> & decoders)
{
  callback_ = callback;
  encoding_ = encoding;
  if (decoders.empty()) {
    const auto all_decoders = findDecoders(encoding);
    std::string decoders_str;
    for (const auto & decoder : all_decoders) {
      decoders_str += " " + decoder;
    }
    RCLCPP_INFO_STREAM(logger_, "trying discovered decoders in order:" << decoders_str);

    return (initDecoder(all_decoders));
  }
  return (initDecoder(decoders));
}

static AVBufferRef * hw_decoder_init(
  AVBufferRef ** hwDeviceContext, const enum AVHWDeviceType hwType, rclcpp::Logger logger)
{
  int rc = av_hwdevice_ctx_create(hwDeviceContext, hwType, NULL, NULL, 0);
  if (rc < 0) {
    RCLCPP_ERROR_STREAM(logger, "failed to create context for HW device: " << hwType);
    return (NULL);
  }
  RCLCPP_INFO_STREAM(logger, "using hardware acceleration: " << av_hwdevice_get_type_name(hwType));
  return (av_buffer_ref(*hwDeviceContext));
}

static enum AVPixelFormat find_pix_format(enum AVHWDeviceType hwDevType, const AVCodec * codec)
{
  for (int i = 0;; i++) {
    const AVCodecHWConfig * config = avcodec_get_hw_config(codec, i);
    if (!config) {
      return (AV_PIX_FMT_NONE);
    }
    if (
      config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
      config->device_type == hwDevType) {
      return (config->pix_fmt);
    }
  }
  return (AV_PIX_FMT_NONE);
}

// This function is a adapted version of avcodec_default_get_format

enum AVPixelFormat get_format(struct AVCodecContext * avctx, const enum AVPixelFormat * fmt)
{
  const AVPixFmtDescriptor * desc;
  const AVCodecHWConfig * config;
  int i, n;
#ifdef DEBUG_PIXEL_FORMAT
  if (avctx->codec) {
    printf("codec is: %s\n", avctx->codec->name);
  }
  char buf[64];
  buf[63] = 0;
  for (n = 0; fmt[n] != AV_PIX_FMT_NONE; n++) {
    av_get_pix_fmt_string(buf, sizeof(buf) - 1, fmt[n]);
    printf("offered pix fmt: %d = %s\n", fmt[n], buf);
  }
#endif
  // If a device was supplied when the codec was opened, assume that the
  // user wants to use it.
  if (avctx->hw_device_ctx && avcodec_get_hw_config(avctx->codec, 0)) {
    AVHWDeviceContext * device_ctx =
      reinterpret_cast<AVHWDeviceContext *>(avctx->hw_device_ctx->data);
    for (i = 0;; i++) {
      config = avcodec_get_hw_config(avctx->codec, i);
      if (!config) break;
      if (!(config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX)) continue;
      if (device_ctx->type != config->device_type) continue;
      for (n = 0; fmt[n] != AV_PIX_FMT_NONE; n++) {
        if (config->pix_fmt == fmt[n]) {
#ifdef DEBUG_PIXEL_FORMAT
          av_get_pix_fmt_string(buf, sizeof(buf) - 1, fmt[n]);
          printf("using pix fmt: %d = %s\n", fmt[n], buf);
#endif
          return fmt[n];
        }
      }
    }
  }
  // No device or other setup, so we have to choose from things which
  // don't any other external information.

  // If the last element of the list is a software format, choose it
  // (this should be best software format if any exist).

  for (n = 0; fmt[n] != AV_PIX_FMT_NONE; n++) {
  }
  desc = av_pix_fmt_desc_get(fmt[n - 1]);
  if (!(desc->flags & AV_PIX_FMT_FLAG_HWACCEL)) {
#ifdef DEBUG_PIXEL_FORMAT
    av_get_pix_fmt_string(buf, sizeof(buf) - 1, fmt[n - 1]);
    printf("using unaccelerated last fmt: %d = %s\n", fmt[n - 1], buf);
#endif
    return fmt[n - 1];
  }

  // Finally, traverse the list in order and choose the first entry
  // with no external dependencies (if there is no hardware configuration
  // information available then this just picks the first entry).
  for (n = 0; fmt[n] != AV_PIX_FMT_NONE; n++) {
    for (i = 0;; i++) {
      config = avcodec_get_hw_config(avctx->codec, i);
      if (!config) break;
      if (config->pix_fmt == fmt[n]) break;
    }
    if (!config) {
      // No specific config available, so the decoder must be able
      // to handle this format without any additional setup.
#ifdef DEBUG_PIXEL_FORMAT
      av_get_pix_fmt_string(buf, sizeof(buf) - 1, fmt[n]);
      printf("handle without setup %d = %s\n", fmt[n], buf);
#endif
      return fmt[n];
    }
    if (config->methods & AV_CODEC_HW_CONFIG_METHOD_INTERNAL) {
// Usable with only internal setup.
#ifdef DEBUG_PIXEL_FORMAT
      av_get_pix_fmt_string(buf, sizeof(buf) - 1, fmt[n]);
      printf("handle with internal setup %d = %s\n", fmt[n], buf);
#endif
      return fmt[n];
    }
  }
  // Nothing is usable, give up.
  return AV_PIX_FMT_NONE;
}

bool Decoder::initDecoder(const std::vector<std::string> & decoders)
{
  for (const auto & decoder : decoders) {
    const AVCodec * codec = avcodec_find_decoder_by_name(decoder.c_str());
    if (codec) {
      // use the decoder if it either is software, or has working
      // hardware support
      if (codec->capabilities & AV_CODEC_CAP_HARDWARE) {
        const AVCodecHWConfig * hwConfig = avcodec_get_hw_config(codec, 0);
        if (hwConfig) {
          if (initDecoder(decoder)) {
            return (true);
          }
        }
      } else {
        if (initDecoder(decoder)) {
          return (true);
        }
      }
    }
  }
  RCLCPP_ERROR_STREAM(logger_, "none of these requested decoders works: ");
  for (const auto & decoder : decoders) {
    RCLCPP_ERROR_STREAM(logger_, "  " << decoder);
  }
  throw(std::runtime_error("cannot find matching decoder!"));
}

bool Decoder::initDecoder(const std::string & decoder)
{
  try {
    // utils::get_decoders_for_encoding();  // initialize the map
    const AVCodec * codec = avcodec_find_decoder_by_name(decoder.c_str());
    if (!codec) {
      RCLCPP_ERROR_STREAM(logger_, "cannot find decoder " << decoder);
      throw(std::runtime_error("cannot find decoder " + decoder));
    }
    codecContext_ = avcodec_alloc_context3(codec);
    if (!codecContext_) {
      RCLCPP_ERROR_STREAM(logger_, "alloc context failed for " + decoder);
      codec = NULL;
      throw(std::runtime_error("alloc context failed!"));
    }
    av_opt_set_int(codecContext_, "refcounted_frames", 1, 0);
    enum AVHWDeviceType hwDevType = AV_HWDEVICE_TYPE_NONE;
    if (codec->capabilities & AV_CODEC_CAP_HARDWARE) {
      const AVCodecHWConfig * hwConfig = avcodec_get_hw_config(codec, 0);
      if (hwConfig) {
        hwDevType = hwConfig->device_type;
        RCLCPP_INFO_STREAM(
          logger_, "decoder " << decoder
                              << " has hw accel config: " << av_hwdevice_get_type_name(hwDevType));
      } else {
        RCLCPP_WARN_STREAM(logger_, "decoder " << decoder << " does not have hw accel config!");
      }
    } else {
      RCLCPP_INFO_STREAM(logger_, "decoder " << decoder << " has no hardware acceleration");
    }
    // default
    if (hwDevType != AV_HWDEVICE_TYPE_NONE) {
      codecContext_->hw_device_ctx = hw_decoder_init(&hwDeviceContext_, hwDevType, logger_);
      if (codecContext_->hw_device_ctx != NULL) {
        hwPixFormat_ = find_pix_format(hwDevType, codec);
        codecContext_->get_format = get_format;
      }
    }
    codecContext_->pkt_timebase = timeBase_;

    if (avcodec_open2(codecContext_, codec, NULL) < 0) {
      RCLCPP_ERROR_STREAM(logger_, "open context failed for " + decoder);
      av_free(codecContext_);
      codecContext_ = NULL;
      codec = NULL;
      throw(std::runtime_error("open context failed!"));
    }
    decodedFrame_ = av_frame_alloc();
    cpuFrame_ = (hwPixFormat_ == AV_PIX_FMT_NONE) ? NULL : av_frame_alloc();
    colorFrame_ = av_frame_alloc();
    colorFrame_->format = AV_PIX_FMT_BGR24;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, e.what());
    reset();
    return (false);
  }
  RCLCPP_INFO_STREAM(logger_, "decoding with " << decoder);
  return (true);
}

bool Decoder::decodePacket(
  const std::string & encoding, const uint8_t * data, size_t size, uint64_t pts,
  const std::string & frame_id, const rclcpp::Time & stamp)
{
  rclcpp::Time t0;
  if (measurePerformance_) {
    t0 = rclcpp::Clock().now();
  }
  if (encoding != encoding_) {
    RCLCPP_ERROR_STREAM(
      logger_, "no on-the fly encoding change from " << encoding_ << " to " << encoding);
    return (false);
  }
  AVCodecContext * ctx = codecContext_;
  AVPacket * packet = av_packet_alloc();
  av_new_packet(packet, size);  // will add some padding!
  memcpy(packet->data, data, size);
  packet->pts = pts;
  packet->dts = packet->pts;
  ptsToStamp_[packet->pts] = stamp;
  int ret = avcodec_send_packet(ctx, packet);
  if (ret != 0) {
    RCLCPP_WARN_STREAM(logger_, "send_packet failed for pts: " << pts);
    av_packet_unref(packet);
    return (false);
  }
  ret = avcodec_receive_frame(ctx, decodedFrame_);
  const bool isAcc = (ret == 0) && (decodedFrame_->format == hwPixFormat_);
  if (isAcc) {
    ret = av_hwframe_transfer_data(cpuFrame_, decodedFrame_, 0);
    if (ret < 0) {
      RCLCPP_WARN_STREAM(logger_, "failed to transfer data from GPU->CPU");
      av_packet_unref(packet);
      return (false);
    }
  }
  AVFrame * frame = isAcc ? cpuFrame_ : decodedFrame_;

  if (ret == 0 && frame->width != 0) {
    // convert image to something palatable
    if (!swsContext_) {
      swsContext_ = sws_getContext(
        ctx->width, ctx->height, (AVPixelFormat)frame->format,        // src
        ctx->width, ctx->height, (AVPixelFormat)colorFrame_->format,  // dest
        SWS_FAST_BILINEAR | SWS_ACCURATE_RND, NULL, NULL, NULL);
      if (!swsContext_) {
        RCLCPP_ERROR(logger_, "cannot allocate sws context!!!!");
        return (false);
      }
    }
    // prepare the decoded message
    ImagePtr image(new Image());
    image->height = frame->height;
    image->width = frame->width;
    image->step = image->width * 3;  // 3 bytes per pixel
    image->encoding = sensor_msgs::image_encodings::BGR8;
    image->data.resize(image->step * image->height);

    // bend the memory pointers in colorFrame to the right locations
    av_image_fill_arrays(
      colorFrame_->data, colorFrame_->linesize, &(image->data[0]),
      (AVPixelFormat)colorFrame_->format, frame->width, frame->height, 1);
    sws_scale(
      swsContext_, frame->data, frame->linesize, 0,            // src
      ctx->height, colorFrame_->data, colorFrame_->linesize);  // dest
    auto it = ptsToStamp_.find(decodedFrame_->pts);
    if (it == ptsToStamp_.end()) {
      RCLCPP_ERROR_STREAM(logger_, "cannot find pts that matches " << decodedFrame_->pts);
    } else {
      image->header.frame_id = frame_id;
      image->header.stamp = it->second;
      ptsToStamp_.erase(it);
#ifdef USE_AV_FLAGS
      callback_(image, decodedFrame_->flags || AV_FRAME_FLAG_KEY);  // deliver callback
#else
      callback_(image, decodedFrame_->key_frame);  // deliver callback
#endif
    }
  }
  av_packet_unref(packet);
  av_packet_free(&packet);
  if (measurePerformance_) {
    const auto t1 = rclcpp::Clock().now();
    double dt = (t1 - t0).seconds();
    tdiffTotal_.update(dt);
  }
  return (true);
}

void Decoder::resetTimers() { tdiffTotal_.reset(); }

void Decoder::printTimers(const std::string & prefix) const
{
  RCLCPP_INFO_STREAM(logger_, prefix << " total decode: " << tdiffTotal_);
}

const std::unordered_map<std::string, std::string> & Decoder::getDefaultEncoderToDecoderMap()
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("ffmpeg_decoder"),
    "default map is deprecated, use findDecoders() or pass empty string instead!");
  throw(std::runtime_error("default map is deprecated!"));
}

void Decoder::findDecoders(
  const std::string & encoding, std::vector<std::string> * hw_decoders,
  std::vector<std::string> * sw_decoders)
{
  utils::find_decoders(encoding, hw_decoders, sw_decoders);
}

std::vector<std::string> Decoder::findDecoders(const std::string & encoding)
{
  std::vector<std::string> sw_decoders, all_decoders;
  utils::find_decoders(encoding, &all_decoders, &sw_decoders);
  all_decoders.insert(all_decoders.end(), sw_decoders.begin(), sw_decoders.end());
  return (all_decoders);
}
}  // namespace ffmpeg_encoder_decoder
