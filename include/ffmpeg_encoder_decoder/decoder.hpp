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

#ifndef FFMPEG_ENCODER_DECODER__DECODER_HPP_
#define FFMPEG_ENCODER_DECODER__DECODER_HPP_

#include <ffmpeg_encoder_decoder/pts_map.hpp>
#include <ffmpeg_encoder_decoder/tdiff.hpp>
#include <ffmpeg_encoder_decoder/types.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/samplefmt.h>
#include <libswscale/swscale.h>
}

namespace ffmpeg_encoder_decoder
{
/**
 * \brief Decodes ffmpeg encoded messages via libav (ffmpeg).
 *
 * The Decoder class facilitates decoding of messages that have been encoded with the
 * Encoder class by leveraging libav, the collection of libraries used by ffmpeg.
 * Sample code:
 ```
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img, bool isKeyFrame)
{
  // process decoded image here...
}

ffmpeg_encoder_decoder::Decoder decoder;
ffmpeg_image_transport_msgs::msg::FFMPEGPacket msg;
msg.header.frame_id = "frame_id";
msg.width = 640;
msg.height = 480;
msg.encoding = "hevc";
msg.data.resize(10000, 0);  // Obviously this is not a valid packet!!!

if (!decoder.isInitialized()) {
  decoder.initialize(msg.encoding, imageCallback, "hevc_cuvid");
}

for (int64_t i = 0; i < 10; i++) {
  msg.header.stamp = rclcpp::Time(i, RCL_SYSTEM_TIME);
  if (!decoder.decodePacket(
        msg.encoding, &msg.data[0], msg.data.size(), msg.pts, msg.header.frame_id,
        msg.header.stamp)) {
    throw(std::runtime_error("error decoding packet!"));
  }
}
decoder.flush();
```
 */
class Decoder
{
public:
  /**
   * \brief callback function signature
   * \param img pointer to decoded image
   * \param isKeyFrame true if the decoded image is a keyframe
   */
  using Callback = std::function<void(const ImageConstPtr & img, bool isKeyFrame)>;

  /**
   * \brief Constructor.
   */
  Decoder();

  /**
   * \brief Destructor. Calls reset();
   */
  ~Decoder();

  /**
   * Test if decoder is initialized.
   * \return true if the decoder is initialized.
   */
  bool isInitialized() const { return (codecContext_ != NULL); }

  /**
   * \brief Initializes the decoder for a given codec and libav decoder.
   *
   * Initializes the decoder, with multiple decoders to pick from.
   * If the name of the libav decoder string is empty, a suitable libav decoder
   * will be picked, or the initialization will fail if none is available.
   * \param codec  the codec (encoding) from the first packet. Can never change!
   * \param callback the function to call when frame has been decoded.
   * \param decoder the name of the libav decoder to use. If empty string,
   *                 the decoder will try to find a suitable one based on the encoding.
   * \return true if initialized successfully.
   */
  bool initialize(const std::string & codec, Callback callback, const std::string & decoder);

  /**
   * \ brief initializes the decoder, trying different libavdecoders in order.
   *
   * Initialize decoder with multiple libav decoders to pick from.
   * If decoders.empty() a default decoder will be chosen (if available).
   * \param codec the codec (encoding) from the first packet. Can never change!
   * \param callback the function to call when frame has been decoded.
   * \param decoders names of the libav decoders to try sequentially. If empty()
   *                 the decoder will try to find a suitable one based on the codec.
   * \return true if successful
   */
  bool initialize(
    const std::string & codec, Callback callback, const std::vector<std::string> & decoders);

  /**
   * \brief Sets the ROS output message encoding format.
   *
   * Sets the ROS output message encoding format. Must be compatible with one of the
   * libav encoding formats, or else exception will be thrown. If not set,
   * the output encoding will default to bgr8.
   * \param output_encoding defaults to bgr8
   * \throw std::runtime_error() if no matching libav pixel format could be found
   */
  void setOutputMessageEncoding(const std::string & output_encoding);

  /**
   * \brief Clears all decoder state except for timers, loggers, and other settings.
   */
  void reset();

  /**
   * \brief Decodes encoded packet.
   *
   * Decodes packet. Decoder must have been initialized beforehand. Calling this
   * function may result in callback with decoded frame.
   * \param encoding  the name of the encoding/codec (typically from msg encoding)
   * \param data pointer to packet data
   * \param size size of packet data
   * \param pts presentation time stamp of data packet
   * \param frame_id ros frame id (from message header)
   * \param stamp ros message header time stamp
   * \return true if decoding was successful
   */
  bool decodePacket(
    const std::string & encoding, const uint8_t * data, size_t size, uint64_t pts,
    const std::string & frame_id, const rclcpp::Time & stamp);

  /**
   * \brief Flush decoder.
   *
   * This method can only be called once at the end of the decoding stream.
   * It will force any buffered packets to be delivered as frames. No further
   * packets can be fed to the decoder after calling flush().
   * \return true if flush was successful (libav returns EOF)
   */
  bool flush();

  /**
   * \brief Overrides the default ("Decoder") logger.
   * \param logger the logger to override the default ("Decoder") with.
   */
  void setLogger(rclcpp::Logger logger) { logger_ = logger; }

  /**
   * \brief Finds all hardware and software decoders for a given codec.
   *
   * Finds the name of all hardware and software decoders that match
   * a certain codec (or encoder).
   * \param codec name of the codec, i.e. h264, hevc etc
   * \param hw_decoders non-null pointer for returning list of hardware decoders
   * \param sw_decoders non-null pointer for returning list of software decoders
   */
  static void findDecoders(
    const std::string & codec, std::vector<std::string> * hw_decoders,
    std::vector<std::string> * sw_decoders);

  /**
   * \brief Finds all decoders that can decode a given codec.
   *
   * Finds the name of all hardware and software decoders (combined)
   * that match a certain codec (or encoder).
   * \param codec name of the codec, i.e. h264, hevc etc
   * \return vector with names of matching libav decoders
   */
  static std::vector<std::string> findDecoders(const std::string & codec);

  /**
   * \brief Enables or disables performance measurements. Poorly tested, may be broken.
   * \param p set to true to enable performance debugging.
   */
  void setMeasurePerformance(bool p) { measurePerformance_ = p; }

  /**
   * \brief Prints performance timers. Poorly tested, may be broken.
   * \param prefix for labeling the printout
   */
  void printTimers(const std::string & prefix) const;

  /**
   * \brief resets performance debugging timers. Poorly tested, may be broken.
   */
  void resetTimers();

  // ------------------- deprecated functions ---------------
  /**
   * \deprecated Use findDecoders(codec) instead.
   */
  [[deprecated("use findDecoders(codec) now.")]]
  static const std::unordered_map<std::string, std::string> & getDefaultEncoderToDecoderMap();

private:
  bool initSingleDecoder(const std::string & decoder);
  bool initDecoder(const std::vector<std::string> & decoders);
  int receiveFrame();
  // --------------- variables
  rclcpp::Logger logger_;
  Callback callback_;
  PTSMap ptsToStamp_;
  // --- performance analysis
  bool measurePerformance_{false};
  TDiff tdiffTotal_;
  // --- libav related variables
  AVRational timeBase_{1, 100};
  std::string encoding_;
  AVCodecContext * codecContext_{NULL};
  AVFrame * swFrame_{NULL};
  AVFrame * cpuFrame_{NULL};
  AVFrame * outputFrame_{NULL};
  SwsContext * swsContext_{NULL};
  enum AVPixelFormat hwPixFormat_;
  std::string outputMsgEncoding_;
  enum AVPixelFormat outputAVPixFormat_ { AV_PIX_FMT_NONE };
  int bitsPerPixel_;  // output format bits/pixel including padding
  AVPacket packet_;
  AVBufferRef * hwDeviceContext_{NULL};
};
}  // namespace ffmpeg_encoder_decoder

#endif  // FFMPEG_ENCODER_DECODER__DECODER_HPP_
