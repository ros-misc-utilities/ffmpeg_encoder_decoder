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
class Decoder
{
public:
  using Callback = std::function<void(const ImageConstPtr & img, bool isKeyFrame)>;
  using PTSMap = std::unordered_map<int64_t, rclcpp::Time>;

  Decoder();
  ~Decoder();
  /**
   * Test if decoder is initialized.
   * @return true if the decoder is initialized.
   */
  bool isInitialized() const { return (codecContext_ != NULL); }
  /**
   * Initialize decoder, with multiple decoders to pick from.
  *  Will pick hardware accelerated decoders first if available.
  * If decoders.empty() a default decoder will be chosen (if available).
  * @param encoding the encoding from the first packet. Can never change!
  * @param callback the function to call when frame has been decoded
  * @param decoder the decoder to use. If empty string,
  *                 the decoder will try to find a suitable one based on the encoding
  * @return true if successful
  */

  bool initialize(const std::string & encoding, Callback callback, const std::string & decoder);
  /**
   * Initialize decoder with multiple decoders to pick from.
  *  Will pick hardware accelerated decoders first if available.
  * If decoders.empty() a default decoder will be chosen (if available).
  * @param encoding the encoding from the first packet. Can never change!
  * @param callback the function to call when frame has been decoded
  * @param decoders the set of decoders to try sequentially. If empty()
  *                 the decoder will try to find a suitable one based on the encoding
  * @return true if successful
  */
  bool initialize(
    const std::string & encoding, Callback callback, const std::vector<std::string> & decoders);
  /**
   * Clears all decoder state but not timers, loggers, and other settings.
   */
  void reset();
  /**
   * Decodes packet. Decoder must have been initialized beforehand. Calling this
   * function may result in callback with decoded frame.
   * @param encoding  the name of the encoding (typically from msg encoding)
   * @param data pointer to packet data
   * @param size size of packet data
   * @param pts presentation time stamp of data packet
   * @param frame_id ros frame id (from message header)
   * @param stamp ros message header time stamp
  */
  bool decodePacket(
    const std::string & encoding, const uint8_t * data, size_t size, uint64_t pts,
    const std::string & frame_id, const rclcpp::Time & stamp);
  /**
   * Override default logger
   * @param logger the logger to override the default with
  */
  void setLogger(rclcpp::Logger logger) { logger_ = logger; }
  /**
   * deprecated, don't use!
   */
  static const std::unordered_map<std::string, std::string> & getDefaultEncoderToDecoderMap();
  /**
   *  Finds the name of hardware and software decoders that match a
   *  certain encoding (or encoder)
   */
  static void findDecoders(
    const std::string & encoding, std::vector<std::string> * hw_decoders,
    std::vector<std::string> * sw_decoders);
  /**
     * Finds the name of all hardware and software decoders that match
     * a certain encoding (or encoder)
     */
  static std::vector<std::string> findDecoders(const std::string & encoding);
  /**
   * For performance debugging
   */
  void setMeasurePerformance(bool p) { measurePerformance_ = p; }
  /**
   * For performance debugging
   */
  void printTimers(const std::string & prefix) const;
  /**
   * For performance debugging
   */
  void resetTimers();

private:
  rclcpp::Logger logger_;
  bool initDecoder(const std::string & decoder);
  bool initDecoder(const std::vector<std::string> & decoders);
  // --------------- variables
  Callback callback_;
  PTSMap ptsToStamp_;  // mapping of header

  // --- performance analysis
  bool measurePerformance_{false};
  TDiff tdiffTotal_;
  // --- libav stuff
  AVRational timeBase_{1, 100};
  std::string encoding_;
  AVCodecContext * codecContext_{NULL};
  AVFrame * decodedFrame_{NULL};
  AVFrame * cpuFrame_{NULL};
  AVFrame * colorFrame_{NULL};
  SwsContext * swsContext_{NULL};
  enum AVPixelFormat hwPixFormat_;
  AVPacket packet_;
  AVBufferRef * hwDeviceContext_{NULL};
};
}  // namespace ffmpeg_encoder_decoder

#endif  // FFMPEG_ENCODER_DECODER__DECODER_HPP_
