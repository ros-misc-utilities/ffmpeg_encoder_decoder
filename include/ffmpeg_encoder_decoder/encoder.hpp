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

#ifndef FFMPEG_ENCODER_DECODER__ENCODER_HPP_
#define FFMPEG_ENCODER_DECODER__ENCODER_HPP_

#include <ffmpeg_encoder_decoder/pts_map.hpp>
#include <ffmpeg_encoder_decoder/tdiff.hpp>
#include <ffmpeg_encoder_decoder/types.hpp>
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>
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
 * \brief Encodes ROS image messages via libav (ffmpeg).
 * 
 * The Encoder class facilitates encoding of ROS images by leveraging libav,
 * the collection of libraries used by ffmpeg.
 * Sample code:
 ```
  // handle the encoded packet in this function
  void packetReady(
     const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
     uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz) {
       // do something useful here ...
  }

  ffmpeg_encoder_decoder::Encoder enc;
  enc.setEncoder("libx265"); // set libav encoder
  // note: will not be lossless unless we happen to have right pixel format
  enc.setAVOption("x265-params", "lossless=1");
  enc.setAVOption("crf", "0");

  sensor_msgs::msg::Image image;
  image.encoding = "bgr8";
  image.width = image_width;
  image.height = image_height;
  image.step = image.width * 3;  // 3 bytes per pixel
  image.header.frame_id = "frame_id";
  image.is_bigendian = false;

  if (!enc.initialize(image.width, image.height, callback_fn, image.encoding)) {
    std::cerr << "failed to initialize encoder!" << std::endl;
    exit (-1);
  }

  for (int64_t i = 0; i < numFrames; i++) {
    image.header.stamp = rclcpp::Time(i + 1, RCL_SYSTEM_TIME);
    image.data.resize(image.step * image.height, static_cast<uint8_t>(i));
    enc.encodeImage(image); // may produce callback
  }
  enc.flush(); // may produce callback
}
  ```
 */
class Encoder
{
public:
  /**
  * \brief defines callback function signature.
  *
  * The encoder will call this function when an encoded packet is ready.
  * \param frame_id the frame_id of the message that produced the packet
  * \param stamp the ROS time stamp of the message that produced the packet
  * \param codec the codec used for compression: h264, hevc
  * \param width image width
  * \param height image height
  * \param pts libav presentation time stamp (pts)
  * \param data pointer to encoded packet
  * \param sz size of encoded packet
  */
  using Callback = std::function<void(
    const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
    uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz)>;

  /**
   * \brief Constructor. Doesn't open codec or do anything else dangerous.
   */
  Encoder();
  /**
   * \brief Destructor. Clears all state and frees all memory.
   */
  ~Encoder();
  /**
   * \brief sets the name of libav encoder to be used
   * \param encoder name of the libav encoder, e.g. libx264, hevc_nvenc etc
   */
  void setEncoder(const std::string & encoder_name);
  /**
   * \brief adds AVOption setting to list of options to be applied before opening the encoder
   * \param key   name of AVOption to set, e.g. "preset"
   * \param value value of AVOption e.g. "slow"
   */
  void addAVOption(const std::string & key, const std::string & value)
  {
    Lock lock(mutex_);
    avOptions_.push_back({key, value});
  }

  /**
   * This is the format of the image that will be fed into the encoder.
   * In the README, this is referred to as av_source_pixel_format.
   *
   * If your ROS message format (cv_bridge_target_format) is different,
   * the image will first be converted to the format you set here.
   * Must be set before calling initialize(), or else the encoder
   * will try to pick some suitable format. See README for more
   * details about formats.
   *
   * \param fmt The pixel format in libav naming convention, e.g. "yuv420p"
   */
  void setAVSourcePixelFormat(const std::string & fmt)
  {
    Lock lock(mutex_);
    pixFormat_ = pixelFormat(fmt);
  }
  /**
   * \brief Set cv_bridge_target_format for first image conversion
   *
   * Sets the target format of the first image conversion done via
   * cv_bridge (cv_bridge_target_format). See explanation in the
   * README file. Must be set before calling initialize(), or else
   * it will default to "bgr8". The cv_bridge_target_format must
   * be compatible with some AV_PIX_FMT* type, or else an exception
   * will be thrown when calling initialize().
   *
   * \param fmt cv_bridge_target_format (see ROS image_encodings.hpp header)
   */
  void setCVBridgeTargetFormat(const std::string & fmt)
  {
    Lock lock(mutex_);
    cvBridgeTargetFormat_ = fmt;
  }

  /**
   * \brief Sets q_max parameter. See ffmpeg docs for explanation.
   *
   * \param q q_max parameter
   */
  void setQMax(int q)
  {
    Lock lock(mutex_);
    qmax_ = q;
  }
  /**
   * \brief Sets bitrate parameter. See ffmpeg docs for explanation.
   * \param bitrate bitrate parameter (in bits/s)
   */
  void setBitRate(int bitrate)
  {
    Lock lock(mutex_);
    bitRate_ = bitrate;
  }
  /**
   * \brief gets current bitrate parameter.
   * \return the bitrate in bits/sec
   */
  int getBitRate() const
  {
    Lock lock(mutex_);
    return (bitRate_);
  }

  /**
   * \brief Sets gop size (max distance between keyframes). See ffmpeg docs.
   * \param gop_size max distance between keyframes
   */
  void setGOPSize(int g)
  {
    Lock lock(mutex_);
    GOPSize_ = g;
  }
  /**
   * \brief gets current gop size.
   * \return the gop_size (max distance between keyframes)
   */
  int getGOPSize() const
  {
    Lock lock(mutex_);
    return (GOPSize_);
  }
  /**
    * \brief sets maximum number of b-frames. See ffmpeg docs.
    * \param b max number of b-frames.
    */
  void setMaxBFrames(int b)
  {
    Lock lock(mutex_);
    maxBFrames_ = b;
  }
  /**
    * \brief gets maximum number of b-frames. See ffmpeg docs.
    * \return max number of b-frames.
    */
  int getMaxBFrames() const
  {
    Lock lock(mutex_);
    return (maxBFrames_);
  }
  /**
   * \brief Sets encoding frame rate as a rational number.
   *
   * Not sure what it really does.
   *
   * \param frames the number of frames during the interval
   * \param second the time interval (integer seconds)
   */
  void setFrameRate(int frames, int second)
  {
    Lock lock(mutex_);
    frameRate_.num = frames;
    frameRate_.den = second;
    timeBase_.num = second;
    timeBase_.den = frames;
  }
  /**
   * \brief enables or disables performance measurements. Poorly tested, hardly used.
   *
   * \param p true enables performance statistics
   */
  void setMeasurePerformance(bool p)
  {
    Lock lock(mutex_);
    measurePerformance_ = p;
  }
  // ------- related to teardown and startup
  /**
   * \brief test if encoder is initialized
   * \return true if encoder is initialized
   */
  bool isInitialized() const
  {
    Lock lock(mutex_);
    return (codecContext_ != NULL);
  }
  /**
   * \brief initializes encoder.
   * \param width image width
   * \param height image height
   * \param callback the function to call for handling encoded packets
   * \param encoding the ros encoding string, e.g. bayer_rggb8, rgb8 ...
   */
  bool initialize(
    int width, int height, Callback callback, const std::string & encoding = std::string());
  /**
   * \brief sets ROS logger to use for info/error messages
   * \param logger the logger to use for messages
   */
  void setLogger(rclcpp::Logger logger) { logger_ = logger; }
  /**
   * \brief completely resets the state of the encoder.
   */
  void reset();
  /**
   * \brief encodes image message. May produce callbacks.
   * \param msg the image message to encode
   */
  void encodeImage(const Image & msg);
  /**
  * \brief flush all packets (produces callbacks).
  */
  void flush();
  /**
   * \brief finds the codec for a given encoder, i.e. returns h264 for h264_vaapi
   * \param encoder name of libav encoder
   * \return codec (libav naming convention)
   */
  static std::string findCodec(const std::string & encoder);

  // ------- performance statistics
  void printTimers(const std::string & prefix) const;
  void resetTimers();
  // ------- deprecated functions
  /**
   * \deprecated use addAVOption("profile", "value") now.
   */
  [[deprecated("use addAVOption() instead.")]] void setProfile(const std::string & p)
  {
    addAVOption("profile", p);
  }
  /**
   * \deprecated use addAVOption("preset", "value") now.
   */
  [[deprecated("use addAVOption() instead.")]] void setPreset(const std::string & p)
  {
    addAVOption("preset", p);
  }
  /**
   * \deprecated use addAVOption("tune", "value") now.
   */
  [[deprecated("use addAVOption() instead.")]] void setTune(const std::string & p)
  {
    addAVOption("tune", p);
  }
  /**
   * \deprecated use addAVOption("delay", "value") now.
   */
  [[deprecated("use addAVOption() instead.")]] void setDelay(const std::string & p)
  {
    addAVOption("delay", p);
  }
  /**
   * \deprecated use addAVOption("crf", "value") now.
   */
  [[deprecated("use addAVOption() instead.")]] void setCRF(const std::string & c)
  {
    addAVOption("crf", c);
  }
  /**
   * \deprecated Use setAVSourcePixelFormat()
   */
  [[deprecated("use setAVSourcePixelFormat(fmt) instead.")]] void setPixelFormat(
    const std::string & fmt)
  {
    setAVSourcePixelFormat(fmt);
  }
  /**
   * \brief encodes image into ffmpeg message. May produce callbacks.
   * \param img openCV matrix representing image to be encoded
   * \param header frame_id and stamp are used to generate ffmpeg packet message
   * \param t0 start time for performance timing. Set to rclcpp::Clock().now()
   */
  void encodeImage(
    const cv::Mat & img, const Header & header, const rclcpp::Time & t0 = rclcpp::Clock().now());
  /**
  * flush all packets (produces callbacks).
  * \deprecated Only header.frame_id is used. Used flush(frame_id) now.
  */
  [[deprecated("use flush() instead.")]] void flush(const Header & header);

private:
  using Lock = std::unique_lock<std::recursive_mutex>;

  bool openCodec(int width, int height, const std::string & encoding);
  void doOpenCodec(int width, int height, const std::string & encoding);
  void closeCodec();
  void doEncodeImage(const cv::Mat & img, const Header & header, const rclcpp::Time & t0);
  int drainPacket(int width, int height);
  AVPixelFormat pixelFormat(const std::string & f) const;
  void openHardwareDevice(
    const AVCodec * codec, enum AVHWDeviceType hwDevType, int width, int height);
  void setAVOption(const std::string & field, const std::string & value);
  enum AVPixelFormat findMatchingSourceFormat(
    const std::string & rosSrcFormat, enum AVPixelFormat targetFormat);

  // --------- variables
  rclcpp::Logger logger_;
  mutable std::recursive_mutex mutex_;
  Callback callback_;
  // config
  std::string encoder_;   // e.g. "libx264"
  std::string codec_;     // e.g. "h264"
  std::string encoding_;  // e.g. "h264/rgb8"
  int qmax_{-1};          // max allowed quantization. The lower the better quality
  int GOPSize_{-1};       // distance between two keyframes
  int maxBFrames_{-1};    // maximum number of b-frames
  int64_t bitRate_{0};    // max rate in bits/s
  std::vector<std::pair<std::string, std::string>> avOptions_;

  AVPixelFormat pixFormat_{AV_PIX_FMT_NONE};
  std::string cvBridgeTargetFormat_ = "bgr8";
  AVRational timeBase_{1, 100};
  AVRational frameRate_{100, 1};
  bool usesHardwareFrames_{false};
  std::string avSourcePixelFormat_;
  // ------ libav state
  AVCodecContext * codecContext_{nullptr};
  AVBufferRef * hwDeviceContext_{nullptr};
  AVFrame * frame_{nullptr};
  AVFrame * hw_frame_{nullptr};
  AVPacket * packet_{nullptr};
  // ------ libswscale state
  AVFrame * wrapperFrame_{nullptr};
  SwsContext * swsContext_{NULL};
  // ---------- other stuff
  int64_t pts_{0};
  PTSMap ptsToStamp_;
  // ---------- performance analysis
  bool measurePerformance_{true};
  int64_t totalInBytes_{0};
  int64_t totalOutBytes_{0};
  unsigned int frameCnt_{0};
  TDiff tdiffUncompress_;
  TDiff tdiffEncode_;
  TDiff tdiffDebayer_;
  TDiff tdiffFrameCopy_;
  TDiff tdiffSendFrame_;
  TDiff tdiffReceivePacket_;
  TDiff tdiffCopyOut_;
  TDiff tdiffPublish_;
  TDiff tdiffTotal_;
};
}  // namespace ffmpeg_encoder_decoder
#endif  // FFMPEG_ENCODER_DECODER__ENCODER_HPP_
