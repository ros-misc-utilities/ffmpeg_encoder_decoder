# ROS2 FFMPeg encoder/decoder

This ROS2 package supports encoding/decoding with the FFMpeg
library, for example encoding h264 and h265 or HEVC, using
Nvidia or other hardware acceleration when available.
This package is meant to be used by image transport plugins like
the [ffmpeg image transport](https://github.com/ros-misc-utilities/ffmpeg_image_transport/).

## Supported systems

Continuous integration is tested under Ubuntu with the following ROS2 distros:

 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__ffmpeg_encoder_decoder__ubuntu_jammy_amd64&subject=Humble)](https://build.ros2.org/job/Hdev__ffmpeg_encoder_decoder__ubuntu_jammy_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jdev__ffmpeg_encoder_decoder__ubuntu_noble_amd64&subject=Jazzy)](https://build.ros2.org/job/Jdev__ffmpeg_encoder_decoder__ubuntu_noble_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rdev__ffmpeg_encoder_decoder__ubuntu_noble_amd64&subject=Rolling)](https://build.ros2.org/job/Rdev__ffmpeg_encoder_decoder__ubuntu_noble_amd64/)


## Installation

### From packages

```bash
sudo apt-get install ros-${ROS_DISTRO}-ffmpeg-encoder-decoder
```

### From source

Set the following shell variables:
```bash
repo=ffmpeg_encoder_decoder
url=https://github.com/ros-misc-utilities/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

Make sure to source your workspace's ``install/setup.bash`` afterwards.

## ROS Parameters

This package does not expose ROS parameters. It is the upper layer's responsibility to e.g. manage the mapping between encoder and decoder, i.e. to tell the decoder class which libav decoder should be used for the decoding, or to set the encoding parameters.


## API usage

### Encoder

Using the encoder involves the following steps:
- instantiating the ``Encoder`` object.
- setting properties like the libav encoder to use, the encoding formats, and AV options.
- initializing the encoder object. This requires knowledge of the image size and therefore
  can only be done when the first image is available. Note that many properties
  (encoding formats etc) must have been set *before* initializing the encoder.
- feeding images to the encoder (and handling the callbacks when encoded packets become available)
- flushing the encoder (may result in additional callbacks with encoded packets)
- destroying the ``Encoder`` object.

The ``Encoder`` class description has a short example code snippet.

When using libav it is important to understand the difference between the encoder and the codec. The *codec* is the standardized format in which images are encoded, for instance ``h264`` or ``hevc``. The *encoder* is a software module that can encode images for a given codec. For instance ``libx264``, ``libx264rgb``, ``h264_nvenc``, and ``h264_vaapi`` are all *encoders* that encode for the *codec* h264.
Some of the encoders are hardware accelerated, some can handle more image formats than others, but in the end they all encode video for a specific codec. You set the libav encoder with the ``setEncoder()`` method.

For the many AV options available for various libav encoders, and for ``qmax``, ``bit_rate`` and similar settings please refer to the ffmpeg documentation.

The topic of "pixel formats" (also called "encodings") can be very confusing and frustrating, so here a few lines about it.
- Images in ROS arrive as messages that are encoded in one of the formats allowed by the [sensor\_msgs/Image encodings](https://github.com/ros2/common_interfaces/blob/a2ef867438e6d4eed074d6e3668ae45187e7de86/sensor_msgs/include/sensor_msgs/image_encodings.hpp). If you pass that message in via the ``encodeImage(const Image & msg)`` message, it will first will be converted to an opencv matrix using the [cv_bridge](https://github.com/ros-perception/vision_opencv).
If you don't set the ``cv_bridge_target_format`` via ``setCVBridgeTargetFormat(const std::string & fmt)``, the image will be converted to the default format of ``bgr8``. Depending on how performance sensitive you are (or if you have a ``mono8`` (gray) image!), this may not be what you want. Ideally, you should pick a ``cv_bridge_target_format`` that can be directly used by the libav decoder that you have chosen. You can use ffmpeg to list the formats that the libav encoder directly supports:
  ```
  ffmpeg -h encoder=libx264 | grep formats
  ```
  Note that the ffmpeg/libav format strings often diverge from the ROS encoding strings, so some guesswork and experimentation may be necessary to pick the right ``cv_bridge_target_format``.
  If you choose to bypass the cv\_bridge conversion by feeding the images to the encoder directly via the ``encodeImage(const cv::Mat & img ...)`` method, you must still set the ``cv_bridge_target_format`` such that encoder knows what format the ``img`` argument has.
- Once the image is available in ``cv_bridge_target_format`` the encoder may perform another conversion to an image format that the libav encoder supports, the ``av_source_pixel_format``. Again, ``ffmpeg -h encoder=<your encoder>`` shows the supported formats. If you don't set the ``av_source_pixel_format`` with ``setAVSourcePixelFormat()``, the encoder will try to pick one that is supported by the libav encoder. That often works but may not be optimal.
- Finally, the libav encoder produces a packet in its output codec, e.g. ``h264``, ``hevc`` etc. This encoding format is provided as the ``codec`` parameter when the encoder calls back with the encoded packet. Later, the codec needs to be provided to the decoder so it can pick a suitable libav decoder.

To summarize, the conversion goes as follows:
```
<ros_message> -> cv_bridge_target_format -> av_source_pixel_format --(libav encoder)--> codec
```
By properly choosing ``cv_bridge_target_format`` and ``av_source_pixel_format`` two of those conversions
may become no-ops, but to what extend the cv\_bridge and libav actually recognize and optimize for this has not been looked into yet.

Note that only very few combinations of libav encoders, ``cv_bridge_target_format`` and ``av_source_pixel_format`` have been tested. Please provide feedback if you observe crashes or find obvious bugs. PRs are always appreciated!

### Decoder

Using the decoder involves the following steps:
- instantiating the ``Decoder`` object.
- if so desired, setting the ROS output (image encoding) format.
- initializing the decoder object. For this you need to know the encoding (codec, e.g. "h264").
  During initialization you also have to present an ordered list of libav decoders that
  should be used. If an empty list is provided, the decoder will attempt to find a suitable
  libav decoder based on the encoding, with a preference for hardware accelerated decoding.
- feeding encoded packets to the decoder (and handling the callbacks
  when decoded images become available)
- flushing the decoder (may result in additional callbacks with decoded images)
- destroying the ``Decoder`` object.
The ``Decoder`` class description has a short example code snippet.

## How to use a custom version of libav (aka ffmpeg)

Compile *and install* ffmpeg. Let's say the install directory is
``/home/foo/ffmpeg/build``, then for it to be found while building,
run colcon like this:
```
colcon build --symlink-install --cmake-args --no-warn-unused-cli -DFFMPEG_PKGCONFIG=/home/foo/ffmpeg/build/lib/pkgconfig -DCMAKE_BUILD_TYPE=RelWithDebInfo 
```

This will compile against the right headers, but at runtime it may
still load the system ffmpeg libraries. To avoid that, set
``LD_LIBRARY_PATH`` at runtime:
```
export LD_LIBRARY_PATH=/home/foo/ffmpeg/build/lib:${LD_LIBRARY_PATH}
```

### How to use ffmpeg hardware accelerated encoding on the NVidia Jetson

Follow the instructions
[here](https://github.com/Keylost/jetson-ffmpeg) to build a version of
ffmpeg that supports NVMPI. This long magic line should build a nvmpi enabled
version of ffmpeg and install it under ``/usr/local/``:
```
git clone https://github.com/berndpfrommer/jetson-ffmpeg.git && cd jetson-ffmpeg && mkdir build && cd build && cmake -DCMAKE_INSTALL
_PREFIX:PATH=/usr/local .. && make install && ldconfig && cd ../.. && git clone git://source.ffmpeg.org/ffmpeg.git -b release/7.1 --
depth=1 &&cd jetson-ffmpeg && ./ffpatch.sh ../ffmpeg && cd ../ffmpeg && ./configure --enable-nvmpi --enable-shared --disable-static 
--prefix=/usr/local && make install
```

Then follow the section above on how to
actually use that custom ffmpeg library. As always first test on the
CLI that the newly compiled ``ffmpeg`` command now supports
``h264_nvmpi``. The transport can then be configured to use
nvmpi like so:

```
        parameters=[{'ffmpeg_image_transport.encoding': 'h264_nvmpi',
                     'ffmpeg_image_transport.profile': 'main',
                     'ffmpeg_image_transport.preset': 'll',
                     'ffmpeg_image_transport.gop_size': 15}]
```
Sometimes the ffmpeg parameters show up under different names. If the above
settings don't work, try the command ``ros2 param dump <name_of_your_node>``
*after* subscribing to the ffmpeg image topic with e.g. ``ros2 topic hz``.
From the output you can see what the correct parameter names are.
## License

This software is issued under the Apache License Version 2.0.
