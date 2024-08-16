# ROS2 FFMPeg encoder/decoder

This ROS2 package supports encoding/decoding with the FFMpeg
library, for example encoding h264 and h265 or HEVC, using
Nvidia or other hardware acceleration when available.
This package is meant to be used by image transport plugins like
the [ffmpeg image transport](https://github.com/ros-misc-utilities/ffmpeg_image_transport/).

## Supported systems

Continuous integration is tested under Ubuntu with the following ROS2 distros:

 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__ffmpeg_encoder_decoder__ubuntu_jammy_amd64&subject=Humble)](https://build.ros2.org/job/Hdev__ffmpeg_encoder_decoder__ubuntu_jammy_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Idev__ffmpeg_encoder_decoder__ubuntu_jammy_amd64&subject=Iron)](https://build.ros2.org/job/Idev__ffmpeg_encoder_decoder__ubuntu_jammy_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Idev__ffmpeg_encoder_decoder__ubuntu_jammy_amd64&subject=Jazzy)](https://build.ros2.org/job/Jdev__ffmpeg_encoder_decoder__ubuntu_noble_amd64/)
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

## Parameters

This package has no parameters. It is the upper layer's responsibility to e.g. manage the mapping between encoder and decoder, i.e. to tell the decoder class which libav decoder should be used for the decoding, or to set the encoding parameters.

### How to use a custom version of libav (aka ffmpeg)

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
[here](https://github.com/jocover/jetson-ffmpeg) to build a version of
ffmpeg that supports NVMPI. Then follow the section above on how to
actually use that custom ffmpeg library. As always first test on the
CLI that the newly compiled ``ffmpeg`` command now supports
``h264_nvmpi``. Afterwards you should be able to use e.g. the ``ffmpeg_image_transport`` with
parameters like so:

The transport can now be configured to use nvmpi like so:

```
        parameters=[{'ffmpeg_image_transport.encoding': 'h264_nvmpi',
                     'ffmpeg_image_transport.profile': 'main',
                     'ffmpeg_image_transport.preset': 'll',
                     'ffmpeg_image_transport.gop': 15}]
```


## License

This software is issued under the Apache License Version 2.0.
