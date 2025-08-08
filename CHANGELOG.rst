^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ffmpeg_encoder_decoder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* added utility functions for splitting
* better error message when filtering decoder
* use mutex to make thread safe
* improved documentation
* use comma-separated decoder list now, use semi-colon to separate encodings
* new feature: can set AV options for decoder
* fix uninitialized memory bug
* reworked tests to cover bayer
* fix debug printout for AV_PIX_FMT_NONE
* filter misconfigured decoders
* fix bug with non-accel pix fmt
* fix nvenc encoding by looking at transfer TO formats
* decoder flush(), decoder testing, improved api docs
* new interfaces to support decoder fallback, encoder avoids cuda and vaapi hard coding, improved flush
* Contributors: Bernd Pfrommer

2.0.1 (2025-05-26)
------------------
* avoid ament_target_dependencies
* When using CMake >= 3.24 use CMAKE_COMPILE_WARNING_AS_ERROR variable
* only build on most recent distros
* Fix deprecated libavcodec (`#1 <https://github.com/ros-misc-utilities/ffmpeg_encoder_decoder/issues/1>`_)
  * fix deprecated features for libav 7
  * fix formatting errors
* Contributors: Bernd Pfrommer, Silvio Traversaro

2.0.0 (2025-03-15)
------------------
* added CRF and updated docs
* Contributors: Bernd Pfrommer

1.0.1 (2024-08-30)
------------------
* initial commit
* Contributors: Bernd Pfrommer
