#
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(ffmpeg_encoder_decoder)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED imgproc)

if(${cv_bridge_VERSION} GREATER "3.3.0")
  add_definitions(-DUSE_CV_BRIDGE_HPP)
endif()

set(FFMPEG_PKGCONFIG "" CACHE STRING "extra path to pkgconfig")
if("${FFMPEG_PKGCONFIG}" STREQUAL "")
else()
  message(WARNING "using FFMPEG package from ${FFMPEG_PKGCONFIG}")
endif()

set(ENV{PKG_CONFIG_PATH} ${FFMPEG_PKGCONFIG})
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavcodec
    libswresample
    libswscale
    libavutil)


set(FFMPEG_PKGCONFIG "" CACHE STRING "extra path to pkgconfig")
if("${FFMPEG_PKGCONFIG}" STREQUAL "")
else()
  message(WARNING "using FFMPEG package from ${FFMPEG_PKGCONFIG}")
endif()

set(ENV{PKG_CONFIG_PATH} ${FFMPEG_PKGCONFIG})
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavcodec
    libswresample
    libswscale
    libavutil)

if(LIBAV_libavcodec_VERSION VERSION_GREATER_EQUAL 60.0.0)
  add_definitions(-DUSE_AV_FLAGS)
endif()


#
# --------- encoding/decoding library
#
add_library(${PROJECT_NAME}
  SHARED
  src/encoder.cpp
  src/decoder.cpp
  src/utils.cpp
  src/tdiff.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
PUBLIC
  cv_bridge
  rclcpp
  sensor_msgs)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    opencv_imgproc
    PkgConfig::LIBAV)


ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_export_dependencies(
  OpenCV
  LIBAV
  cv_bridge
  rclcpp
  sensor_msgs
  std_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_encoder_test test/encoder_test.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    target_include_directories(${PROJECT_NAME}_encoder_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_encoder_test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_package(CONFIG_EXTRAS cmake/${PROJECT_NAME}-extras.cmake.in)