cmake_minimum_required (VERSION 2.6)
project(VRCalibrator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/openvr/headers")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/kinect/v2.0_1409/inc")

link_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/openvr/lib/win64")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/Kinect/v2.0_1409/Lib/x64")

add_executable(VRCalibrator
  main.cpp
  samples.hpp
)

target_link_libraries(VRCalibrator "openvr_api.lib")
target_link_libraries(VRCalibrator "Kinect20.lib")