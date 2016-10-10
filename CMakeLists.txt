cmake_minimum_required (VERSION 2.6)
project(VRCalibrator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/openvr/headers")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/kinect/v2.0_1409/inc")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/glm")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/armadillo/include")

link_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/openvr/lib/win64")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/Kinect/v2.0_1409/Lib/x64")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/externals/armadillo/examples/lib_win64")

add_executable(VRCalibrator
  main.cpp
  samples.hpp
)

target_link_libraries(VRCalibrator "openvr_api.lib")
target_link_libraries(VRCalibrator "Kinect20.lib")
target_link_libraries(VRCalibrator "blas_win64_MT.lib")
target_link_libraries(VRCalibrator "lapack_win64_MT.lib")

add_custom_target(CopyBinaries
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/externals/openvr/bin/win64/openvr_api.dll ${CMAKE_BINARY_DIR}/Release
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/externals/armadillo/examples/lib_win64/blas_win64_MT.dll ${CMAKE_BINARY_DIR}/Release
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/externals/armadillo/examples/lib_win64/lapack_win64_MT.dll ${CMAKE_BINARY_DIR}/Release
)
add_dependencies(VRCalibrator CopyBinaries)