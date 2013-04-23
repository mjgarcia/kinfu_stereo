cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

FIND_PACKAGE(PCL)
message(${PCL_VERSION})
message(${PCL_INCLUDE_DIRS})
message(${PCL_LIBRARIES})

FIND_PACKAGE(CUDA)
#message(${CUDA_VERSION})
#message(${CUDA_INCLUDE_DIRS})
#message(${PCL_LIBRARY_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES src/kinfu_stereo_app.cpp src/capture.cpp src/evaluation.cpp)

#common commands for building c++ executables and libraries
rosbuild_add_library(kinfu_stereo src/nodelets/depth_map.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(depth_map src/nodes/depth_map.cpp)

rosbuild_add_executable(kinfu_stereo_app ${SOURCES})
target_link_libraries(kinfu_stereo_app ${PCL_LIBRARIES})