# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2021 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesGLNet)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

if(BUILD_GRAPHICAL_EXAMPLES AND NOT APPLE)
    #pointcloud
    add_executable(rs-gl-net rs-gl-net.cpp ../example.hpp)
    target_link_libraries(rs-gl-net ${DEPENDENCIES} realsense2-gl realsense2-net)
    include_directories(../)
    set_target_properties (rs-gl-net PROPERTIES FOLDER Examples)
    install(TARGETS rs-gl-net RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
