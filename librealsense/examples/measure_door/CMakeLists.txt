# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.5.1)

project(RealsenseExamplesMeasure)
set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES comm
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(/usr/include/python2.7)

add_executable(subscribe_all subscribe_all.cpp)
target_link_libraries(subscribe_all ${catkin_LIBRARIES})


set(THREADS_PREFER_PTHREAD_FLAG ON)  #这两个语句是使用Threads必须要用到的两条语句
find_package(Threads REQUIRED)

set(OpenCV_DIR /usr/local/opencv-3.4.16/share/OpenCV)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv-3.4.16/include)
find_package(OpenCV 3.4 REQUIRED)    #寻找OPENCV包

add_executable(main main.cpp rs-measure.hpp ../example.hpp quaternions.cpp quaternions.h readPython.cpp readPython.h)
set_property(TARGET main PROPERTY CXX_STANDARD 11)
target_link_libraries(main ${OpenCV_LIBS} ${DEPENDENCIES} Threads::Threads /usr/lib/x86_64-linux-gnu/libpython2.7.so ${catkin_LIBRARIES})
include_directories(../)
set_target_properties (main PROPERTIES FOLDER "Examples") 

install(TARGETS main RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

