cmake_minimum_required(VERSION 3.1)

SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(OpenCV REQUIRED)

SET(OpenCV_DIR /usr/local/Cellar/opencv/4.3.0_5/lib/cmake/opencv4)
INCLUDE_DIRECTORIES(
    ${OpenCV_INCLUDE_DIRS}
    includes)

project(vio)

set(SOURCES
  includes/StreamReader.hpp
  includes/HoughLines.hpp
  StreamReader.cpp
  HoughLines.cpp
  main.cc
)

add_executable(vio ${SOURCES})

set_property(TARGET vio PROPERTY CXX_STANDARD 17)
set_property(TARGET vio PROPERTY CXX_STANDARD_REQUIRED ON)
TARGET_LINK_LIBRARIES (vio ${OpenCV_LIBS})
