cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

list(APPEND CMAKE_MODULE_PATH /opt/homebrew/Cellar)

if(WIN32)
set(OpenCV_DIR D:/Soft/Opencv3.4/opencv/build/)
find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR} NO_DEFAULT_PATH)
else()
find_package(OpenCV REQUIRED)
endif()

find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS}/..)
set(CMAKE_CXX_STANDARD 17)

include_directories(${OpenCV_INCLUDE_DIRS};${EIGEN3_INCLUDE_DIR})

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
