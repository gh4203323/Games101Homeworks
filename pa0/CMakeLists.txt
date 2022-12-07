cmake_minimum_required (VERSION 2.8.11)
project (Transformation)

#list(APPEND CMAKE_PREFIX_PATH /opt/homebrew/Cellar)
#find_package(Eigen3 REQUIRED)
include_directories("/opt/homebrew/Cellar/eigen/3.4.0_1/include")

add_executable (Transformation main.cpp)
