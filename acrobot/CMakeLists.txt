cmake_minimum_required(VERSION 2.8.5)
PROJECT(AcrobotSim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET (CMAKE_CXX_STANDARD 11)
SET (CMAKE_CXX_STANDARD_REQUIRED ON)
SET (CMAKE_CXX_FLAGS "-O3")

SET(CMAKE_INSTALL_RPATH "/usr/local/lib")
SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

find_package (Eigen3 REQUIRED Dense)
message(${EIGEN3_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(test test.cpp)
link_directories(/usr/local/lib)
target_link_libraries(test
    iitgenacrobot
)
