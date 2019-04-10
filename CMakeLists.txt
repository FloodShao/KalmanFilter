project(SensorFusion)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-wall")
set(CMAKE_CXX_FLAGS "${CSS_FLAGS}")

set(sources src/main.cpp
            src/usagecheck.cpp
            src/tools.cpp
            src/datapoint.cpp
            src/kalmanfilter.cpp
            src/fusionEKF.cpp
            src/fusionEKF.hpp)

add_executable(SensorFusion ${sources})
