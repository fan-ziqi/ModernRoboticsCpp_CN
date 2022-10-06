cmake_minimum_required(VERSION 3.22)
project(ModernRobotics)

set(CMAKE_CXX_STANDARD 14)

include_directories(. Eigen MRLib)

add_library(MRLib SHARED ${CMAKE_CURRENT_SOURCE_DIR}/MRLib/modern_robotics.cpp ${CMAKE_CURRENT_SOURCE_DIR}/MRLib/modern_robotics.h)

add_executable(ModernRobotics
        main.cpp
        MRLib/MRTest.cpp
        )

target_link_libraries(ModernRobotics MRLib)