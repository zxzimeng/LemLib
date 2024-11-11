cmake_minimum_required(VERSION 3.29)
project(LemLib)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)


# Collect all source files in the src directory
file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h" "include/*.hpp")

include_directories("include/")
# Add executable with all source files
add_executable(${PROJECT_NAME} ${SOURCES}
        src/
        include/lemlib/chassis/chassis.hpp
        include/lemlib/chassis/odom.hpp
        include/lemlib/api.hpp
        include/lemlib/driveCurve.hpp
        include/lemlib/pose.hpp
        include/lemlib/timer.hpp
        include/lemlib/util.hpp
        include/api.h
        include/main.h
        include/lemlib/chassis/chassis.hpp
        include/lemlib/api.hpp
        include/lemlib/driveCurve.hpp
        include/lemlib/pose.hpp
        include/lemlib/timer.hpp
        include/lemlib/util.hpp
        include/api.h
        include/main.h
        include/lemlib/chassis/extended_chassis.hpp
        src/lemlib/movement_utils.cpp
        include/lemlib/movement_utils.h)