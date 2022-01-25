cmake_minimum_required(VERSION 3.12)
project(SurviveLibIOLinux VERSION 0.0.1)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../survive-lib/src" "${CMAKE_CURRENT_SOURCE_DIR}/../survive-lib/build")
add_subdirectory(unittest_live)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()
