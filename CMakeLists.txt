cmake_minimum_required(VERSION 3.12)
project(SpreeSurviveAll VERSION 0.0.1)

add_subdirectory(src)
add_subdirectory(unittest_live)
# add_subdirectory(unittest_rec)

# include(GNUInstallDirs)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()
