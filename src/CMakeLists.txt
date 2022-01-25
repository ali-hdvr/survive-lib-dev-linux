cmake_minimum_required(VERSION 3.12)
project(SpreeSurviveLibPoser VERSION 0.0.1)

file(GLOB survive_CXX CONFIGURE_DEPENDS "*.h" "*.cpp")


add_library(
  survive SHARED ${survive_CXX}
                    "${CMAKE_CURRENT_SOURCE_DIR}/../include/redist/linmath.cpp"
                    "${CMAKE_CURRENT_SOURCE_DIR}/../include/mpfit/mpfit.cpp"
                    )

message(STATUS "current source dir ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "bindir ${CMAKE_BINARY_DIR}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)


target_include_directories(survive PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

if(NOT DEFINED ANDROID_ABI)
  message(STATUS "ABI not defined")
  set_target_properties(
    # following properties can be set to path of corresponding android studio
    # folders   https://stackoverflow.com/a/6595001
    survive
    PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
               LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
               RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
else()
  message(STATUS "ABI ${ANDROID_ABI}")
  set_target_properties(
    # following properties can be set to path of corresponding android studio
    # folders   https://stackoverflow.com/a/6595001
    survive
    PROPERTIES
      ARCHIVE_OUTPUT_DIRECTORY
      "${CMAKE_CURRENT_SOURCE_DIR}/../redist/${ANDROID_ABI}"
      LIBRARY_OUTPUT_DIRECTORY
      "${CMAKE_CURRENT_SOURCE_DIR}/../redist/${ANDROID_ABI}"
      RUNTIME_OUTPUT_DIRECTORY
      "${CMAKE_CURRENT_SOURCE_DIR}/../redist/${ANDROID_ABI}"
  )
endif()

if(ANDROID_ABI STREQUAL arm64-v8a)

  message(STATUS "turn on NEON")
  set_target_properties(survive PROPERTIES COMPILE_FLAGS -mfpu=neon)
  add_compile_options(-mfpu=neon -mfloat-abi=hardfp -ftree-vectorize)
endif()
