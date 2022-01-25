project(SpreeSurviveLibUT VERSION 0.0.1)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../include"
  "${CMAKE_CURRENT_SOURCE_DIR}/../includeut"
  "${CMAKE_CURRENT_SOURCE_DIR}/../unittest_common")



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")

find_package(
  Boost
  COMPONENTS system filesystem
  REQUIRED)

find_library(zlib "z")
find_library(usb "usb-1.0")
find_library(hidapihd "hidapi-hidraw")
  

add_executable(unit_test_live "usb_track.cpp"
"${CMAKE_CURRENT_SOURCE_DIR}/../unittest_common/io_handler.cpp"
  ${usb}
  ${hidapihd}
  )

target_link_libraries(unit_test_live PRIVATE survive 
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  ${zlib}
  ${usb}
  ${hidapihd})

set_target_properties(
  # following properties can be set to path of corresponding android studio
  # folders   https://stackoverflow.com/a/6595001
  unit_test_live
  PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../"
             LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../"
             RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../")

set_property(TARGET unit_test_live PROPERTY VS_DEBUGGER_WORKING_DIRECTORY
                              "${CMAKE_SOURCE_DIR}")