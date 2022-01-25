project(SpreeSurviveLibUT VERSION 0.0.1)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../include")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")

add_executable(unit_test_rec "${CMAKE_CURRENT_SOURCE_DIR}/../unittest_common/io_handler.cpp" "main.cpp")

target_link_libraries(unit_test_rec PRIVATE spreeposer)

set_target_properties(
  # following properties can be set to path of corresponding android studio
  # folders   https://stackoverflow.com/a/6595001
  unit_test_rec
  PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../"
             LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../"
             RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../")