cmake_minimum_required(VERSION 3.10)
project(babeltrace2_plugins)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

find_package(PkgConfig REQUIRED)
pkg_check_modules(BABELTRACE2 REQUIRED babeltrace2)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BABELTRACE2_CFLAGS} -fPIC")

add_library(trace_analysis SHARED src/plugin.cpp src/callback_durations.cpp src/callback_durations.hpp src/memory_usage.cpp src/memory_usage.hpp)

target_link_libraries(trace_analysis ${BABELTRACE2_LIBRARIES})

target_include_directories(trace_analysis PRIVATE ${BABELTRACE2_INCLUDE_DIRS})

set_target_properties(trace_analysis PROPERTIES OUTPUT_NAME "trace_analysis")

set(CMAKE_INSTALL_PREFIX /usr)
install(TARGETS trace_analysis LIBRARY DESTINATION lib/babeltrace2/plugins/)

include(CPack)

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "ros2_tracing_cpp")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_DESCRIPTION "ROS2 tracing C++ plugin")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "William Kaiser <wkaisertexas@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "babeltrace2")
