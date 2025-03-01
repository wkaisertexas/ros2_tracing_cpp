cmake_minimum_required(VERSION 3.10)
project(ros2_tracing_cpp VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build (Debug, Release, RelWithDebInfo, MinSizeRel)"
    FORCE)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(BABELTRACE2 REQUIRED babeltrace2)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BABELTRACE2_CFLAGS}")

add_library(ros2_tracing_cpp SHARED src/plugin.cpp src/callback_duration.cpp src/memory_usage.cpp)

target_link_libraries(ros2_tracing_cpp ${BABELTRACE2_LIBRARIES})

target_include_directories(ros2_tracing_cpp PRIVATE ${BABELTRACE2_INCLUDE_DIRS})

set_target_properties(ros2_tracing_cpp PROPERTIES OUTPUT_NAME "ros2_tracing_cpp")

set(CMAKE_INSTALL_PREFIX /usr)
install(TARGETS ros2_tracing_cpp LIBRARY DESTINATION local/lib/babeltrace2/plugins)

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "ros2_tracing_cpp")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_DEBIAN_COMPRESSION_TYPE "gzip")
set(CPACK_PACKAGE_FILE_NAME "ros2-tracing-cpp")
set(CPACK_PACKAGE_DESCRIPTION "ROS2 tracing C++ plugin")
set(CPACK_PACKAGE_CONTACT "William Kaiser <wkaisertexas@gmail.com>")
set(CPACK_RESOURCE_FILE_LICENSE "../LICENSE")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "babeltrace2")
set(CPACK_DEBIAN_PACKAGE_SECTION "utils")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")

include(CPack)
