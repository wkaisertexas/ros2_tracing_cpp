cmake_minimum_required(VERSION 3.10)
project(babeltrace2_plugins)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

# Find Babeltrace2 using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(BABELTRACE2 REQUIRED babeltrace2)

# Set output directory for the compiled plugin (optional)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)

# Compiler flags for position-independent code (PIC) and Babeltrace2 includes
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BABELTRACE2_CFLAGS} -fPIC")

# Add the shared library target (plugin)
add_library(callback_duration SHARED src/callback_duration.cpp)
add_library(memory_usage SHARED src/memory_usage.cpp)

# Link Babeltrace2 libraries
target_link_libraries(callback_duration ${BABELTRACE2_LIBRARIES})
target_link_libraries(memory_usage ${BABELTRACE2_LIBRARIES})

# Set include directories for Babeltrace2
target_include_directories(callback_duration PRIVATE ${BABELTRACE2_INCLUDE_DIRS})
target_include_directories(memory_usage PRIVATE ${BABELTRACE2_INCLUDE_DIRS})

# Rename output to match .so naming convention
set_target_properties(callback_duration PROPERTIES OUTPUT_NAME "callback_duration")
set_target_properties(memory_usage PROPERTIES OUTPUT_NAME "memory_usage")