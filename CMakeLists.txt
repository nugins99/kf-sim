cmake_minimum_required(VERSION 3.5)
project(cdtags)
enable_testing()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 20)

include_directories(${CMAKE_SOURCE_DIR}/src)

set(Boost_FIND_QUIETLY VERBOSE)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    message(STATUS "Building in Debug mode")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall -Wextra -Wpedantic")
    add_compile_options(-fsanitize=address)
    add_link_options(-fsanitize=address)
else()
    message(STATUS "Building in Release mode")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra -Wpedantic")
    endif()

# Set the package version.
# This applies to both the RPM package and # a "configure_file" command to
# replace values in src/Version.h.in"
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

# Source Dirs
add_subdirectory(src/kf)
add_subdirectory(test)

# Packaging
set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_SOURCE_IGNORE_FILES
  \\.git/
  build/
  ".*~$")
set(CPACK_VERBATIM_VARIABLES YES)
include(CPack)
