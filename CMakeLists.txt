# Version 2.8.10 onwards is required for dependency build to work correctly.
cmake_minimum_required(VERSION 2.8.10)
set (CMAKE_CXX_STANDARD 11)
project( ekf_bayesopt )

# Add to module path, so we can find our cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

add_subdirectory( applications )

# make an uninstall target
include(${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in)
add_custom_target(uninstall
    COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

