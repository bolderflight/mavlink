cmake_minimum_required(VERSION 3.13)
# Project information
project(mavlink_c_library_v2
  VERSION 1.0
  DESCRIPTION "MavLink c Library v2 Headers."
  LANGUAGES C
)
# Add the library target
add_library(mavlink_headers_v2 INTERFACE)
# Setup include directories 
target_include_directories(mavlink_headers_v2 INTERFACE .)
