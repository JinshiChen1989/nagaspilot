cmake_minimum_required(VERSION 3.10)
project(NagasPilot)

set(CMAKE_CXX_STANDARD 17)

# Collect all source files
file(GLOB_RECURSE SOURCES
  ${CMAKE_SOURCE_DIR}/selfdrive/**/*.cc
)

# Dummy target to allow clangd to work
add_executable(dummy_target ${SOURCES})
