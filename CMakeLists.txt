# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.14)

# Set the project name
project(HAL-mcp230xx)

# Make sure we use the C++17 compiler standard
set(CMAKE_CXX_STANDARD 17)

# Create a static library.
add_library(HAL-mcp230xx MCP23008.cpp MCP23008.hpp MCP23017.cpp MCP23017.hpp)

