cmake_minimum_required(VERSION 3.5)

project(cpp-interface-coppelia LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(cpp-interface-coppelia src/main.cpp)

