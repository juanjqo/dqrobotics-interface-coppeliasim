cmake_minimum_required(VERSION 3.5)

project(cpp-interface-coppelia LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(COPPELIASIM_INCLUDE_DIR /Applications/coppeliaSim.app/Contents/)
include(${CMAKE_CURRENT_SOURCE_DIR}/dependencies_exp.cmake)








# Required for MacOS
# brew install pkg-config
# brew install boost
# brew install cppzmq

# Required vcpkg packages
#./vcpkg install cppzmq  Not anymore!

add_executable(example src/main.cpp)
target_link_libraries(example PRIVATE ${PROJECT_NAME})
