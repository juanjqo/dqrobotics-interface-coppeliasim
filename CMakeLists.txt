cmake_minimum_required(VERSION 3.5)

project(cpp-interface-coppelia LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/dependencies.cmake)


set(COPPELIASIM_INCLUDE_DIR /Applications/coppeliaSim.app/Contents/)
set(Python3_EXECUTABLE /usr/local/bin/python3)


# Required for MacOS
# brew install pkg-config
# brew install boost

# Required vcpkg packages
#./vcpkg install cppzmq

add_executable(cpp-interface-coppelia src/main.cpp)
target_link_libraries(cpp-interface-coppelia PRIVATE RemoteAPIClient)
