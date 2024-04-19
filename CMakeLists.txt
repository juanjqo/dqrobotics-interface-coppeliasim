cmake_minimum_required(VERSION 3.5)

project(cpp-interface-coppelia LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CPP_INTERFACE_COPPELIA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/)
include(${CPP_INTERFACE_COPPELIA_DIR}/dependencies.cmake)


set(COPPELIASIM_INCLUDE_DIR /Applications/coppeliaSim.app/Contents/)

find_package(cppzmq CONFIG REQUIRED)
find_package(jsoncons CONFIG REQUIRED)

#find_package(Boost COMPONENTS )

include_directories(
    ${CPPZMQ_INCLUDE_DIRS}
    ${JSONCONS_INCLUDE_DIRS}
)

# Required for MacOS
# brew install pkg-config

# Required vcpkg packages
#./vcpkg install cppzmq


#set(SUBMODULES ${CMAKE_CURRENT_SOURCE_DIR}/submodules)
#set(ZMQ_REMOTE_API_PATH ${SUBMODULES}/zmqRemoteApi/clients/cpp)

#include_directories(${ZMQ_REMOTE_API_PATH})


#add_library(RemoteAPIObjects ${ZMQ_REMOTE_API_PATH}/RemoteAPIObjects.cpp)

#set(ZMQ_REMOTE_API_PATH
#    /Applications/coppeliaSim.app/Contents/Resources/programming/zmqRemoteApi/clients/cpp/
#)

#set(RemoteAPIClient_headers
 #   ${ZMQ_REMOTE_API_PATH}/RemoteAPIClient.h
 #   ${ZMQ_REMOTE_API_PATH}/RemoteAPIObjects.h
#    ${ZMQ_REMOTE_API_PATH}/sim-deprecated.h
#    ${ZMQ_REMOTE_API_PATH}/sim-special.h
#)
#set(RemoteAPIClient_src
#    ${ZMQ_REMOTE_API_PATH}/RemoteAPIClient.cpp
#    ${ZMQ_REMOTE_API_PATH}/RemoteAPIObjects.cpp
#    ${ZMQ_REMOTE_API_PATH}/sim-deprecated.cpp
#    ${ZMQ_REMOTE_API_PATH}/sim-special.cpp
#)

#set(Boost_USE_STATIC_LIBS        ON)  # only find static libs
#set(Boost_USE_DEBUG_LIBS         OFF) # ignore debug libs and
#set(Boost_USE_RELEASE_LIBS       ON)  # only find release libs
#set(Boost_USE_MULTITHREADED      ON)
#set(Boost_USE_STATIC_RUNTIME    OFF)
#find_package(Boost 1.84 COMPONENTS ALL)
#if(Boost_FOUND)
#    include_directories(${Boost_INCLUDE_DIRS})
#    add_executable(foo foo.cc)
#    target_link_libraries(foo ${Boost_LIBRARIES})
#endif()

#add_library(RemoteAPIClient ${RemoteAPIClient_headers} ${RemoteAPIClient_src})
#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(RemoteAPIClient cppzmq jsoncons ${Boost_LIBRARIES})

#target_include_directories(RemoteAPIClient PUBLIC ${JSONCONS_PATH}/include)
#set_source_files_properties(RemoteAPIClient.h OBJECT_DEPENDS ${ZMQ_REMOTE_API_PATH}/RemoteAPIObjects.h)
#set_source_files_properties(RemoteAPIClient.cpp OBJECT_DEPENDS ${ZMQ_REMOTE_API_PATH}/RemoteAPIObjects.cpp)



add_executable(cpp-interface-coppelia src/main.cpp)
target_link_libraries(cpp-interface-coppelia PRIVATE RemoteAPIClient)
