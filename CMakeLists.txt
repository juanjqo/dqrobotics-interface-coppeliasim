cmake_minimum_required(VERSION 3.5)

project(cpp-interface-coppelia LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(cppzmq)

set(SUBMODULES ${CMAKE_CURRENT_SOURCE_DIR}/submodules)
set(JSONCONS_PATH ${SUBMODULES}/jsoncons)
set(ZMQ_REMOTE_API_PATH ${SUBMODULES}/zmqRemoteApi/clients/cpp)


include_directories(${JSONCONS_PATH}/include)
include_directories(${ZMQ_REMOTE_API_PATH})


add_library(RemoteAPIObjects ${ZMQ_REMOTE_API_PATH}/RemoteAPIObjects.cpp)
add_library(RemoteAPIClient ${ZMQ_REMOTE_API_PATH}/RemoteAPIClient.cpp)
target_link_libraries(RemoteAPIClient RemoteAPIObjects cppzmq)

target_include_directories(RemoteAPIClient PUBLIC ${JSONCONS_PATH}/include)
set_source_files_properties(RemoteAPIClient.h OBJECT_DEPENDS ${ZMQ_REMOTE_API_PATH}/RemoteAPIObjects.h)
set_source_files_properties(RemoteAPIClient.cpp OBJECT_DEPENDS ${ZMQ_REMOTE_API_PATH}/RemoteAPIObjects.cpp)



add_executable(cpp-interface-coppelia src/main.cpp)
target_link_libraries(cpp-interface-coppelia RemoteAPIClient)
