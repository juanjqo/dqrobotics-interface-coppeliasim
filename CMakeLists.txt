cmake_minimum_required(VERSION 3.5)

project(dqrobotics-interface-coppeliasim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/dependencies.cmake)

set(SUBMODULES ${CMAKE_CURRENT_SOURCE_DIR}/submodules)
set(ZMQ_REMOTE_API_PATH ${CMAKE_CURRENT_SOURCE_DIR}/coppeliarobotics/zmqRemoteApi/)
set(SOURCE_DIR ${ZMQ_REMOTE_API_PATH}/clients/cpp)

INCLUDE_DIRECTORIES(${PROJECT_NAME} include)
INCLUDE_DIRECTORIES(${PROJECT_NAME} /coppeliarobotics/zmqRemoteApi/clients/cpp)

add_library(${PROJECT_NAME} SHARED
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.cpp
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.cpp
    src/dqrobotics/interfaces/coppeliasim/DQ_SerialCoppeliaSimRobot.cpp
    src/dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimRobot.cpp
    coppeliarobotics/zmqRemoteApi/clients/cpp/RemoteAPIClient.cpp
    )

target_compile_definitions(${PROJECT_NAME} PUBLIC -DSIM_REMOTEAPICLIENT_OBJECTS)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/jsoncons/include)
target_link_libraries(${PROJECT_NAME}
                        Boost::filesystem
                        Boost::format
                        Boost::program_options)


include_directories(${SOURCE_DIR})
set_source_files_properties(RemoteAPIClient.h OBJECT_DEPENDS ${SOURCE_DIR}/RemoteAPIObjects.h)
set_source_files_properties(RemoteAPIClient.cpp OBJECT_DEPENDS ${SOURCE_DIR}/RemoteAPIObjects.cpp)


SET_TARGET_PROPERTIES(${PROJECT_NAME}
    PROPERTIES PUBLIC_HEADER
    "include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h"
    )

INSTALL(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics/interfaces/coppeliasim"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

if(APPLE)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME}
        cppzmq
        -ldqrobotics)
endif()


################################################################
# INSTALL HEADERS IN SUBFOLDERS
################################################################

# Other Headers
INSTALL(FILES
    include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.h
    include/dqrobotics/interfaces/coppeliasim/DQ_SerialCoppeliaSimRobot.h
    DESTINATION "src/dqrobotics/interfaces/coppeliasim")

# Robots
INSTALL(FILES
    include/dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimRobot.h
    DESTINATION "include/dqrobotics/interfaces/coppeliasim/robots")

################################################################
# INSTALL SOURCE FILES (So that the debugger can find them)
################################################################

# base folder
INSTALL(FILES
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.cpp
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.cpp
    src/dqrobotics/interfaces/coppeliasim/DQ_SerialCoppeliaSimRobot.cpp
    DESTINATION "src/dqrobotics/interfaces/coppeliasim")

# robots folder
INSTALL(FILES
    src/dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimRobot.cpp
    DESTINATION "src/dqrobotics/interfaces/coppeliasim/robots")


# Temporal Only for Test
add_executable(example src/main.cpp)
target_link_libraries(example PRIVATE ${PROJECT_NAME})

add_executable(example2 src/test_library.cpp)
target_link_libraries(example2 PRIVATE ${PROJECT_NAME})#dqrobotics dqrobotics-interface-coppeliasim)


add_executable(example3 src/test_new_methods.cpp)
target_link_libraries(example3 PRIVATE
                      dqrobotics
                      ${PROJECT_NAME})#dqrobotics dqrobotics-interface-coppeliasim)
