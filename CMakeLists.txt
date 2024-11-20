cmake_minimum_required(VERSION 3.5)

project(dqrobotics-interface-coppeliasim-zmq LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(${CMAKE_CURRENT_SOURCE_DIR}/dependencies.cmake)

set(SUBMODULES ${CMAKE_CURRENT_SOURCE_DIR}/submodules)
set(ZMQ_REMOTE_API_PATH ${SUBMODULES}/zmqRemoteApi/)
set(SOURCE_DIR ${ZMQ_REMOTE_API_PATH}/clients/cpp)

INCLUDE_DIRECTORIES(${PROJECT_NAME} include)
INCLUDE_DIRECTORIES(${PROJECT_NAME} ${SUBMODULES}/zmqRemoteApi/clients/cpp)



add_library(${PROJECT_NAME} SHARED
    src/dqrobotics/interfaces/coppeliasim/internal/_zmq_wrapper.cpp
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.cpp
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.cpp
#    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.cpp
#    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobotZMQ.cpp
#    src/dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.cpp
#    src/dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimZMQRobot.cpp
    submodules/zmqRemoteApi/clients/cpp/RemoteAPIClient.cpp

  )

target_compile_definitions(${PROJECT_NAME} PUBLIC -DSIM_REMOTEAPICLIENT_OBJECTS)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/jsoncons/include)
target_link_libraries(${PROJECT_NAME}
                        ${Boost_LIBRARIES}
                        zmq
                        dqrobotics
                        )


include_directories(${SOURCE_DIR})
set_source_files_properties(RemoteAPIClient.h OBJECT_DEPENDS ${SOURCE_DIR}/RemoteAPIObjects.h)
set_source_files_properties(RemoteAPIClient.cpp OBJECT_DEPENDS ${SOURCE_DIR}/RemoteAPIObjects.cpp)


SET_TARGET_PROPERTIES(${PROJECT_NAME}
    PROPERTIES PUBLIC_HEADER
    "include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h"
    )

INSTALL(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics/interfaces/coppeliasim"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

if(WIN32)
INSTALL(TARGETS ${PROJECT_NAME}
# https://stackoverflow.com/questions/21592361/cmake-install-is-not-installing-libraries-on-windows
    RUNTIME DESTINATION "bin"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
endif()


################################################################
# INSTALL HEADERS IN SUBFOLDERS
################################################################

# Other Headers
INSTALL(FILES
    include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobotZMQ.h
    include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.h
#    include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.h
    DESTINATION "include/dqrobotics/interfaces/coppeliasim")

# Robots
INSTALL(FILES
#    include/dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.h
#    include/dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimZMQRobot.h
    DESTINATION "include/dqrobotics/interfaces/coppeliasim/robots")

################################################################
# INSTALL SOURCE FILES (So that the debugger can find them)
################################################################

# base folder
INSTALL(FILES
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.cpp
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.cpp
#    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobotZMQ.cpp
#    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.cpp
    DESTINATION "src/dqrobotics/interfaces/coppeliasim")

# robots folder
INSTALL(FILES
#    src/dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.cpp
#    src/dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimZMQRobot.cpp
    DESTINATION "src/dqrobotics/interfaces/coppeliasim/robots")


add_executable(general_tests src/main.cpp)
target_link_libraries(general_tests PRIVATE
                    dqrobotics
                    ${PROJECT_NAME})
