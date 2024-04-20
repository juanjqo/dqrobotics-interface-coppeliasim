

if(APPLE) #APPLE
    INCLUDE_DIRECTORIES(
        /usr/local/include/
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3
    )
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
    # The library is installed here when using the regular cmake ., make, sudo make install
    LINK_DIRECTORIES(
        /usr/local/lib/
        /opt/homebrew/lib/
        )

    find_package(cppzmq REQUIRED)
    find_package(ZeroMQ REQUIRED)

endif()




if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        message(FATAL_ERROR "Environment variable COPPELIASIM_ROOT_DIR is not set")
    endif()
endif()

set(SUBMODULES ${CMAKE_CURRENT_SOURCE_DIR}/submodules)
set(ZMQ_REMOTE_API_PATH ${CMAKE_CURRENT_SOURCE_DIR}/coppeliarobotics/zmqRemoteApi/)
set(SOURCE_DIR ${ZMQ_REMOTE_API_PATH}/clients/cpp)




list(APPEND CMAKE_MODULE_PATH
    ${SOURCE_DIR}/cmake/modules #${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${COPPELIASIM_INCLUDE_DIR}/cmake)

include(FetchContent)

FetchContent_Declare(jsoncons
    GIT_REPOSITORY https://github.com/danielaparker/jsoncons
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/jsoncons
)
FetchContent_GetProperties(jsoncons)
if(NOT jsoncons_POPULATED)
    FetchContent_Populate(jsoncons)
    #add_subdirectory(${jsoncons_SOURCE_DIR} ${jsoncons_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()



#    include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h

INCLUDE_DIRECTORIES(${PROJECT_NAME} include)
INCLUDE_DIRECTORIES(${PROJECT_NAME} /coppeliarobotics/zmqRemoteApi/clients/cpp)

add_library(${PROJECT_NAME} SHARED
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.cpp
    coppeliarobotics/zmqRemoteApi/clients/cpp/RemoteAPIClient.cpp)

target_compile_definitions(${PROJECT_NAME} PUBLIC -DSIM_REMOTEAPICLIENT_OBJECTS)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/jsoncons/include)


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


INSTALL(FILES
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.cpp
    DESTINATION "src/dqrobotics/interfaces/coppeliasim")



