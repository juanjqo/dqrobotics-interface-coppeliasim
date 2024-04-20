

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
set(ZMQ_REMOTE_API_PATH ${SUBMODULES}/zmqRemoteApi/)
set(SOURCE_DIR ${ZMQ_REMOTE_API_PATH}/clients/cpp)


list(APPEND CMAKE_MODULE_PATH
    ${SOURCE_DIR}/cmake/modules #${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${COPPELIASIM_INCLUDE_DIR}/cmake)
#find_package(CoppeliaSim 4.1.0.0 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GENERATE OFF CACHE BOOL "Generate wrappers for objects and methods (requires CoppeliaSim to be running).")
set(GENERATE_INCLUDE_OBJECTS "" CACHE STRING "Include only given objects (e.g.: \"sim,simIK\") or leave blank to include all objects in wrapper code generation.")
set(GENERATE_EXCLUDE_OBJECTS "simEigen,simB0,simRemoteApi,simQML,simOMPL,simUI,simIM,simIGL,simURDF,simSkel,simBWF,simCmd,simSubprocess,simURLDrop,simWS,simZMQ" CACHE STRING "Exclude given objects (e.g.: \"simUI,simIM\") from being generated. Note: option GENERATE_INCLUDE_OBJECTS, if different from \"\", has precedence over this.")
set(GENERATE_EXCLUDE_METHODS "sim.test,sim.auxFunc,sim.handleExtCalls,sim.getStringSignal,sim.getInt32Signal,sim.getFloatSignal" CACHE STRING "Exclude given methods (e.g.: \"sim.test,simUI.foo\") from being generated.")

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




add_library(RemoteAPIClient STATIC ${SOURCE_DIR}/RemoteAPIClient.cpp)

target_compile_definitions(RemoteAPIClient PUBLIC -DSIM_REMOTEAPICLIENT_OBJECTS)
target_include_directories(RemoteAPIClient PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/jsoncons/include)


include_directories(${SOURCE_DIR})
set_source_files_properties(RemoteAPIClient.h OBJECT_DEPENDS ${SOURCE_DIR}/RemoteAPIObjects.h)
set_source_files_properties(RemoteAPIClient.cpp OBJECT_DEPENDS ${SOURCE_DIR}/RemoteAPIObjects.cpp)
target_link_libraries(RemoteAPIClient PUBLIC cppzmq)






