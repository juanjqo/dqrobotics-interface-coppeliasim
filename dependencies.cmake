

if(APPLE) #APPLE
    INCLUDE_DIRECTORIES(
        /usr/local/include/
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3
        #/opt/howebrew/include/fmt
        #/opt/howebrew/include/GLFW   #OPENGL 3
        #/opt/howebrew/include/SDL2   #METAL
        #/System/Library/Frameworks
        #/Applications/coppeliaSim.app/Contents/Resources/programming/zmqRemoteApi/clients/cpp/
        #/Applications/coppeliaSim.app/Contents/Frameworks
        #/Applications/coppeliaSim.app/Contents/Resources/programming/include
        #/Applications/coppeliaSim.app/Contents/
    )
ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
# The library is installed here when using the regular cmake ., make, sudo make install
LINK_DIRECTORIES(
    /usr/local/lib/
    /opt/homebrew/lib
    #/Applications/coppeliaSim.app/Contents/MacOS/
    #/Applications/coppeliaSim.app/Contents/
    #/Applications/coppeliaSim.app/Contents/Frameworks
    )

#"-DCMAKE_TOOLCHAIN_FILE=/Users/juanjqo/vcpkg/scripts/buildsystems/vcpkg.cmake"
#set(VCPKG_TARGET_ARCHITECTURE arm64)
#set(VCPKG_OSX_ARCHITECTURES arm64)

set(CMAKE_OSX_ARCHITECTURES arm64)
set(VCPKG_TARGET_TRIPLET arm64-osx) #x64-windows)
set(CMAKE_TOOLCHAIN_FILE /Users/juanjqo/vcpkg/scripts/buildsystems/vcpkg.cmake)
include(/Users/juanjqo/vcpkg/scripts/buildsystems/vcpkg.cmake)


endif()


set(SUBMODULES ${CMAKE_CURRENT_SOURCE_DIR}/submodules)
set(ZMQ_REMOTE_API_PATH ${SUBMODULES}/zmqRemoteApi/)
set(SOURCE_DIR ${ZMQ_REMOTE_API_PATH}/clients/cpp)

#CMAKE_CURRENT_BINARY_DIR="/Users/juanjqo/git/cpp-interface-coppelia/submodules/zmqRemoteApi/clients/build-cpp-Qt_6_6_1_for_macOS-Debug"
#CMAKE_CURRENT_SOURCE_DIR="/Users/juanjqo/git/cpp-interface-coppelia/submodules/zmqRemoteApi/clients/cpp"


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

set(CPPZMQ_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(cppzmq
    GIT_REPOSITORY https://github.com/zeromq/cppzmq
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cppzmq
)
FetchContent_GetProperties(cppzmq)
if(NOT cppzmq_POPULATED)
    FetchContent_Populate(cppzmq)
    add_subdirectory(${cppzmq_SOURCE_DIR} ${cppzmq_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)

add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/calltips.json"
    COMMAND
        ${CMAKE_COMMAND} -E env
        PYTHONPATH="${SOURCE_DIR}/../python/src"
        ${Python3_EXECUTABLE}
        "${SOURCE_DIR}/../../tools/get_raw_calltips.py"
        "${CMAKE_CURRENT_BINARY_DIR}/generated/calltips.json"
    DEPENDS
        "${SOURCE_DIR}/../../tools/get_raw_calltips.py"
)
add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/constants.json"
    COMMAND
        ${CMAKE_COMMAND} -E env
        PYTHONPATH="${SOURCE_DIR}/../python/src"
        ${Python3_EXECUTABLE}
        "${SOURCE_DIR}/../../tools/get_constants.py"
        "${CMAKE_CURRENT_BINARY_DIR}/generated/constants.json"
    DEPENDS
        "${SOURCE_DIR}/../../tools/get_constants.py"
)

set(generatedFiles)
file(GLOB templateFiles RELATIVE ${SOURCE_DIR}/templates/ ${SOURCE_DIR}/templates/*)
foreach(templateFile ${templateFiles})
    add_custom_command(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}"
        COMMAND
            ${Python3_EXECUTABLE}
            "${COPPELIASIM_INCLUDE_DIR}/simStubsGen/external/pycpp/pycpp.py"
            -p "calltips_json=${CMAKE_CURRENT_BINARY_DIR}/generated/calltips.json"
            -p "constants_json=${CMAKE_CURRENT_BINARY_DIR}/generated/constants.json"
            -p "include_objects=${GENERATE_INCLUDE_OBJECTS}"
            -p "exclude_objects=${GENERATE_EXCLUDE_OBJECTS}"
            -p "exclude_methods=${GENERATE_EXCLUDE_METHODS}"
            -i "${SOURCE_DIR}/templates/${templateFile}"
            -o "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}"
            -P "${SOURCE_DIR}/../../tools"
            -P "${SOURCE_DIR}"
        DEPENDS
            "${COPPELIASIM_INCLUDE_DIR}/simStubsGen/external/pycpp/pycpp.py"
            "${SOURCE_DIR}/templates/${templateFile}"
            "${CMAKE_CURRENT_BINARY_DIR}/generated/calltips.json"
            "${CMAKE_CURRENT_BINARY_DIR}/generated/constants.json"
            "${SOURCE_DIR}/cpp_utils.py"
            "${SOURCE_DIR}/../../tools/calltip.py"
            "${SOURCE_DIR}/../../tools/calltip.lark"
    )
    list(APPEND generatedFiles "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}")
endforeach()
add_custom_target(generate_code DEPENDS ${generatedFiles})




add_library(RemoteAPIClient STATIC ${SOURCE_DIR}/RemoteAPIClient.cpp)
if(GENERATE)
    add_dependencies(RemoteAPIClient generate_code)
else()
    foreach(templateFile ${templateFiles})
        if(NOT EXISTS "${SOURCE_DIR}/${templateFile}")
            message(FATAL_ERROR "File ${SOURCE_DIR}/${templateFile} is missing")
        endif()
    endforeach()
endif()
target_compile_definitions(RemoteAPIClient PUBLIC -DSIM_REMOTEAPICLIENT_OBJECTS)
target_include_directories(RemoteAPIClient PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/jsoncons/include)
if(GENERATE)
    message(Generated in = "${CMAKE_CURRENT_BINARY_DIR}/generated/")
    target_include_directories(RemoteAPIClient BEFORE PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/generated)
    set_source_files_properties(RemoteAPIClient.h OBJECT_DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/generated/RemoteAPIObjects.h)
    set_source_files_properties(RemoteAPIClient.cpp OBJECT_DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/generated/RemoteAPIObjects.cpp)
else()
    message(Not generated)
    include_directories(${SOURCE_DIR})
    set_source_files_properties(RemoteAPIClient.h OBJECT_DEPENDS ${SOURCE_DIR}/RemoteAPIObjects.h)
    set_source_files_properties(RemoteAPIClient.cpp OBJECT_DEPENDS ${SOURCE_DIR}/RemoteAPIObjects.cpp)
endif()
target_link_libraries(RemoteAPIClient PUBLIC cppzmq)






