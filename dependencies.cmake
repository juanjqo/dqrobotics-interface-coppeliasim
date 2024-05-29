

# Set CMake Policies
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif(POLICY CMP0135)

if(UNIX AND NOT APPLE)
    FIND_PACKAGE(Eigen3 REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
    #find_package(cppzmq REQUIRED)
    #find_package(ZeroMQ REQUIRED)
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)

    include(FetchContent)
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
endif()

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


if(WIN32)
    include(C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
    find_package(cppzmq CONFIG REQUIRED)

    set(DQROBOTICS_PATH "C:/Program Files (x86)/dqrobotics")
    add_library(dqrobotics SHARED IMPORTED)
    set_target_properties(dqrobotics PROPERTIES
        IMPORTED_LOCATION ${DQROBOTICS_PATH}/bin/dqrobotics.dll
        IMPORTED_IMPLIB   ${DQROBOTICS_PATH}/lib/dqrobotics.lib
        INTERFACE_INCLUDE_DIRECTORIES ${DQROBOTICS_PATH}/include)
endif()


#-----This works but is super slow--------------#
#set(BOOST_INCLUDE_LIBRARIES thread format filesystem system program_options)
#set(BOOST_ENABLE_CMAKE ON)
# Download and extract the boost library from GitHub
#message(STATUS "Downloading and extracting boost library sources. This will take some time...")
#Set(FETCHCONTENT_QUIET FALSE)
#include(FetchContent)
#FetchContent_Declare(
#  Boost
#  GIT_REPOSITORY https://github.com/boostorg/boost.git
#  GIT_TAG boost-1.84.0
#  USES_TERMINAL_DOWNLOAD TRUE
#  GIT_PROGRESS TRUE
#  GIT_SHALLOW TRUE
#)
#FetchContent_MakeAvailable(Boost)
#-----------------------------------------------



# Add boost lib sources
set(BOOST_INCLUDE_LIBRARIES thread format filesystem system program_options)
set(BOOST_ENABLE_CMAKE ON)

# Download and extract the boost library from GitHub
message(STATUS "Downloading dependencies. This will take some time...")
include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE) # Needed to print downloading progress
FetchContent_Declare(
    Boost
    URL https://github.com/boostorg/boost/releases/download/boost-1.84.0/boost-1.84.0.tar.gz
    USES_TERMINAL_DOWNLOAD FALSE
    GIT_PROGRESS FALSE
    DOWNLOAD_NO_EXTRACT FALSE
)
FetchContent_MakeAvailable(Boost)


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
message(STATUS "Dependencies ready!")





