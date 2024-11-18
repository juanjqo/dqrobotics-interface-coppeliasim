

# Set CMake Policies
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif(POLICY CMP0135)

if(UNIX AND NOT APPLE)
    FIND_PACKAGE(Eigen3 REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
    
    find_package(Boost 1.71.0 COMPONENTS filesystem system)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

if(APPLE)
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

    find_package(Boost)
    if(Boost_FOUND)
        include_directories(${Boost_INCLUDE_DIRS})
        message(AUTHOR_WARNING "Local Boost ${Boost_VERSION_MAJOR}.${Boost_VERSION_MINOR}.${Boost_VERSION_COUNT} found!")
    else()
        message(AUTHOR_WARNING "Local Boost not found. I'm going to download it!")

        # Download and extract the boost library from GitHub
        # Add boost lib sources
        set(BOOST_INCLUDE_LIBRARIES thread format filesystem system program_options)
        set(BOOST_ENABLE_CMAKE ON)

        message(STATUS "Downloading dependencies. This will take some time...")
        include(FetchContent)
        Set(FETCHCONTENT_QUIET FALSE) # Needed to print downloading progress
        FetchContent_Declare(
            Boost
            #URL https://github.com/boostorg/boost/releases/download/boost-1.84.0/boost-1.84.0.tar.gz
            URL https://github.com/boostorg/boost/releases/download/boost-1.81.0/boost-1.81.0.tar.gz
            USES_TERMINAL_DOWNLOAD FALSE
            GIT_PROGRESS FALSE
            DOWNLOAD_NO_EXTRACT FALSE
        )
        FetchContent_MakeAvailable(Boost)

        #set(CUSTOM_BOOST_COMPONENTS
        #    Boost::filesystem
        #    Boost::format
        #    Boost::program_options
        #)
    endif()

endif()


INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/submodules/cppzmq)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/submodules/jsoncons/include)



