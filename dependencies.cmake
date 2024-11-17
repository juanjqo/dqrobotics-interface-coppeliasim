

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
    
    # libboost-all-dev 

    find_package(Boost 1.74.0 COMPONENTS filesystem format options)
    include_directories(${Boost_INCLUDE_DIRS})
    set(CUSTOM_BOOST_COMPONENTS
        ${Boost_PROGRAM_FILESYTEM_LIBRARY}
        ${Boost_PROGRAM_FORMAT_LIBRARY}
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
    )

    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/submodules/cppzmq)
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/submodules/jsoncons/include)
    #include(FetchContent)
    #set(CPPZMQ_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    #FetchContent_Declare(cppzmq
    #    GIT_REPOSITORY https://github.com/zeromq/cppzmq
    #    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cppzmq
    #)
    #FetchContent_GetProperties(cppzmq)
    #if(NOT cppzmq_POPULATED)
    #    FetchContent_Populate(cppzmq)
    #    add_subdirectory(${cppzmq_SOURCE_DIR} ${cppzmq_BINARY_DIR} EXCLUDE_FROM_ALL)
    #endif()

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

    find_package(Boost)
    if(Boost_FOUND)
        #if (Boost_VERSION_MAJOR LESS_EQUAL 1 AND Boost_VERSION_MINOR LESS_EQUAL 81)
                include_directories(${Boost_INCLUDE_DIRS})
                message(AUTHOR_WARNING "Local Boost ${Boost_VERSION_MAJOR}.${Boost_VERSION_MINOR}.${Boost_VERSION_COUNT} found!")
                set(CUSTOM_BOOST_COMPONENTS
                    ${Boost_PROGRAM_FILESYTEM_LIBRARY}
                    ${Boost_PROGRAM_FORMAT_LIBRARY}
                    ${Boost_PROGRAM_OPTIONS_LIBRARY}
                    )
        #else()
        #    message(AUTHOR_WARNING "Local Boost ${Boost_VERSION_MAJOR}.${Boost_VERSION_MINOR}.${Boost_VERSION_COUNT} is not compatible. I'm going to download a compatible one!")
        #endif()
    else()
        message(AUTHOR_WARNING "Local Boost not found. I'm going to download it!")
        include(boost_dependencies.cmake)
    endif()

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








#include(FetchContent)
#FetchContent_Declare(jsoncons
#    GIT_REPOSITORY https://github.com/danielaparker/jsoncons
#    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/jsoncons
#)
#FetchContent_GetProperties(jsoncons)
#if(NOT jsoncons_POPULATED)
#    FetchContent_Populate(jsoncons)
#    #add_subdirectory(${jsoncons_SOURCE_DIR} ${jsoncons_BINARY_DIR} EXCLUDE_FROM_ALL)
#endif()
#message(STATUS "Dependencies ready!")



#include(FetchContent)
#FetchContent_Declare(
#  googletest
#  # Specify the commit you depend on and update it regularly.
#  #URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
#  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
#)

# For Windows: Prevent overriding the parent project's compiler/linker settings
#set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#FetchContent_MakeAvailable(googletest)

