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

set(CUSTOM_BOOST_COMPONENTS
    Boost::filesystem
    Boost::format
    Boost::program_options
)



