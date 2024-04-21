

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








