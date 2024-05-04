![Static Badge](https://img.shields.io/badge/status-experimental-critical)![Static Badge](https://img.shields.io/badge/Platform-Apple_silicon-magenta)![Static Badge](https://img.shields.io/badge/Tested-Apple)![Static Badge](https://img.shields.io/badge/Platform-Ubuntu_x64-orange)![Static Badge](https://img.shields.io/badge/Untested-red)![Static Badge](https://img.shields.io/badge/CoppeliaSim-4.6.0--rev18-orange)![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![GitHub License](https://img.shields.io/github/license/juanjqo/cpp-interface-coppeliasim)![Static Badge](https://img.shields.io/badge/based_on-ZeroMQ_remote_API-blue)





# cpp-interface-coppeliasim 

An **unofficial** DQ Robotics interface to connect with CoppeliaSim based on ZeroMQ remote API and C++17. This API provides more functionalities when compared to the legacy remote API (the one used by the [DQ Robotics interface](https://github.com/dqrobotics/cpp-interface-vrep)). However, unlike DQ Robotics, cpp-interface-coppeliasim is experimental and lacks official support.

# Overall advice: Don't use this interface for your project! :warning:

![Static Badge](https://img.shields.io/badge/warning-yellow)
This project is under active development, incomplete, and experimental/unstable. Furthermore, it is compatible with macOS (Apple Silicon) only. Ubuntu versions are expected later, though. Therefore, **I highly recommend** the [official DQ Robotics interface](https://github.com/dqrobotics/cpp-interface-vrep) if you want stability, and outstanding technical support.



| CoppeliaSim  | SO | Status |
| ------------- | ------------- |------------- |
| ![Static Badge](https://img.shields.io/badge/CoppeliaSim-4.6.0--rev18-orange)![Static Badge](https://img.shields.io/badge/arm64-blue)| macOS Sonoma (Apple Silicon) | ![Static Badge](https://img.shields.io/badge/experimental-red)|
| ![Static Badge](https://img.shields.io/badge/CoppeliaSim-4.6.0--rev18-orange)![Static Badge](https://img.shields.io/badge/x64-blue)   | Ubuntu 22.04 LTS  |  ![Static Badge](https://img.shields.io/badge/Unsupported-gray)|
| ![Static Badge](https://img.shields.io/badge/CoppeliaSim-4.6.0--rev18-orange)![Static Badge](https://img.shields.io/badge/x64-blue)   | Windows 11  |  ![Static Badge](https://img.shields.io/badge/Unsupported-gray)


## Requirements

- [DQ Robotics](https://github.com/dqrobotics/cpp)


### MacOS (Apple Silicon)

#### Install some brew packages

```shell
brew install pkg-config cppzmq eigen
```
### Ubuntu 
```shell
sudo apt install libzmq3-dev
```

#### Download and install CoppeliaSim for MacOS arm64

<img width="200" alt="Screenshot 2024-04-20 at 14 15 13" src="https://github.com/juanjqo/cpp-interface-coppelia/assets/23158313/24ffcd38-d24e-447c-a7d3-aaaadf8f85a1">



#### Build and Install 

Example for coppeliasim-v4.6.0-rev18. NOTE: replace coppeliasim-v4.6.0-rev18 with the actual CoppeliaSim version you have.

```shell
git clone https://github.com/juanjqo/cpp-interface-coppeliasim --recursive
cd cpp-interface-coppeliasim/coppeliarobotics/zmqRemoteApi
git checkout coppeliasim-v4.6.0-rev18
cd ../.. && mkdir build && cd build
cmake ..
make -j16
sudo make install
```

#### Uninstall (optional)

```shell
sudo xargs rm < install_manifest.txt
```


#### Example

```shell
#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h"


int main()
{
    try
    {
        DQ_CoppeliaSimInterface vi;
        vi.connect();
        vi.set_stepping_mode(true);
        vi.start_simulation();
        double t = 0.0;

        while (t < 4.0)
        {
            std::cout<<"status: "<<vi.is_simulation_running()<<std::endl;
            t = vi.get_simulation_time();
            std::cout<<"Simulation time: "<<t<<std::endl;
            vi.trigger_next_simulation_step();

        }
        vi.stop_simulation();
        std::cout<<"status: "<<vi.is_simulation_running()<<std::endl;
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Caught a runtime error: " << e.what() << std::endl;
    }

    return 0;
}
```


```cmake
add_executable(example example.cpp)
target_link_libraries(example dqrobotics dqrobotics-interface-coppeliasim)
```




