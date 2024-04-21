
![Static Badge](https://img.shields.io/badge/Platform-Apple_silicon-magenta)![Static Badge](https://img.shields.io/badge/Tested-Apple)![Static Badge](https://img.shields.io/badge/Platform-Ubuntu_x64-orange)![Static Badge](https://img.shields.io/badge/Untested-red)![Static Badge](https://img.shields.io/badge/CoppeliaSim-4.6.0--rev18-orange)

# cpp-interface-coppeliasim

## Requirements

- [DQ Robotics](https://github.com/dqrobotics/cpp)


### MacOS (Apple Silicon)

#### Install some brew packages

```shell
brew install pkg-config boost cppzmq eigen
```

#### Download and install CoppeliaSim for MacOS arm64

<img width="200" alt="Screenshot 2024-04-20 at 14 15 13" src="https://github.com/juanjqo/cpp-interface-coppelia/assets/23158313/24ffcd38-d24e-447c-a7d3-aaaadf8f85a1">



#### Build and Install 

Example for coppeliasim-v4.6.0-rev18. NOTE: replace coppeliasim-v4.6.0-rev18 with the actual CoppeliaSim version you have.

```shell
git clone https://github.com/juanjqo/cpp-interface-coppeliasim --recursive
cd cpp-interface-coppelia/coppeliarobotics/zmqRemoteApi
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




