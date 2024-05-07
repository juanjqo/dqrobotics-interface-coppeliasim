![Static Badge](https://img.shields.io/badge/status-experimental-critical)![Static Badge](https://img.shields.io/badge/Platform-Apple_silicon-magenta)![Static Badge](https://img.shields.io/badge/Tested-Apple)![Static Badge](https://img.shields.io/badge/Platform-Ubuntu_x64-orange)![Static Badge](https://img.shields.io/badge/tested-green)![Static Badge](https://img.shields.io/badge/CoppeliaSim-4.6.0--rev18-orange)![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![GitHub License](https://img.shields.io/github/license/juanjqo/cpp-interface-coppeliasim)![Static Badge](https://img.shields.io/badge/based_on-ZeroMQ_remote_API-blue)





# cpp-interface-coppeliasim 

An **unofficial** DQ Robotics interface to connect with CoppeliaSim based on ZeroMQ remote API and C++17. This API provides more functionalities when compared to the legacy remote API (the one used by the [DQ Robotics interface](https://github.com/dqrobotics/cpp-interface-vrep)). However, unlike DQ Robotics, cpp-interface-coppeliasim is experimental and lacks official support.

# Overall advice: Don't use this interface for your project! :warning:

![Static Badge](https://img.shields.io/badge/warning-yellow)
This project is under active development, incomplete, and experimental/unstable. Therefore, **I highly recommend** the [official DQ Robotics interface](https://github.com/dqrobotics/cpp-interface-vrep) if you want stability, and outstanding technical support.



| CoppeliaSim  | SO | Status |
| ------------- | ------------- |------------- |
| ![Static Badge](https://img.shields.io/badge/CoppeliaSim-4.6.0--rev18-orange)![Static Badge](https://img.shields.io/badge/arm64-blue)| macOS Sonoma (Apple Silicon) | ![Static Badge](https://img.shields.io/badge/experimental-red)|
| ![Static Badge](https://img.shields.io/badge/CoppeliaSim-4.6.0--rev18-orange)![Static Badge](https://img.shields.io/badge/x64-blue)   | Ubuntu 22.04 LTS  |  ![Static Badge](https://img.shields.io/badge/experimental-red)|
| ![Static Badge](https://img.shields.io/badge/CoppeliaSim-4.6.0--rev18-orange)![Static Badge](https://img.shields.io/badge/x64-blue)   | Windows 11  |  ![Static Badge](https://img.shields.io/badge/Unsupported-gray)


## Requirements

- [DQ Robotics](https://github.com/dqrobotics/cpp)


### MacOS (Apple Silicon)

```shell
brew install pkg-config cppzmq
```
### Ubuntu 
```shell
sudo apt install libzmq3-dev
```

#### Download and install CoppeliaSim (for Apple Silicon Macs use CoppeliaSim arm64)

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

Additional step for Ubuntu users:
```shell
sudo ldconfig
```

#### To Uninstall 

```shell
sudo xargs rm < install_manifest.txt
```


#### Example (Find more examples [here](https://github.com/juanjqo/cpp-interface-coppeliasim-examples/tree/main))

1) Open CoppeliaSim. (You do not need to load a specific scene).
2) Run and enjoy!

![ezgif com-video-to-gif-converter (1)](https://github.com/juanjqo/cpp-interface-coppeliasim/assets/23158313/c916025a-de3d-4058-8edf-14976d23584a)

```cpp
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>
#include <dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimRobot.h>

using namespace DQ_robotics;
using namespace Eigen;

VectorXd compute_control_signal(const MatrixXd J,
                                const VectorXd& q,
                                const double& damping,
                                const double& gain,
                                const VectorXd task_error);

int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimInterface>();
    vi->connect();

    // Load the models only if they are not already on the scene.
    vi->load_from_model_browser("/robots/non-mobile/UR5.ttm", "/UR5");
    vi->load_from_model_browser("/other/reference frame.ttm", "/Current_pose");
    vi->load_from_model_browser("/other/reference frame.ttm", "/Desired_pose");
    vi->start_simulation();

    auto robot = URXCoppeliaSimRobot("/UR5", vi, URXCoppeliaSimRobot::MODEL::UR5);
    auto robot_model = robot.kinematics();
    robot.set_robot_as_visualization_tool();

    auto q = robot.get_configuration_space_positions();
    double gain = 10;
    double T = 0.001;
    double damping = 0.01;

    auto xd = robot_model.fkm(((VectorXd(6) <<  0.5, 0, 1.5, 0, 0, 0).finished()));
    vi->set_object_pose("/Desired_pose", xd);

    for (int i=0; i<300; i++)
    {
        auto x = robot_model.fkm(q);
        vi->set_object_pose("/Current_pose", x);
        auto J =  robot_model.pose_jacobian(q);
        auto Jt = robot_model.translation_jacobian(J, x);
        auto task_error = (x.translation()-xd.translation()).vec4();
        auto u = compute_control_signal(Jt, q, damping, gain, task_error);
        q = q + T*u;
        robot.set_control_inputs(q);
        std::cout<<"error: "<<task_error.norm()<<std::endl;
    }
    vi->stop_simulation();
}

VectorXd compute_control_signal(const MatrixXd J,
                                const VectorXd& q,
                                const double& damping,
                                const double& gain,
                                const VectorXd task_error)
{
    VectorXd u = (J.transpose()*J + damping*damping*MatrixXd::Identity(q.size(), q.size())).inverse()*
        J.transpose()*(-gain*task_error);
    return u;
}
```


```cmake
add_executable(${CMAKE_PROJECT_NAME} main.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME}
                      dqrobotics
                      dqrobotics-interface-coppeliasim)
```






