#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>
#include<chrono>
#include<thread>
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimRobot.h>

using namespace DQ_robotics;
using namespace Eigen;

int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimInterface>();
    vi->connect();
    vi->set_dynamic_engine(DQ_CoppeliaSimInterface::MUJOCO);
    vi->set_gravity(DQ(0));

    auto robot = FrankaEmikaPandaCoppeliaSimRobot("/Franka", vi);
    robot.set_robot_as_visualization_tool();
    //robot.set_joint_control_type(DQ_CoppeliaSimInterface::TORQUE);
    vi->start_simulation();

    VectorXd u = VectorXd::Zero(7);
    u << 0.1, 0, 0, 0, 0, 0, 0;

    for (int i=0; i<200; i++)
    {
        robot.set_control_inputs(u);
    }

    vi->stop_simulation();



}
