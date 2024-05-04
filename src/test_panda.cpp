#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>
#include<chrono>
#include<thread>
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimRobot.h>
#include <dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimRobot.h>

using namespace DQ_robotics;
using namespace Eigen;

int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimInterface>();
    vi->connect();
    vi->set_dynamic_engine(DQ_CoppeliaSimInterface::MUJOCO);
    vi->set_gravity(DQ(0));


    vi->load_model_from_model_browser_if_missing("/robots/non-mobile/UR5.ttm", "/UR5", true);
    vi->load_model_from_model_browser_if_missing("/other/reference frame.ttm","x");
    vi->load_model_from_model_browser_if_missing("/other/reference frame.ttm","desired");
    vi->load_model_from_model_browser("/robots/non-mobile/FrankaEmikaPanda.ttm", "/Franka");


    std::cout<<vi->_remove_first_slash_from_string("UR5")<<std::endl;
    /*
    auto robot = URXCoppeliaSimRobot("/UR5", vi, URXCoppeliaSimRobot::MODEL::UR5);
    auto robot_model = robot.kinematics();
    robot.set_robot_as_visualization_tool();

    auto q = robot.get_configuration_space_positions();
    std::cout<<"q: "<<q.transpose()<<std::endl;

    VectorXd u = VectorXd::Zero(6);
    u << M_PI/4, 0, 0, -M_PI/4, 0, 0;

    for (int i=0; i<200; i++)
    {
        DQ x = robot_model.fkm(robot.get_configuration_space_positions());
        vi->set_object_pose("/ReferenceFrame1", x);
        robot.set_control_inputs(u);
    }



    */
    vi->stop_simulation();



}
