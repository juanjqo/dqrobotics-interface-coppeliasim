#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimZmqInterface.h"

//#include <dqrobotics/solvers/DQ_PROXQPSolver.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZmqRobot.h>
#include <dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimZmqRobot.h>
#include <dqrobotics/DQ.h>

using namespace DQ_robotics;
using namespace Eigen;

int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimZmqInterface>();
    vi->connect("192.168.50.16", 23000, 800);


    auto vi_exp = DQ_CoppeliaSimZmqInterface::experimental(vi);

    vi_exp.close_scene();

    vi_exp.plot_reference_frame("/x2", 1+0.5*E_*0.3*k_, 1.5, {0.01, 0.1});
    vi_exp.plot_plane("/plane", k_, 0.5*k_);
    vi_exp.plot_line("/line",   i_, -0.3*j_);
    vi_exp.plot_cylinder("/cyl", i_, 0.1*k_);

    vi_exp.load_from_model_browser("/robots/non-mobile/FrankaEmikaPanda.ttm",
                                   "/Franka",
                                    true,
                                    true);

    vi_exp.load_from_model_browser("/robots/non-mobile/UR5.ttm",
                                   "/UR5");


    auto robot1 = FrankaEmikaPandaCoppeliaSimZmqRobot("/Franka", vi);
    vi_exp.set_joint_control_modes(robot1.get_joint_names(),
                                   DQ_CoppeliaSimZmqInterface::JOINT_CONTROL_MODE::VELOCITY);
    auto robot2 = URXCoppeliaSimZmqRobot("/UR5", vi);
    auto robot2_pose = vi->get_object_pose("/UR5");
    vi->set_object_pose("/UR5", robot2_pose*(1+0.5*E_*0.5*i_));


    vi->start_simulation();
    std::cout<<vi->get_object_pose("/x2")<<std::endl;

    int iterations = 100;
    for(int i=0;i<iterations;i++)
    {
        robot1.set_target_configuration_space_velocities((VectorXd(7)<<0.5,0,0,0,0,0,0).finished());
        auto q = robot2.get_configuration_space_positions();
        std::cout<<q.transpose()<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }



    vi->stop_simulation();

}
