/**
(C) Copyright 2024 DQ Robotics Developers

This file is based on DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Juan Jose Quiroz Omana


##### INSTRUCTIONS #######
1) Open CoppeliaSim. (You do not need to load a specific scene).
2) Run and enjoy!

*/

#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.h>
using namespace DQ_robotics;
using namespace Eigen;

VectorXd compute_control_signal(const MatrixXd J,
                                const VectorXd& q,
                                const double& damping,
                                const double& gain,
                                const VectorXd task_error);

int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
    vi->connect();


  //  vi->close_scene();

    // Load the models only if they are not already on the scene.
  //  vi_exp->load_from_model_browser("/robots/non-mobile/UR5.ttm", "/UR5");
  //  vi_exp->load_from_model_browser("/other/reference frame.ttm", "/Current_pose");
    //vi_exp->load_from_model_browser("/other/reference frame.ttm", "/Desired_pose");
    DQ x = 1 + 0.5*E_*0.3*k_;
   // vi->plot_reference_frame("/Desired_pose", DQ(1), 1.5, {0.02, 0.1});




  //  vi_exp->enable_dynamics(false);
    //vi_exp->set_joint_control_modes(robot.get_joint_names(), DQ_CoppeliaSimZmqInterface::JOINT_CONTROL_MODE::POSITION);
  //  vi_exp->set_joint_modes(robot.get_joint_names(), DQ_CoppeliaSimInterfaceZMQ::JOINT_MODE::KINEMATIC);


   // vi->set_object_pose("/Desired_pose", x);

    vi->start_simulation();

    for (int i=0; i<1; i++)
    {

    }
    vi->stop_simulation();
}
