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
       - Responsible for the original implementation.
*/

#include <dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimRobot.h>


namespace dqrobotics{
URXCoppeliaSimRobot::URXCoppeliaSimRobot(const std::string &robot_name,
                                         const std::shared_ptr<DQ_CoppeliaSimInterface> &coppeliasim_interface_sptr,
                                         const MODEL &model)
    :DQ_SerialCoppeliaSimRobot(robot_name, coppeliasim_interface_sptr), model_(model)
{

}

//DQ_SerialManipulatorDH URXCoppeliaSimRobot::kinematics()
//{

//}

MatrixXd get_raw_kinematics(const URXCoppeliaSimRobot::MODEL& model)
{
    const double pi2 = M_PI/2.0;
    Matrix<double,5,7> raw_franka_mdh(5,7);
    raw_franka_mdh <<  0,    0,       0,         0,         0,      0,      0,
        0.333,  0, 3.16e-1,         0,   3.84e-1,      0,      0,
        0,     0,       0,   8.25e-2,  -8.25e-2,      0, 8.8e-2,
        0,  -pi2,     pi2,       pi2,      -pi2,    pi2,    pi2,
        0,     0,       0,         0,         0,      0,      0;

    return raw_franka_mdh;
    switch (model){

    case URXCoppeliaSimRobot::MODEL::UR3:
    case URXCoppeliaSimRobot::MODEL::UR5:
    case URXCoppeliaSimRobot::MODEL::UR10:
        break;
    }
}

}



/*
 *  theta [rad]     a [m]       d [m]       alpha [rad]
Joint 1	0              0       0.089159     π/2
Joint 2	0           -0.425          0       0
Joint 3	0           -0.39225        0       0
Joint 4	0              0       0.10915     π/2
Joint 5	0              0       0.09465     -π/2
Joint 6	0              0       0.0823      0
 * */
