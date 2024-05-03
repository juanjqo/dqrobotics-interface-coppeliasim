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


namespace DQ_robotics
{

/**
 * @brief _get_dh_matrix returns a matrix related to the D-H parameters of the
 *                        Universal Robot {}, which is
 *                        defined as
 *
 *                        Matrix<Xd raw_dh_matrix;
 *                        raw_franka_mdh << theta,
 *                                              d,
 *                                              a,
 *                                           alpha,
 *                                            type_of_joints;
 * Source: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
 * @param model  ex: UR3,UR5, UR10
 *
 * @return MatrixXd raw_dh_matrix a matrix related to the D-H parameters
 */
MatrixXd _get_dh_matrix(const URXCoppeliaSimRobot::MODEL& model)
{
    const double pi = M_PI;
    switch (model){
    case URXCoppeliaSimRobot::MODEL::UR5:
    {
        Matrix<double,5,6> raw_dh_matrix(5,6);
        raw_dh_matrix <<  -pi/2, -pi/2, 0, -pi/2, 0, 0,
            0.089159-0.02315, 0, 0, 0.10915, 0.09465, 0.0823,
            0, -0.425, -0.39225, 0, 0, 0,
            pi/2,0,0,pi/2,-pi/2,0,
            0,      0,       0,         0,         0,      0;

        return raw_dh_matrix;
        break;
    }
    break;
    }
}

URXCoppeliaSimRobot::URXCoppeliaSimRobot(const std::string &robot_name,
                                         const std::shared_ptr<DQ_CoppeliaSimInterface> &coppeliasim_interface_sptr,
                                         const MODEL &model)
    :DQ_SerialCoppeliaSimRobot(robot_name, coppeliasim_interface_sptr), model_(model)
{

}


DQ_SerialManipulatorDH URXCoppeliaSimRobot::kinematics()
{
    auto kin = DQ_SerialManipulatorDH(_get_dh_matrix(model_));
    kin.set_reference_frame(_get_interface_sptr()->get_object_pose(base_frame_name_));
    kin.set_base_frame(_get_interface_sptr()->get_object_pose(base_frame_name_));
    return kin;
}



}



