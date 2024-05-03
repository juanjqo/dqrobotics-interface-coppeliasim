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
         The DQ_SerialCoppeliaSimRobot class is partially based on the DQ_SerialVrepRobot class
         (https://github.com/dqrobotics/cpp-interface-vrep/blob/master/include/dqrobotics/interfaces/vrep/DQ_SerialVrepRobot.h)

*/

#include <dqrobotics/interfaces/coppeliasim/DQ_SerialCoppeliaSimRobot.h>


namespace DQ_robotics
{

/**
 * @brief DQ_SerialCoppeliaSimRobot::_initialize_jointnames_from_coppeliasim
 */
void DQ_SerialCoppeliaSimRobot::_initialize_jointnames_from_coppeliasim()
{
    jointnames_ = _get_interface_sptr()->get_jointnames_from_base_objectname(robot_name_);
    base_frame_name_ = jointnames_.at(0);
}


/**
 * @brief DQ_SerialCoppeliaSimRobot::DQ_SerialCoppeliaSimRobot
 * @param robot_name
 * @param coppeliasim_interface_sptr
 */
DQ_SerialCoppeliaSimRobot::DQ_SerialCoppeliaSimRobot(const std::string &robot_name,
                                                     const std::shared_ptr<DQ_CoppeliaSimInterface> &coppeliasim_interface_sptr)
    :DQ_CoppeliaSimRobot(robot_name, coppeliasim_interface_sptr)
{
    _initialize_jointnames_from_coppeliasim();
    // By Default, the robot is controlled by joint positions with both the dynamic engine
    // and the stepping mode enabled.
    joint_control_mode_ = DQ_CoppeliaSimInterface::JOINT_CONTROL_MODE::POSITION;
    robot_is_used_as_visualization_tool_ = false;
}

/**
 * @brief DQ_SerialCoppeliaSimRobot::set_operation_modes
 * @param joint_mode
 * @param joint_control_mode
 */
void DQ_SerialCoppeliaSimRobot::set_operation_modes(const DQ_CoppeliaSimInterface::JOINT_MODE &joint_mode, const DQ_CoppeliaSimInterface::JOINT_CONTROL_MODE &joint_control_mode)
{
    _get_interface_sptr()->set_joint_modes(jointnames_, joint_mode);
    _get_interface_sptr()->set_joint_control_modes(jointnames_, joint_control_mode);
}

/**
 * @brief DQ_SerialCoppeliaSimRobot::set_robot_as_visualization_tool enables the joint
 *        kinematic mode in the robot. Furthermore, both the stepping mode and the
 *        dynamic engine are disabled. Use this mode if you want to control the robot
 *        by joint position commands without taking into account the dynamics.
 *        In other words, the CoppeliaSim scene is used as a visualization tool.
 */
void DQ_SerialCoppeliaSimRobot::set_robot_as_visualization_tool()
{
    _get_interface_sptr()->set_stepping_mode(false);
    _get_interface_sptr()->enable_dynamics(false);
    _get_interface_sptr()->set_joint_modes(jointnames_,
                                           DQ_CoppeliaSimInterface::JOINT_MODE::KINEMATIC);
}

/**
 * @brief DQ_SerialCoppeliaSimRobot::set_joint_control_type sets the joint control mode
 *        of the robot. Furthermore, both the dynamic engine and the stepping mode are enabled.
 * @param joint_control_mode Use POSITION, VELOCITY or TORQUE.
 */
void DQ_SerialCoppeliaSimRobot::set_joint_control_type(const DQ_CoppeliaSimInterface::JOINT_CONTROL_MODE &joint_control_mode)
{
    joint_control_mode_ = joint_control_mode;
    _get_interface_sptr()->enable_dynamics(true);
    _get_interface_sptr()->set_stepping_mode(true);
    set_operation_modes(DQ_CoppeliaSimInterface::JOINT_MODE::DYNAMIC,
                        joint_control_mode_);
}

/**
 * @brief DQ_SerialCoppeliaSimRobot::set_control_inputs sends the commands to CoppeliaSim.
 *
 * @param u joint positions, velocities or torques.
 */
void DQ_SerialCoppeliaSimRobot::set_control_inputs(const VectorXd &u)
{
    if (robot_is_used_as_visualization_tool_)
        _get_interface_sptr()->set_joint_positions(jointnames_, u);
    else
    {
        switch (joint_control_mode_)
        {
        case DQ_CoppeliaSimInterface::FREE:
            break;
        case DQ_CoppeliaSimInterface::FORCE:
            break;
        case DQ_CoppeliaSimInterface::VELOCITY:
            _get_interface_sptr()->set_joint_target_velocities(jointnames_, u);
            break;
        case DQ_CoppeliaSimInterface::POSITION:
            _get_interface_sptr()->set_joint_target_positions(jointnames_, u);
            break;
        case DQ_CoppeliaSimInterface::SPRING:
            break;
        case DQ_CoppeliaSimInterface::CUSTOM:
            break;
        case DQ_CoppeliaSimInterface::TORQUE:
            _get_interface_sptr()->set_joint_torques(jointnames_, u);
            break;
        }
        _get_interface_sptr()->trigger_next_simulation_step();
    }
}


/**
 * @brief DQ_SerialCoppeliaSimRobot::get_joint_names
 * @return
 */
std::vector<std::string> DQ_SerialCoppeliaSimRobot::get_joint_names()
{
    return jointnames_;
}


/**
 * @brief DQ_SerialCoppeliaSimRobot::set_configuration_space_positions
 * @param q
 */
void DQ_SerialCoppeliaSimRobot::set_configuration_space_positions(const VectorXd &q)
{
    _get_interface_sptr()->set_joint_positions(jointnames_,q);
}


/**
 * @brief DQ_SerialCoppeliaSimRobot::get_configuration_space_positions
 * @return
 */
VectorXd DQ_SerialCoppeliaSimRobot::get_configuration_space_positions()
{
    return  _get_interface_sptr()->get_joint_positions(jointnames_);
}


/**
 * @brief DQ_SerialCoppeliaSimRobot::set_target_configuration_space_positions
 * @param q_target
 */
void DQ_SerialCoppeliaSimRobot::set_target_configuration_space_positions(const VectorXd &q_target)
{
    return _get_interface_sptr()->set_joint_target_positions(jointnames_, q_target);
}


/**
 * @brief DQ_SerialCoppeliaSimRobot::get_configuration_space_velocities
 * @return
 */
VectorXd DQ_SerialCoppeliaSimRobot::get_configuration_space_velocities()
{
    return _get_interface_sptr()->get_joint_velocities(jointnames_);
}

/**
 * @brief DQ_SerialCoppeliaSimRobot::set_target_configuration_space_velocities
 * @param v_target
 */
void DQ_SerialCoppeliaSimRobot::set_target_configuration_space_velocities(const VectorXd &v_target)
{
    _get_interface_sptr()->set_joint_target_velocities(jointnames_, v_target);
}

/**
 * @brief DQ_SerialCoppeliaSimRobot::set_configuration_space_torques
 * @param torques
 */
void DQ_SerialCoppeliaSimRobot::set_configuration_space_torques(const VectorXd &torques)
{
    _get_interface_sptr()->set_joint_torques(jointnames_, torques);
}

/**
 * @brief DQ_SerialCoppeliaSimRobot::get_configuration_space_torques
 * @return
 */
VectorXd DQ_SerialCoppeliaSimRobot::get_configuration_space_torques()
{
    return _get_interface_sptr()->get_joint_torques(jointnames_);
}

}
