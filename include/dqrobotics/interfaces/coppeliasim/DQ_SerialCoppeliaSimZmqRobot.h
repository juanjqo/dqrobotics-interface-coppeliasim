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
         The DQ_SerialCoppeliaSimZmqRobot class is partially based on the DQ_SerialVrepRobot class
         (https://github.com/dqrobotics/cpp-interface-vrep/blob/master/include/dqrobotics/interfaces/vrep/DQ_SerialVrepRobot.h)

*/

#pragma once
#include <vector>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimZmqInterface.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_SerialCoppeliaSimRobot.h>

namespace DQ_robotics
{
class DQ_SerialCoppeliaSimZmqRobot: public DQ_SerialCoppeliaSimRobot
{
private:
    std::shared_ptr<DQ_CoppeliaSimZmqInterface> interface_sptr_;
    std::shared_ptr<DQ_CoppeliaSimZmqInterface::experimental> coppeliasim_interface_sptr_;
    DQ_CoppeliaSimZmqInterface::JOINT_CONTROL_MODE joint_control_mode_;
    bool robot_is_used_as_visualization_tool_;

    std::shared_ptr<DQ_CoppeliaSimZmqInterface::experimental> _get_exp_interface_sptr();

    void _initialize_jointnames_from_coppeliasim();

protected:
    std::shared_ptr<DQ_CoppeliaSimZmqInterface> _get_interface_sptr();

    DQ_SerialCoppeliaSimZmqRobot(const std::string& robot_name,
                                 const std::shared_ptr<DQ_CoppeliaSimZmqInterface>& interface_sptr);
private:
    void _set_operation_modes(const DQ_CoppeliaSimZmqInterface::JOINT_MODE& joint_mode,
                             const DQ_CoppeliaSimZmqInterface::JOINT_CONTROL_MODE& joint_control_mode);
    void _set_robot_as_visualization_tool();
    void _set_robot_as_dynamic_tool(const DQ_CoppeliaSimZmqInterface::JOINT_CONTROL_MODE& joint_control_mode);
    void _set_joint_control_type(const DQ_CoppeliaSimZmqInterface::JOINT_CONTROL_MODE& joint_control_mode);
    void _set_control_inputs(const VectorXd& u);

public:

    std::vector<std::string> get_joint_names() override;

    void set_configuration_space(const VectorXd& q) override;
    VectorXd get_configuration_space() override;

    void set_target_configuration_space(const VectorXd& q_target) override;

    VectorXd get_configuration_space_velocities() override;
    void set_target_configuration_space_velocities(const VectorXd& v_target) override;

    void set_configuration_space_torques(const VectorXd& t) override;
    VectorXd get_configuration_space_torques() override;

    //For backwards compatibility, to be removed in a future version of dqrobotics
    [[deprecated("Use set_configuration_space instead")]]
    void send_q_to_vrep(const VectorXd& q);
    [[deprecated("Use get_configuration_space instead")]]
    VectorXd get_q_from_vrep();

    [[deprecated("Use set_configuration_space instead")]]
    void set_configuration_space_positions(const VectorXd& q);

    [[deprecated("Use get_configuration_space instead")]]
    VectorXd get_configuration_space_positions();

    [[deprecated("Use set_target_configuration_space instead")]]
    void set_target_configuration_space_positions(const VectorXd& q_target);




};

}


