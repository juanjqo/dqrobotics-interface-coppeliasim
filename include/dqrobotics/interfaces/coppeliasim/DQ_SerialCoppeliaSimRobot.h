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

#pragma once
#include <vector>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.h>

namespace DQ_robotics
{
class DQ_SerialCoppeliaSimRobot: public DQ_CoppeliaSimRobot
{
protected:
    std::vector<std::string> jointnames_;
    std::string base_frame_name_;

    void _initialize_jointnames_from_coppeliasim();

    DQ_SerialCoppeliaSimRobot(const std::string& robot_name,
                              const std::shared_ptr<DQ_CoppeliaSimInterface>& coppeliasim_interface_sptr);
public:
    virtual std::vector<std::string> get_joint_names();

    virtual void set_configuration_space_positions(const VectorXd& q);
    virtual VectorXd get_configuration_space_positions();
    virtual void set_target_configuration_space_positions(const VectorXd& q_target);

    virtual VectorXd get_configuration_space_velocities();
    virtual void set_target_configuration_space_velocities(const VectorXd& v_target);

    virtual void set_configuration_space_torques(const VectorXd& torques);
    virtual VectorXd get_configuration_space_torques();

};

}


