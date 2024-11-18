/**
(C) Copyright 2024 DQ Robotics Developers

This file is part of DQ Robotics.

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

DQ Robotics website: dqrobotics.github.io

Contributors:

   1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
        - Responsible for the original implementation. This class is based on
          https://github.com/dqrobotics/cpp-interface-vrep/blob/master/include/dqrobotics/interfaces/vrep/DQ_SerialVrepRobot.h
*/

#pragma once
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>
#include <string>
#include <vector>


using namespace Eigen;

namespace DQ_robotics
{
class DQ_SerialCoppeliaSimRobot
{
protected:
    std::string robot_name_;
    std::vector<std::string> jointnames_;
    std::string base_frame_name_;
    DQ_SerialCoppeliaSimRobot(const std::string& robot_name);

public:
    virtual ~DQ_SerialCoppeliaSimRobot() = default;

    virtual std::vector<std::string> get_joint_names() = 0;

    virtual void set_configuration_space(const VectorXd& q) = 0;
    virtual VectorXd get_configuration_space() = 0;

    virtual void set_target_configuration_space(const VectorXd& q_target)=0;

    virtual VectorXd get_configuration_space_velocities()=0;
    virtual void set_target_configuration_space_velocities(const VectorXd& v_target)=0;

    virtual void set_configuration_space_torques(const VectorXd& t)=0;
    virtual VectorXd get_configuration_space_torques()=0;

};
}

