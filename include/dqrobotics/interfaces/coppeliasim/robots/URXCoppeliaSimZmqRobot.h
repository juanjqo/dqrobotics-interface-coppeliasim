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

#pragma once
#include <dqrobotics/interfaces/coppeliasim/DQ_SerialCoppeliaSimZmqRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>

namespace DQ_robotics
{


class URXCoppeliaSimZmqRobot: public DQ_SerialCoppeliaSimZmqRobot
{
public:
    enum class MODEL{
        UR5
    };
protected:
    URXCoppeliaSimZmqRobot::MODEL model_;

public:
    URXCoppeliaSimZmqRobot(const std::string& robot_name,
                        const std::shared_ptr<DQ_CoppeliaSimZmqInterface>& coppeliasim_interface_sptr,
                        const MODEL& model = MODEL::UR5);
    DQ_SerialManipulatorDH kinematics();
};

}



