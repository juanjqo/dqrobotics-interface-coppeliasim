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
         This class is based on the FrankaEmikaPandaVrepRobot class
         (https://github.com/dqrobotics/cpp-interface-vrep/blob/master/include/dqrobotics/interfaces/vrep/robots/FrankaEmikaPandaVrepRobot.h)

*/

#pragma once
#include <vector>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobotZMQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics
{
class FrankaEmikaPandaCoppeliaSimZMQRobot: public DQ_CoppeliaSimRobotZMQ
{
public:
    FrankaEmikaPandaCoppeliaSimZMQRobot(const std::string& robot_name,
                                        const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ>& coppeliasim_interface_sptr
                                     );
    DQ_SerialManipulatorMDH kinematics();
};
}


