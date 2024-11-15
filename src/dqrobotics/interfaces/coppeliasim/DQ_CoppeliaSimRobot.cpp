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
         The DQ_CoppeliaSimRobot class is partially based on the DQ_VrepRobot class
         (https://github.com/dqrobotics/cpp-interface-vrep/blob/master/include/dqrobotics/interfaces/vrep/DQ_VrepRobot.h)

*/

#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.h>

namespace DQ_robotics
{

/**
 * @brief DQ_CoppeliaSimRobot::_get_interface_sptr
 * @return
 */
std::shared_ptr<DQ_CoppeliaSimZmqInterface> DQ_CoppeliaSimRobot::_get_interface_sptr() const
{
    return coppeliasim_interface_sptr_;
}

/**
 * @brief DQ_CoppeliaSimRobot::DQ_CoppeliaSimRobot
 * @param robot_name
 * @param coppeliasim_interface_sptr
 */
DQ_CoppeliaSimRobot::DQ_CoppeliaSimRobot(const std::string& robot_name,
                                         const std::shared_ptr<DQ_CoppeliaSimZmqInterface>& coppeliasim_interface_sptr)
{
    robot_name_ = robot_name;
    if(!coppeliasim_interface_sptr)
        throw std::runtime_error("Null reference to coppeliasim_interface, initialize it first!");
    coppeliasim_interface_sptr_ = coppeliasim_interface_sptr;
}

}
