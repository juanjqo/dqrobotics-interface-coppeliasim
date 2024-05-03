#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.h>

namespace DQ_robotics
{

/**
 * @brief DQ_CoppeliaSimRobot::_get_interface_sptr
 * @return
 */
std::shared_ptr<DQ_CoppeliaSimInterface> DQ_CoppeliaSimRobot::_get_interface_sptr() const
{
    return coppeliasim_interface_sptr_;
}

/**
 * @brief DQ_CoppeliaSimRobot::DQ_CoppeliaSimRobot
 * @param robot_name
 * @param coppeliasim_interface_sptr
 */
DQ_CoppeliaSimRobot::DQ_CoppeliaSimRobot(const std::string& robot_name,
                                         const std::shared_ptr<DQ_CoppeliaSimInterface>& coppeliasim_interface_sptr)
{
    robot_name_ = robot_name;
    if(!coppeliasim_interface_sptr)
        throw std::runtime_error("Null reference to coppeliasim_interface, initialize it first!");
    coppeliasim_interface_sptr_ = coppeliasim_interface_sptr;
}

}
