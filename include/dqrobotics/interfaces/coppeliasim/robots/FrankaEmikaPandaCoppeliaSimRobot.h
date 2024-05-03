#pragma once
#include <vector>
#include <dqrobotics/interfaces/coppeliasim/DQ_SerialCoppeliaSimRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics
{
class FrankaEmikaPandaCoppeliaSimRobot: public DQ_SerialCoppeliaSimRobot
{
public:
    FrankaEmikaPandaCoppeliaSimRobot(const std::string& robot_name,
                                     const std::shared_ptr<DQ_CoppeliaSimInterface>& coppeliasim_interface_sptr);
    DQ_SerialManipulatorMDH kinematics();
};
}


