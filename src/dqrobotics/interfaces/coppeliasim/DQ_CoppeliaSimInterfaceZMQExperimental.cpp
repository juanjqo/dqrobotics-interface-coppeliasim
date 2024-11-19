#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.h>

DQ_CoppeliaSimInterfaceZMQExperimental::DQ_CoppeliaSimInterfaceZMQExperimental()
    :DQ_CoppeliaSimInterfaceZMQ()
{
}


void DQ_CoppeliaSimInterfaceZMQExperimental::plot_reference_frame(const std::string &name,
                                                                  const DQ &pose,
                                                                  const double &scale,
                                                                  const std::vector<double> &thickness_and_length)
{
   experimental_smptr_->plot_reference_frame(name, pose, scale, thickness_and_length);
}
