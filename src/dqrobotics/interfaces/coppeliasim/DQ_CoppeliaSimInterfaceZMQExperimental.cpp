#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.h>

DQ_CoppeliaSimInterfaceZMQExperimental::DQ_CoppeliaSimInterfaceZMQExperimental()
{

}

bool DQ_CoppeliaSimInterfaceZMQExperimental::connect(const std::string &host, const int &port, const int &TIMEOUT_IN_MILISECONDS)
{
    auto rtn = connect(host, port, TIMEOUT_IN_MILISECONDS);
    auto ptr = this->DQ_CoppeliaSimInterfaceZMQ::_get_own_smptr();
    this->DQ_CoppeliaSimInterfaceZMQ::experimental::_set_smptr(ptr);
    return rtn;
}


void DQ_CoppeliaSimInterfaceZMQExperimental::plot_reference_frame(const std::string &name,
                                                                  const DQ &pose,
                                                                  const double &scale,
                                                                  const std::vector<double> &thickness_and_length)
{
    this->DQ_CoppeliaSimInterfaceZMQ::experimental::plot_reference_frame(name, pose, scale, thickness_and_length);
}
