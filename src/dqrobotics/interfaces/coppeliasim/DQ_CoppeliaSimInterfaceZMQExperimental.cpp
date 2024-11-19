#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.h>
#include <RemoteAPIClient.h>

DQ_CoppeliaSimInterfaceZMQExperimental::DQ_CoppeliaSimInterfaceZMQExperimental()
    :DQ_CoppeliaSimInterfaceZMQ()
{

}

bool DQ_CoppeliaSimInterfaceZMQExperimental::connect(const std::string &host, const int &port, const int &TIMEOUT_IN_MILISECONDS)
{
    auto rtn = DQ_CoppeliaSimInterfaceZMQ::connect(host, port, TIMEOUT_IN_MILISECONDS);
    if (rtn)
    {

    }
    return rtn;
}

void DQ_CoppeliaSimInterfaceZMQExperimental::close_scene()
{

}


void DQ_CoppeliaSimInterfaceZMQExperimental::plot_reference_frame(const std::string &name,
                                                                  const DQ &pose,
                                                                  const double &scale,
                                                                  const std::vector<double> &thickness_and_length)
{

}
