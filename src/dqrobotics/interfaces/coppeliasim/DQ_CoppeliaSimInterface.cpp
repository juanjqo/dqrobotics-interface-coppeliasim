#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>


DQ_CoppeliaSimInterface::DQ_CoppeliaSimInterface() {}

bool DQ_CoppeliaSimInterface::connect(const std::string &host, const int &rpcPort, const int &cntPort, const int &verbose_)
{
    try
    {
        client_ = std::make_unique<RemoteAPIClient>(host, rpcPort, cntPort, verbose_);
        return true;
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Runtime error in DQ_CoppeliaSimInterface::connect. " << e.what() << std::endl;
        return false;
    }
}

