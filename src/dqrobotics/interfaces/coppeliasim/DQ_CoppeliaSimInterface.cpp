#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>


DQ_CoppeliaSimInterface::DQ_CoppeliaSimInterface() {}

/**
 * @brief DQ_CoppeliaSimInterface::connect
 * @param host
 * @param rpcPort
 * @param cntPort
 * @param verbose_
 */
void DQ_CoppeliaSimInterface::connect(const std::string &host, const int &rpcPort, const int &cntPort, const int &verbose_)
{
    try
    {
        client_ = std::make_unique<RemoteAPIClient>(host, rpcPort, cntPort, verbose_);

    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Runtime error in DQ_CoppeliaSimInterface::connect. " << e.what() << std::endl;

    }
}

/**
 * @brief DQ_CoppeliaSimInterface::start_simulation
 */
void DQ_CoppeliaSimInterface::start_simulation() const
{
    client_->getObject().sim().startSimulation();
}


/**
 * @brief DQ_CoppeliaSimInterface::stop_simulation
 */
void DQ_CoppeliaSimInterface::stop_simulation() const
{
    client_->getObject().sim().stopSimulation();
}

/**
 * @brief DQ_CoppeliaSimInterface::disconnect
 */
void DQ_CoppeliaSimInterface::disconnect()
{

}

/**
 * @brief DQ_CoppeliaSimInterface::set_synchronous
 * @param flag
 */
void DQ_CoppeliaSimInterface::set_synchronous(const bool &flag)
{
    client_->getObject().sim().setStepping(flag);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_stepping_mode
 * @param flag
 */
void DQ_CoppeliaSimInterface::set_stepping_mode(const bool &flag)
{
    client_->getObject().sim().setStepping(flag);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_simulation_time
 * @return
 */
double DQ_CoppeliaSimInterface::get_simulation_time()
{
    return client_->getObject().sim().getSimulationTime();
}

void DQ_CoppeliaSimInterface::trigger_next_simulation_step()
{
    client_->getObject().sim().step();
}

int DQ_CoppeliaSimInterface::wait_for_simulation_step_to_end()
{
    return 0;
}

