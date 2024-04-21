#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>
#include <RemoteAPIClient.h>

DQ_CoppeliaSimInterface::DQ_CoppeliaSimInterface()
    :_client_created(false)
{

}


std::unique_ptr<RemoteAPIClient> client_;

void _create_client(const std::string& host = "localhost",
                                             const int& rpcPort = 23000,
                                             const int& cntPort = -1,
                                             const int& verbose_ = -1,
                                             const bool& client_flag = false)
{
    if (!client_flag)
    {
        client_ = std::make_unique<RemoteAPIClient>(host, rpcPort, cntPort, verbose_);
    }

};

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
        //client_ = std::make_unique<RemoteAPIClient>(host, rpcPort, cntPort, verbose_);
        _create_client(host, rpcPort, cntPort, verbose_, _client_created);
        _client_created = true;

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

bool DQ_CoppeliaSimInterface::get_client_flag()
{
    return _client_created;
}

