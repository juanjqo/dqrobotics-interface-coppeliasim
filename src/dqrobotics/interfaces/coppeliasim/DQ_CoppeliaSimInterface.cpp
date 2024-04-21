#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>
#include <RemoteAPIClient.h>

std::string DQ_CoppeliaSimInterface::_map_simulation_state(const int &state) const
{
    std::string str;
    switch (state)
    {
        case 0:
            str = "simulation_stopped";
            break;
        case 8:
            str = "simulation_paused";
            break;
        case 17:
            str = "simulation_advancing_running";
            break;
        case 22:
            str = " simulation_advancing_lastbeforestop";
            break;
        case 19:
            str = "simulation_advancing_lastbeforepause";
            break;
        case 16:
            str = "simulation_advancing_firstafterstop or simulation_advancing";
            break;
        case 20:
            str = "simulation_advancing_firstafterpause";
            break;
        case 21:
            str = "simulation_advancing_abouttostop";
            break;
        default:
            str = "Unknown status";
    }
    return str;
}

DQ_CoppeliaSimInterface::DQ_CoppeliaSimInterface()
    :_client_created(false)
{

}


std::unique_ptr<RemoteAPIClient> client_;

/**
 * @brief _create_client
 * @param host
 * @param rpcPort
 * @param cntPort
 * @param verbose_
 * @param client_flag
 */
void _create_client(const std::string& host = "localhost",
                                             const int& rpcPort = 23000,
                                             const int& cntPort = -1,
                                             const int& verbose_ = -1,
                                             const bool& client_flag = false)
{
    if (!client_flag)
         client_ = std::make_unique<RemoteAPIClient>(host, rpcPort, cntPort, verbose_);
}

/**
 * @brief DQ_CoppeliaSimInterface::connect establish a connection between the client (your code) and
 *                                         the host (the CoppeliaSim scene).
 * @param host    eg. 'localhost' if the host is running in the same
 *                machine in which is running the client.
 * @param rpcPort The port to establish a connection. (e.g. 23000, 23001, 23002, 23003...).
 * @param cntPort
 * @param verbose_
 */
void DQ_CoppeliaSimInterface::connect(const std::string &host, const int &rpcPort, const int &cntPort, const int &verbose_)
{

    try
    {
        _create_client(host, rpcPort, cntPort, verbose_, _client_created);
        _client_created = true;
        set_status_bar_message("       ");
        set_status_bar_message("DQ_CoppeliaSimInterface "
                               "is brought to you by Juan Jose Quiroz");
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Runtime error in DQ_CoppeliaSimInterface::connect. "
                  << e.what() << std::endl;
    }
}

/**
 * @brief DQ_CoppeliaSimInterface::start_simulation starts the CoppeliaSim simulation.
 */
void DQ_CoppeliaSimInterface::start_simulation() const
{
    client_->getObject().sim().startSimulation();
}

/**
 * @brief DQ_CoppeliaSimInterface::pause_simulation pauses the CoppeliaSim simulation.
 */
void DQ_CoppeliaSimInterface::pause_simulation() const
{
    client_->getObject().sim().pauseSimulation();

}


/**
 * @brief DQ_CoppeliaSimInterface::stop_simulation stops the  CoppeliaSim simulation.
 */
void DQ_CoppeliaSimInterface::stop_simulation() const
{
    client_->getObject().sim().stopSimulation();
}


/**
 * @brief DQ_CoppeliaSimInterface::set_stepping_mode enables or disables the stepping mode
 *        (formerly known as synchronous mode).
 * @param flag. Eg: set_stepping_mode(true)  // enables the stepping mode
 *                  set_stepping_mode(false)  // disables the stepping mode
 */
void DQ_CoppeliaSimInterface::set_stepping_mode(const bool &flag)
{
    client_->getObject().sim().setStepping(flag);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_simulation_time returns the simulation time.
 *        This time does not correspond to the real-time necessarily.
 * @return The simulation time.
 */
double DQ_CoppeliaSimInterface::get_simulation_time() const
{
    return client_->getObject().sim().getSimulationTime();
}

/**
 * @brief DQ_CoppeliaSimInterface::trigger_next_simulation_step This method sends a trigger
 *        signal to the CoppeliaSim scene, which performs a simulation step when the stepping mode is used.
 */
void DQ_CoppeliaSimInterface::trigger_next_simulation_step() const
{
    client_->getObject().sim().step();
}



/**
 * @brief DQ_CoppeliaSimInterface::is_simulation_running checks if the simulation is running.
 * @return True if the simulation is running. False otherwise.
 */
bool DQ_CoppeliaSimInterface::is_simulation_running() const
{
    return (client_->getObject().sim().getSimulationState() >
            client_->getObject().sim().simulation_paused);
}


/**
 * @brief DQ_CoppeliaSimInterface::get_simulation_state
 *        See more in https://manual.coppeliarobotics.com/en/simulation.htm
 *
 * @return The simulation state.
 *         simulation_advancing = 16
 *         simulation_advancing_abouttostop = 21
 *         simulation_advancing_firstafterpause = 20
 *         simulation_advancing_firstafterstop = 16
 *         simulation_advancing_lastbeforepause = 19
 *         simulation_advancing_lastbeforestop = 22
 *         simulation_advancing_running = 17
 *         simulation_paused = 8
 *         simulation_stopped = 0
 *
 */
int DQ_CoppeliaSimInterface::get_simulation_state() const
{
    return client_->getObject().sim().getSimulationState();
}

/**
 * @brief DQ_CoppeliaSimInterface::set_status_bar_message sends a message to CoppeliaSim to be
 *        displayed in the status bar.
 *
 * @param message
 */
void DQ_CoppeliaSimInterface::set_status_bar_message(const std::string &message) const
{
    client_->getObject().sim().addLog(client_->getObject().sim().verbosity_undecorated, message);
}


//---------------Deprecated methods-----------------------------
void DQ_CoppeliaSimInterface::disconnect(){}
void DQ_CoppeliaSimInterface::disconnect_all(){}
void DQ_CoppeliaSimInterface::set_synchronous(const bool &flag){set_stepping_mode(flag);}
int DQ_CoppeliaSimInterface::wait_for_simulation_step_to_end(){return 0;}

//--------------------------------------------------------------

