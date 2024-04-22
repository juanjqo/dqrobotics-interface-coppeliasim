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
    :client_created_(false)
{

}

//-------------------Private components-----------------------//
std::unique_ptr<RemoteAPIClient> client_;


void _set_status_bar_message(const std::string &message, const int& verbosity_type)
{
    client_->getObject().sim().addLog(verbosity_type, message);
}



//-------------------------------------------------------------//

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
        _create_client(host, rpcPort, cntPort, verbose_, client_created_);
        client_created_ = true;
        set_status_bar_message("       ");
        _set_status_bar_message("DQ_CoppeliaSimInterface "
                                "is brought to you by Juan Jose Quiroz Omana",
                                client_->getObject().sim().verbosity_warnings);
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
    _set_status_bar_message(message, client_->getObject().sim().verbosity_undecorated);
}


/**
 * @brief DQ_CoppeliaSimInterface::get_object_handle gets the object handle from
 *        CoppeliaSim. If the handle is not included in the map, then the map is
 *        updated.
 * @param objectname
 * @return the object handle.
 */
int DQ_CoppeliaSimInterface::get_object_handle(const std::string &objectname)
{
    int handle;
    try
    {
        handle = client_->getObject().sim().getObject(objectname);
    }
    catch(const std::runtime_error& e)
    {
        throw std::runtime_error(
            std::string(e.what())
            + " "
            + std::string("The object ")
            + objectname
            + std::string(" does not exist in the current scene in CoppeliaSim.")
            );
    }
    _update_map(objectname, handle);
    return handle;
}

/**
 * @brief DQ_CoppeliaSimInterface::get_object_handles
 * @param objectnames
 * @return
 */
std::vector<int> DQ_CoppeliaSimInterface::get_object_handles(const std::vector<std::string> &objectnames)
{
    int n = objectnames.size();
    std::vector<int> handles(n);
    for(int i=0;i<n;i++)
    {
        handles[i]=get_object_handle(objectnames[i]);
    }
    return handles;
}

std::map<std::string, int> DQ_CoppeliaSimInterface::get_map()
{
    return set_states_map_;
}

void DQ_CoppeliaSimInterface::show_map()
{
    for (const auto& p : set_states_map_)
    {
        std::cout << '[' << p.first << "] = " << p.second << '\n';
    }
}

/**
 * @brief DQ_CoppeliaSimInterface::get_object_translation returns the position
 *        of a handle in the CoppeliaSim scene with respect to the absolute frame.
 * @param handle The handle of the object.
 * @return the absolute position of the handle.
 */
DQ DQ_CoppeliaSimInterface::get_object_translation(const int &handle)
{
    auto position = client_->getObject().sim().getObjectPosition(handle,
                                                 client_->getObject().sim().handle_world);
    const DQ t = DQ(0, position.at(0),position.at(1),position.at(2));
    return t;
}


/**
 * @brief DQ_CoppeliaSimInterface::get_object_translation returns the position
 *        of a handle in the CoppeliaSim scene with respect to 'relative_to_handle'
 * @param handle
 * @param relative_to_handle
 * @return the relative position of the handle.
 */
DQ DQ_CoppeliaSimInterface::get_object_translation(const int &handle, const int &relative_to_handle)
{
    DQ handle_position1 = get_object_translation(handle);
    DQ handle_position2 = get_object_translation(relative_to_handle);
    DQ x = (1 + 0.5*E_*handle_position2).conj()*(1 + 0.5*E_*handle_position1);
    return x.translation();
}


/**
 * @brief DQ_CoppeliaSimInterface::get_object_translation returns the position
 *        of an object in the CoppeliaSim scene with respect to the absolute frame.
 * @param objectname The name of the object.
 * @return the absolute position of the object.
 */
DQ DQ_CoppeliaSimInterface::get_object_translation(const std::string &objectname)
{
    return get_object_translation(_get_object_handle(objectname));
}


/**
 * @brief DQ_CoppeliaSimInterface::get_object_translation returns the position of
 *        an object in the CoppeliaSim scene with respect to relative_to_objectname.
 * @param objectname
 * @param relative_to_objectname
 * @return the relative position of the objectname.
 */
DQ DQ_CoppeliaSimInterface::get_object_translation(const std::string &objectname,
                                                   const std::string &relative_to_objectname)
{
    return get_object_translation(_get_object_handle(objectname), _get_object_handle(relative_to_objectname));
}


/**
 * @brief DQ_CoppeliaSimInterface::set_object_translation sets the translation of a handle
 *        in the CoppeliaSim scene.
 * @param handle
 * @param t desired position.
 */
void DQ_CoppeliaSimInterface::set_object_translation(const int &handle, const DQ &t)
{
    VectorXd vec_t = t.vec3();
    std::vector<double> position = {vec_t[0], vec_t[1],vec_t[2]};
    client_->getObject().sim().setObjectPosition(handle, position,
                                                 client_->getObject().sim().handle_world);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_object_translation
 * @param objectname
 * @param t
 */
void DQ_CoppeliaSimInterface::set_object_translation(const std::string &objectname, const DQ &t)
{
    set_object_translation(_get_object_handle(objectname), t);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_object_rotation
 * @param handle
 * @return the object rotation
 */
DQ DQ_CoppeliaSimInterface::get_object_rotation(const int &handle)
{
    auto rotation = client_->getObject().sim().getObjectQuaternion(handle +
                                                   client_->getObject().sim().handleflag_wxyzquat,
                                                   client_->getObject().sim().handle_world);

    return DQ(rotation.at(0), rotation.at(1), rotation.at(2), rotation.at(3));
}

/**
 * @brief DQ_CoppeliaSimInterface::get_object_rotation
 * @param objectname
 * @return
 */
DQ DQ_CoppeliaSimInterface::get_object_rotation(const std::string &objectname)
{
    return get_object_rotation(_get_object_handle(objectname));
}

/**
 * @brief DQ_CoppeliaSimInterface::set_object_rotation
 * @param handle
 * @param r
 */
void DQ_CoppeliaSimInterface::set_object_rotation(const int &handle, const DQ &r)
{
    VectorXd vec_r = r.vec4();
    std::vector<double> rotation= {vec_r[0], vec_r[1],vec_r[2], vec_r[3]};
    client_->getObject().sim().setObjectQuaternion(handle +
                                                   client_->getObject().sim().handleflag_wxyzquat,
                                                   rotation,
                                                   client_->getObject().sim().handle_world);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_object_rotation
 * @param objectname
 * @param r
 */
void DQ_CoppeliaSimInterface::set_object_rotation(const std::string &objectname, const DQ &r)
{
    set_object_rotation(_get_object_handle(objectname), r);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_object_pose
 * @param handle
 * @return
 */
DQ DQ_CoppeliaSimInterface::get_object_pose(const int &handle)
{
    DQ t = get_object_translation(handle);
    DQ r = get_object_rotation(handle);
    return r + 0.5*E_*t*r;
}

/**
 * @brief DQ_CoppeliaSimInterface::get_object_pose
 * @param objectname
 * @return
 */
DQ DQ_CoppeliaSimInterface::get_object_pose(const std::string &objectname)
{
    return get_object_pose(_get_object_handle(objectname));
}


/**
 * @brief DQ_CoppeliaSimInterface::_update_map updates the map if and only if
 *        the objectname is not in the map.
 * @param objectname
 * @param handle
 */
void DQ_CoppeliaSimInterface::_update_map(const std::string &objectname,
                                                    const int &handle)
{
    set_states_map_.try_emplace(objectname, handle);
}

/**
 * @brief DQ_CoppeliaSimInterface::_get_object_handle gets the object handle from the map.
 *        If the object is not included in the map, the method gets the handle from
 *        CoppeliaSim, in which the map will be updated.
 * @param objectname
 * @return the handle.
 */
int DQ_CoppeliaSimInterface::_get_object_handle(const std::string &objectname)
{
    int handle;
    bool found;
    std::tie(found, handle) = _get_handle_from_map(objectname);
    if (found)
        return handle;
    else
        return get_object_handle(objectname); // the map is updated here.
}


/**
 * @brief DQ_CoppeliaSimInterface::_get_handle_from_map searchs a handle in the map.
 *
 * @param objectname
 * @return a tuple <bool, int>. If the handle is found in the map, returns <true, handle>.
 *                              Otherwise, returns <false, 0>
 */
std::tuple<bool, int> DQ_CoppeliaSimInterface::_get_handle_from_map(const std::string &objectname)
{
    if (auto search = set_states_map_.find(objectname); search != set_states_map_.end())
        //std::cout << "Found " << search->first << ' ' << search->second << '\n';
        return {true, search->second};
    else
        //std::cout << "Not found\n";
        return {false, 0};
}



//---------------Deprecated methods-----------------------------
void DQ_CoppeliaSimInterface::disconnect(){}
void DQ_CoppeliaSimInterface::disconnect_all(){}
void DQ_CoppeliaSimInterface::set_synchronous(const bool &flag){set_stepping_mode(flag);}
int DQ_CoppeliaSimInterface::wait_for_simulation_step_to_end(){return 0;}

//--------------------------------------------------------------

