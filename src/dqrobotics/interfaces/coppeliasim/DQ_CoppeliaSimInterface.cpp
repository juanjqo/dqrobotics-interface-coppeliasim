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
std::unique_ptr<RemoteAPIObject::sim> sim_;


void _set_status_bar_message(const std::string &message, const int& verbosity_type)
{
    sim_->addLog(verbosity_type, message);
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
    {
         client_ = std::make_unique<RemoteAPIClient>(host, rpcPort, cntPort, verbose_);
         sim_   = std::make_unique<RemoteAPIObject::sim >(client_->getObject().sim());
    }
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
                                sim_->verbosity_warnings);
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
   sim_->startSimulation();
}

/**
 * @brief DQ_CoppeliaSimInterface::pause_simulation pauses the CoppeliaSim simulation.
 */
void DQ_CoppeliaSimInterface::pause_simulation() const
{
   sim_->pauseSimulation();

}


/**
 * @brief DQ_CoppeliaSimInterface::stop_simulation stops the  CoppeliaSim simulation.
 */
void DQ_CoppeliaSimInterface::stop_simulation() const
{
   sim_->stopSimulation();
}


/**
 * @brief DQ_CoppeliaSimInterface::set_stepping_mode enables or disables the stepping mode
 *        (formerly known as synchronous mode).
 * @param flag. Eg: set_stepping_mode(true)  // enables the stepping mode
 *                  set_stepping_mode(false)  // disables the stepping mode
 */
void DQ_CoppeliaSimInterface::set_stepping_mode(const bool &flag)
{
   sim_->setStepping(flag);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_simulation_time returns the simulation time.
 *        This time does not correspond to the real-time necessarily.
 * @return The simulation time.
 */
double DQ_CoppeliaSimInterface::get_simulation_time() const
{
    return sim_->getSimulationTime();
}

/**
 * @brief DQ_CoppeliaSimInterface::trigger_next_simulation_step This method sends a trigger
 *        signal to the CoppeliaSim scene, which performs a simulation step when the stepping mode is used.
 */
void DQ_CoppeliaSimInterface::trigger_next_simulation_step() const
{
    sim_->step();
}



/**
 * @brief DQ_CoppeliaSimInterface::is_simulation_running checks if the simulation is running.
 * @return True if the simulation is running. False otherwise.
 */
bool DQ_CoppeliaSimInterface::is_simulation_running() const
{
    return (sim_->getSimulationState() > sim_->simulation_paused);
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
    return sim_->getSimulationState();
}

/**
 * @brief DQ_CoppeliaSimInterface::set_status_bar_message sends a message to CoppeliaSim to be
 *        displayed in the status bar.
 *
 * @param message
 */
void DQ_CoppeliaSimInterface::set_status_bar_message(const std::string &message) const
{
    _set_status_bar_message(message, sim_->verbosity_undecorated);
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
        handle = sim_->getObject(objectname);
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
DQ DQ_CoppeliaSimInterface::get_object_translation(const int &handle) const
{
    auto position = sim_->getObjectPosition(handle, sim_->handle_world);
    const DQ t = DQ(0, position.at(0),position.at(1),position.at(2));
    return t;
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
 * @brief DQ_CoppeliaSimInterface::set_object_translation sets the translation of a handle
 *        in the CoppeliaSim scene.
 * @param handle
 * @param t desired position.
 */
void DQ_CoppeliaSimInterface::set_object_translation(const int &handle, const DQ &t)
{
    VectorXd vec_t = t.vec3();
    std::vector<double> position = {vec_t[0], vec_t[1],vec_t[2]};
    sim_->setObjectPosition(handle, position,sim_->handle_world);
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
DQ DQ_CoppeliaSimInterface::get_object_rotation(const int &handle) const
{
    auto rotation = sim_->getObjectQuaternion(handle +
                                              sim_->handleflag_wxyzquat,
                                              sim_->handle_world);

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
    sim_->setObjectQuaternion(handle + sim_->handleflag_wxyzquat, rotation, sim_->handle_world);
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
DQ DQ_CoppeliaSimInterface::get_object_pose(const int &handle) const
{
    DQ t = get_object_translation(handle);
    DQ r = get_object_rotation(handle);
    return r + 0.5*E_*t*r;
}

DQ DQ_CoppeliaSimInterface::get_object_pose(const int &handle, const int &relative_to_handle) const
{
    DQ x1 = get_object_pose(handle);
    DQ x2 = get_object_pose(relative_to_handle);
    return x2.conj()*x1;
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
 * @brief DQ_CoppeliaSimInterface::get_object_pose
 * @param objectname
 * @param relative_to_objectname
 * @return
 */
DQ DQ_CoppeliaSimInterface::get_object_pose(const std::string &objectname, const std::string &relative_to_objectname)
{
    return get_object_pose(_get_object_handle(objectname), _get_object_handle(relative_to_objectname));
}

/**
 * @brief DQ_CoppeliaSimInterface::set_object_pose
 * @param handle
 * @param h
 */
void DQ_CoppeliaSimInterface::set_object_pose(const int &handle, const DQ &h)
{
    VectorXd vec_r = h.P().vec4();
    VectorXd vec_p = h.translation().vec3();
    std::vector<double> pose = {vec_p[0], vec_p[1],vec_p[2],vec_r[0], vec_r[1],vec_r[2], vec_r[3]};
    sim_->setObjectPose(handle + sim_->handleflag_wxyzquat, pose, sim_->handle_world);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_object_pose
 * @param objectname
 * @param h
 */
void DQ_CoppeliaSimInterface::set_object_pose(const std::string &objectname, const DQ &h)
{
    set_object_pose(_get_object_handle(objectname), h);
}


/**
 * @brief DQ_CoppeliaSimInterface::get_joint_position
 * @param handle
 * @return
 */
double DQ_CoppeliaSimInterface::get_joint_position(const int &handle) const
{
    return double(sim_->getJointPosition(handle));
}


/**
 * @brief DQ_CoppeliaSimInterface::get_joint_position
 * @param jointname
 * @return
 */
double DQ_CoppeliaSimInterface::get_joint_position(const std::string &jointname)
{
    return get_joint_position(_get_object_handle(jointname));
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_positions
 * @param handles
 * @return
 */
VectorXd DQ_CoppeliaSimInterface::get_joint_positions(const std::vector<int> &handles) const
{
    std::size_t n = handles.size();
    VectorXd joint_positions(n);
    for(std::size_t i=0;i<n;i++)
    {
        joint_positions(i)=get_joint_position(handles.at(i));
    }
    return joint_positions;
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_positions
 * @param jointnames
 * @return
 */
VectorXd DQ_CoppeliaSimInterface::get_joint_positions(const std::vector<std::string> &jointnames)
{
    std::size_t n = jointnames.size();
    VectorXd joint_positions(n);
    for(std::size_t i=0;i<n;i++)
    {
        joint_positions(i)=get_joint_position(jointnames[i]);
    }
    return joint_positions;
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_position
 * @param handle
 * @param angle_rad
 */
void DQ_CoppeliaSimInterface::set_joint_position(const int &handle, const double &angle_rad) const
{
    sim_->setJointPosition(handle, angle_rad);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_position
 * @param jointname
 * @param angle_rad
 */
void DQ_CoppeliaSimInterface::set_joint_position(const std::string &jointname, const double &angle_rad)
{
    set_joint_position(_get_object_handle(jointname), angle_rad);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_positions
 * @param handles
 * @param angles_rad
 */
void DQ_CoppeliaSimInterface::set_joint_positions(const std::vector<int> &handles, const VectorXd &angles_rad) const
{
    for(std::size_t i=0;i<handles.size();i++)
        set_joint_position(handles.at(i), angles_rad(i));
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_positions
 * @param jointnames
 * @param angles_rad
 */
void DQ_CoppeliaSimInterface::set_joint_positions(const std::vector<std::string> &jointnames, const VectorXd &angles_rad)
{
     _check_sizes(jointnames, angles_rad, "Error in DQ_CoppeliaSimInterface::set_joint_positions: "
                                          "jointnames and angles_rad have incompatible sizes");
    for(std::size_t i=0;i<jointnames.size();i++)
         set_joint_position(jointnames.at(i), angles_rad(i));
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_target_position
 * @param handle
 * @param angle_rad
 */
void DQ_CoppeliaSimInterface::set_joint_target_position(const int &handle, const double &angle_rad) const
{
    sim_->setJointTargetPosition(handle, angle_rad);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_target_position
 * @param jointname
 * @param angle_rad
 */
void DQ_CoppeliaSimInterface::set_joint_target_position(const std::string &jointname, const double &angle_rad)
{
    set_joint_target_position(_get_object_handle(jointname), angle_rad);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_target_positions
 * @param handles
 * @param angles_rad
 */
void DQ_CoppeliaSimInterface::set_joint_target_positions(const std::vector<int> &handles, const VectorXd &angles_rad) const
{
    for(std::size_t i=0;i<handles.size();i++)
        set_joint_target_position(handles.at(i), angles_rad(i));
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_target_positions
 * @param jointnames
 * @param angles_rad
 */
void DQ_CoppeliaSimInterface::set_joint_target_positions(const std::vector<std::string> &jointnames, const VectorXd &angles_rad)
{
    _check_sizes(jointnames, angles_rad, "Error in DQ_CoppeliaSimInterface::set_joint_target_positions: "
                                         "jointnames and angles_rad have incompatible sizes");
    for(std::size_t i=0;i<jointnames.size();i++)
        set_joint_target_position(jointnames.at(i), angles_rad(i));
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_velocity
 * @param handle
 * @return
 */
double DQ_CoppeliaSimInterface::get_joint_velocity(const int &handle) const
{
    return sim_->getObjectFloatParam(handle, sim_->jointfloatparam_velocity);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_velocity
 * @param jointname
 * @return
 */
double DQ_CoppeliaSimInterface::get_joint_velocity(const std::string &jointname)
{
    return get_joint_velocity(_get_object_handle(jointname));
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_velocties
 * @param handles
 * @return
 */
VectorXd DQ_CoppeliaSimInterface::get_joint_velocities(const std::vector<int> &handles) const
{
    std::size_t n = handles.size();
    VectorXd joint_velocities(n);
    for(std::size_t i=0;i<n;i++)
        joint_velocities(i)=get_joint_velocity(handles.at(i));

    return joint_velocities;
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_velocities
 * @param jointnames
 * @return
 */
VectorXd DQ_CoppeliaSimInterface::get_joint_velocities(const std::vector<std::string> &jointnames)
{
    std::size_t n = jointnames.size();
    VectorXd joint_velocities(n);
    for(std::size_t i=0;i<n;i++)
    {
        joint_velocities(i)=get_joint_velocity(jointnames[i]);
    }
    return joint_velocities;
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_target_velocity
 * @param handle
 * @param angle_rad_dot
 */
void DQ_CoppeliaSimInterface::set_joint_target_velocity(const int &handle, const double &angle_rad_dot) const
{
    sim_->setJointTargetVelocity(handle, angle_rad_dot);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_target_velocity
 * @param jointname
 * @param angle_rad_dot
 */
void DQ_CoppeliaSimInterface::set_joint_target_velocity(const std::string &jointname, const double &angle_rad_dot)
{
    set_joint_target_velocity(_get_object_handle(jointname), angle_rad_dot);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_target_velocities
 * @param handles
 * @param angles_rad_dot
 */
void DQ_CoppeliaSimInterface::set_joint_target_velocities(const std::vector<int> &handles, const VectorXd &angles_rad_dot) const
{
    for(std::size_t i=0;i<handles.size();i++)
        set_joint_target_velocity(handles.at(i), angles_rad_dot(i));
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_target_velocities
 * @param jointnames
 * @param angles_rad_dot
 */
void DQ_CoppeliaSimInterface::set_joint_target_velocities(const std::vector<std::string> &jointnames, const VectorXd &angles_rad_dot)
{
    _check_sizes(jointnames, angles_rad_dot, "Error in DQ_CoppeliaSimInterface::set_joint_target_velocities: "
                                             "jointnames and angles_rad_Dot have incompatible sizes");
    for(std::size_t i=0;i<jointnames.size();i++)
        set_joint_target_velocity(jointnames.at(i), angles_rad_dot(i));
}

/**
 * @brief DQ_CoppeliaSimInterface::get_object_name
 * @param handle
 * @return
 */
std::string DQ_CoppeliaSimInterface::get_object_name(const int &handle)
{
    std::string objectname = sim_->getObjectAlias(handle, 1);
    _update_map(objectname, handle);
    return objectname;
}


/**
 * @brief DQ_CoppeliaSimInterface::get_jointnames_from_base_objectname
 * @param base_objectname
 * @return
 */
std::vector<std::string> DQ_CoppeliaSimInterface::get_jointnames_from_base_objectname(const std::string &base_objectname)
{
    int base_handle = _get_object_handle(base_objectname);
    std::vector<int64_t> jointhandles = sim_->getObjectsInTree(base_handle,
                                        sim_->object_joint_type,
                                        0);
    std::size_t n = jointhandles.size();
    std::vector<std::string> jointnames(n);
    for(std::size_t i=0;i<n;i++)
    {
        jointnames.at(i)=get_object_name(jointhandles.at(i));
    }
    return jointnames;

}


/**
 * @brief DQ_CoppeliaSimInterface::set_joint_mode
 * @param jointname
 * @param joint_mode
 */
void DQ_CoppeliaSimInterface::set_joint_mode(const std::string &jointname, const JOINT_MODE &joint_mode)
{
    int jointMode;
    switch (joint_mode)
        {
            case KINEMATIC:
                jointMode = sim_->jointmode_kinematic;
                break;
            case DYNAMIC:
                jointMode = sim_->jointmode_dynamic;
                break;
            case DEPENDENT:
                jointMode = sim_->jointmode_dependent;
                break;
        }
        sim_->setJointMode(_get_object_handle(jointname), jointMode, 0);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_modes
 * @param jointnames
 * @param joint_mode
 */
void DQ_CoppeliaSimInterface::set_joint_modes(const std::vector<std::string> &jointnames, const JOINT_MODE &joint_mode)
{
    for(std::size_t i=0;i<jointnames.size();i++)
        set_joint_mode(jointnames.at(i), joint_mode);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_control_mode
 * @param jointname
 * @param joint_control_mode
 */
void DQ_CoppeliaSimInterface::set_joint_control_mode(const std::string &jointname, const JOINT_CONTROL_MODE &joint_control_mode)
{
    int64_t control_mode;
    switch (joint_control_mode)
    {

    case FREE:
        control_mode = sim_->jointdynctrl_free;
        break;
    case FORCE:
        control_mode = sim_->jointdynctrl_force;
        break;
    case VELOCITY:
        control_mode = sim_->jointdynctrl_velocity;
        break;
    case POSITION:
        control_mode = sim_->jointdynctrl_position;
        break;
    case SPRING:
        control_mode = sim_->jointdynctrl_spring;
        break;
    case CUSTOM:
        control_mode = sim_->jointdynctrl_callback;
        break;
    }
    sim_->setObjectInt32Param(_get_object_handle(jointname),
                              sim_->jointintparam_dynctrlmode,
                              control_mode);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_control_modes
 * @param jointnames
 * @param joint_control_mode
 */
void DQ_CoppeliaSimInterface::set_joint_control_modes(const std::vector<std::string> &jointnames, const JOINT_CONTROL_MODE &joint_control_mode)
{
    for(std::size_t i=0;i<jointnames.size();i++)
        set_joint_control_mode(jointnames.at(i), joint_control_mode);
}

/**
 * @brief DQ_CoppeliaSimInterface::enable_dynamics_engine
 * @param flag
 */
void DQ_CoppeliaSimInterface::enable_dynamics(const bool &flag)
{
   sim_->setBoolParam(sim_->boolparam_dynamics_handling_enabled, flag);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_simulation_time_step
 * @return
 */
double DQ_CoppeliaSimInterface::get_simulation_time_step() const
{
    return sim_->getFloatParam(sim_->floatparam_simulation_time_step);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_simulation_time_step
 * @param time_step
 */
void DQ_CoppeliaSimInterface::set_simulation_time_step(const double &time_step)
{
    sim_->setFloatParam(sim_->floatparam_simulation_time_step, time_step);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_physics_time_step
 * @return
 */
double DQ_CoppeliaSimInterface::get_physics_time_step() const
{
    return sim_->getFloatParam(sim_->floatparam_physicstimestep);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_physics_time_step
 * @param time_step
 */
void DQ_CoppeliaSimInterface::set_physics_time_step(const double &time_step) const
{
    sim_->setFloatParam(sim_->floatparam_physicstimestep, time_step);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_dynamic_engine
 * @param engine
 */
void DQ_CoppeliaSimInterface::set_dynamic_engine(const ENGINE &engine)
{
    sim_->setInt32Param(sim_->intparam_dynamic_engine, engine);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_gravity
 * @param gravity
 */
void DQ_CoppeliaSimInterface::set_gravity(const DQ &gravity)
{
    VectorXd gravity_vec = gravity.vec3();
    std::vector<double> g = {gravity_vec(0),gravity_vec(1), gravity_vec(2)};
    sim_->setArrayParam(sim_->arrayparam_gravity, g);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_gravity
 * @return
 */
DQ DQ_CoppeliaSimInterface::get_gravity() const
{
    std::vector<double> g = sim_->getArrayParam(sim_->arrayparam_gravity);
    return DQ(0, g.at(0), g.at(1), g.at(2));
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

