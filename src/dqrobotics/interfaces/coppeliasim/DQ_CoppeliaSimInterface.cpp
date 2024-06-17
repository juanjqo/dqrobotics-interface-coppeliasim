/**
(C) Copyright 2024 DQ Robotics Developers

This file is based on DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Juan Jose Quiroz Omana
       - Responsible for the original implementation.
         The DQ_CoppeliaSimInterface class is partially based on the DQ_VrepInterface class
         (https://github.com/dqrobotics/cpp-interface-vrep/blob/master/include/dqrobotics/interfaces/vrep/DQ_VrepInterface.h)

*/

#include "dqrobotics/DQ.h"
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

DQ_CoppeliaSimInterface::~DQ_CoppeliaSimInterface()
{
    _join_if_joinable_chronometer_thread();
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
         sim_    = std::make_unique<RemoteAPIObject::sim >(client_->getObject().sim());
    }
}

/**
 * @brief DQ_CoppeliaSimInterface::_join_if_joinable_chronometer_thread
 */
void DQ_CoppeliaSimInterface::_join_if_joinable_chronometer_thread()
{
    if (chronometer_thread_.joinable())
        chronometer_thread_.join();
}

/**
 * @brief DQ_CoppeliaSimInterface::_start_chronometer
 */
void DQ_CoppeliaSimInterface::_start_chronometer()
{
    auto initial_time_ = std::chrono::steady_clock::now();
    while(elapsed_time_ < MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_)
    {
        auto end{std::chrono::steady_clock::now()};
        auto elapsed_seconds_ = std::chrono::duration<double>{end - initial_time_};
        elapsed_time_ = elapsed_seconds_.count()*1e3;
    }
    _check_connection();
}

/**
 * @brief DQ_CoppeliaSimInterface::_check_connection
 */
void DQ_CoppeliaSimInterface::_check_connection()
{
    if (!client_created_)
    {
        std::cerr<<"Unestablished connection at "+ host_
                         + " in port " + std::to_string(rpcPort_)<<std::endl;
        std::cerr<<"You used a wait time of "+std::to_string(MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_) + "ms. Is enough time for your system?"<<std::endl;
        if(rpcPort_ != 23000)
        {
            std::cerr<<""<<std::endl;
            std::cerr<<"is CoppeliaSim running with the port "<<std::to_string(rpcPort_)<<" enabled?"<<std::endl;
            std::cerr<<""<<std::endl;
            std::cerr<<"Example: using the terminal, open CoppeliaSim with arguments:"<<std::endl;
            std::cerr<<"----------------------------------------"<<std::endl;
            std::cerr<<"coppeliasim -GzmqRemoteApi.rpcPort="+std::to_string(rpcPort_)<<std::endl;
            std::cerr<<"----------------------------------------"<<std::endl;
        }

        std::cerr<<""<<std::endl;

        throw std::runtime_error("Unestablished connection.");
    }
}


/**
 * @brief DQ_CoppeliaSimInterface::connect establish a connection between the client (your code) and
 *                                         the host (the CoppeliaSim scene).
 * @param host    eg. 'localhost' if the host is running in the same
 *                machine in which is running the client.
 * @param rpcPort The port to establish a connection. (e.g. 23000, 23001, 23002, 23003...).
 * @param cntPort
 * @param verbose
 * @return
 */
bool DQ_CoppeliaSimInterface::connect(const std::string &host, const int &rpcPort, const int &MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION, const int &cntPort, const int &verbose)
{
    try
    {
        host_ = host;
        rpcPort_ = rpcPort;
        cntPort_ = cntPort;
        verbose_ = verbose;
        MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_ = MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION;

        _join_if_joinable_chronometer_thread();
        chronometer_thread_ = std::thread(&DQ_CoppeliaSimInterface::_start_chronometer, this);


        _create_client(host, rpcPort, cntPort, verbose, client_created_);
        client_created_ = true;

        _join_if_joinable_chronometer_thread();
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
    return client_created_;
}

/**
 * @brief DQ_CoppeliaSimInterface::start_simulation starts the CoppeliaSim simulation.
 */
void DQ_CoppeliaSimInterface::start_simulation() const
{
    _check_client();
    sim_->startSimulation();
}

/**
 * @brief DQ_CoppeliaSimInterface::pause_simulation pauses the CoppeliaSim simulation.
 */
void DQ_CoppeliaSimInterface::pause_simulation() const
{
    _check_client();
    sim_->pauseSimulation();

}


/**
 * @brief DQ_CoppeliaSimInterface::stop_simulation stops the  CoppeliaSim simulation.
 */
void DQ_CoppeliaSimInterface::stop_simulation() const
{
    _check_client();
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
    _check_client();
    sim_->setStepping(flag);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_simulation_time returns the simulation time.
 *        This time does not correspond to the real-time necessarily.
 * @return The simulation time.
 */
double DQ_CoppeliaSimInterface::get_simulation_time() const
{
    _check_client();
    return sim_->getSimulationTime();
}

/**
 * @brief DQ_CoppeliaSimInterface::trigger_next_simulation_step This method sends a trigger
 *        signal to the CoppeliaSim scene, which performs a simulation step when the stepping mode is used.
 */
void DQ_CoppeliaSimInterface::trigger_next_simulation_step() const
{
    _check_client();
    sim_->step();
}



/**
 * @brief DQ_CoppeliaSimInterface::is_simulation_running checks if the simulation is running.
 * @return True if the simulation is running. False otherwise.
 */
bool DQ_CoppeliaSimInterface::is_simulation_running() const
{
    _check_client();
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
    _check_client();
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
    _check_client();
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
    std::string additional_error_message = "";
    if (!_start_with_slash(objectname) && enable_deprecated_name_compatibility_ == false)
    {
        additional_error_message = std::string("Did you mean \"/" + objectname + "\"? \n");
    }
    try
    {
        _check_client();
        auto standard_objectname = _get_standard_name(objectname);
        handle = sim_->getObject(standard_objectname );
        _update_map(standard_objectname, handle);
    }
    catch(const std::runtime_error& e)
    {
        auto error_msg =
            std::string(e.what())
            + " \n"
            + std::string("The object \"")
            + objectname + std::string("\"")
            + std::string(" does not exist in the current scene in CoppeliaSim. \n")
            + additional_error_message;
        _throw_runtime_error(error_msg);
    }
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
    for(auto i=0;i<n;i++)
        handles[i]=get_object_handle(objectnames[i]);

    return handles;
}

std::unordered_map<std::string, int> DQ_CoppeliaSimInterface::get_map()
{
    return handles_map_;
}

void DQ_CoppeliaSimInterface::show_map()
{
    for (const auto& p : handles_map_)
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
    _check_client();
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
    return get_object_translation(_get_handle_from_map(objectname));
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
    _check_client();
    sim_->setObjectPosition(handle, position,sim_->handle_world);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_object_translation
 * @param objectname
 * @param t
 */
void DQ_CoppeliaSimInterface::set_object_translation(const std::string &objectname, const DQ &t)
{
    set_object_translation(_get_handle_from_map(objectname), t);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_object_rotation
 * @param handle
 * @return the object rotation
 */
DQ DQ_CoppeliaSimInterface::get_object_rotation(const int &handle) const
{
    _check_client();
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
    return get_object_rotation(_get_handle_from_map(objectname));
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
    _check_client();
    sim_->setObjectQuaternion(handle + sim_->handleflag_wxyzquat, rotation, sim_->handle_world);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_object_rotation
 * @param objectname
 * @param r
 */
void DQ_CoppeliaSimInterface::set_object_rotation(const std::string &objectname, const DQ &r)
{
    set_object_rotation(_get_handle_from_map(objectname), r);
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

/**
 * @brief DQ_CoppeliaSimInterface::get_object_pose
 * @param objectname
 * @return
 */
DQ DQ_CoppeliaSimInterface::get_object_pose(const std::string &objectname)
{
    return get_object_pose(_get_handle_from_map(objectname));
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
    _check_client();
    sim_->setObjectPose(handle + sim_->handleflag_wxyzquat, pose, sim_->handle_world);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_object_pose
 * @param objectname
 * @param h
 */
void DQ_CoppeliaSimInterface::set_object_pose(const std::string &objectname, const DQ &h)
{
    if (!is_unit(h))
        _throw_runtime_error(static_cast<std::string>(std::source_location::current().function_name())
                             + ". The pose must be a unit dual quaternion!");
    set_object_pose(_get_handle_from_map(objectname), h);
}


/**
 * @brief DQ_CoppeliaSimInterface::get_joint_position
 * @param handle
 * @return
 */
double DQ_CoppeliaSimInterface::get_joint_position(const int &handle) const
{
    _check_client();
    return double(sim_->getJointPosition(handle));
}


/**
 * @brief DQ_CoppeliaSimInterface::get_joint_position
 * @param jointname
 * @return
 */
double DQ_CoppeliaSimInterface::get_joint_position(const std::string &jointname)
{
    return get_joint_position(_get_handle_from_map(jointname));
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_positions
 * @param handles
 * @return
 */
VectorXd DQ_CoppeliaSimInterface::get_joint_positions(const std::vector<int> &handles) const
{
    int n = handles.size();
    VectorXd joint_positions(n);
    for(auto i=0;i<n;i++)
        joint_positions(i)=get_joint_position(handles.at(i));

    return joint_positions;
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_positions
 * @param jointnames
 * @return
 */
VectorXd DQ_CoppeliaSimInterface::get_joint_positions(const std::vector<std::string> &jointnames)
{
    int n = jointnames.size();
    VectorXd joint_positions(n);
    for(auto i=0;i<n;i++)
        joint_positions(i)=get_joint_position(jointnames[i]);

    return joint_positions;
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_position
 * @param handle
 * @param angle_rad
 */
void DQ_CoppeliaSimInterface::set_joint_position(const int &handle, const double &angle_rad) const
{
    _check_client();
    sim_->setJointPosition(handle, angle_rad);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_position
 * @param jointname
 * @param angle_rad
 */
void DQ_CoppeliaSimInterface::set_joint_position(const std::string &jointname, const double &angle_rad)
{
    set_joint_position(_get_handle_from_map(jointname), angle_rad);
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
    _check_client();
    sim_->setJointTargetPosition(handle, angle_rad);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_target_position
 * @param jointname
 * @param angle_rad
 */
void DQ_CoppeliaSimInterface::set_joint_target_position(const std::string &jointname, const double &angle_rad)
{
    set_joint_target_position(_get_handle_from_map(jointname), angle_rad);
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
    _check_client();
    return sim_->getObjectFloatParam(handle, sim_->jointfloatparam_velocity);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_velocity
 * @param jointname
 * @return
 */
double DQ_CoppeliaSimInterface::get_joint_velocity(const std::string &jointname)
{
    return get_joint_velocity(_get_handle_from_map(jointname));
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_velocties
 * @param handles
 * @return
 */
VectorXd DQ_CoppeliaSimInterface::get_joint_velocities(const std::vector<int> &handles) const
{
    int n = handles.size();
    VectorXd joint_velocities(n);
    for(auto i=0;i<n;i++)
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
    int n = jointnames.size();
    VectorXd joint_velocities(n);
    for(auto i=0;i<n;i++)
        joint_velocities(i)=get_joint_velocity(jointnames[i]);

    return joint_velocities;
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_target_velocity
 * @param handle
 * @param angle_rad_dot
 */
void DQ_CoppeliaSimInterface::set_joint_target_velocity(const int &handle, const double &angle_rad_dot) const
{
    _check_client();
    sim_->setJointTargetVelocity(handle, angle_rad_dot);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_target_velocity
 * @param jointname
 * @param angle_rad_dot
 */
void DQ_CoppeliaSimInterface::set_joint_target_velocity(const std::string &jointname, const double &angle_rad_dot)
{
    set_joint_target_velocity(_get_handle_from_map(jointname), angle_rad_dot);
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
 * @brief DQ_CoppeliaSimInterface::set_joint_torque
 * @param handle
 * @param torque
 */
void DQ_CoppeliaSimInterface::set_joint_torque(const int &handle, const double &torque) const
{
    _check_client();
    double angle_dot_rad_max = 10000.0;
    if (torque==0)
        angle_dot_rad_max = 0.0;
    else if (torque<0)
        angle_dot_rad_max = -10000.0;

    //simxSetJointTargetVelocity(clientid_,handle,angle_dot_rad_max,_remap_op_mode(opmode));
    //simxSetJointForce(clientid_,handle,abs(torque_f),_remap_op_mode(opmode));
    sim_->setJointTargetVelocity(handle, angle_dot_rad_max);
    sim_->setJointTargetForce(handle, abs(torque));
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_torque
 * @param jointname
 * @param torque
 */
void DQ_CoppeliaSimInterface::set_joint_torque(const std::string &jointname, const double &torque)
{
    set_joint_torque(_get_handle_from_map(jointname), torque);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_torques
 * @param handles
 * @param torques
 */
void DQ_CoppeliaSimInterface::set_joint_torques(const std::vector<int> &handles, const VectorXd &torques) const
{
    for(std::size_t i=0;i<handles.size();i++)
        set_joint_torque(handles.at(i), torques(i));
}

/**
 * @brief DQ_CoppeliaSimInterface::set_joint_torques
 * @param jointnames
 * @param torques
 */
void DQ_CoppeliaSimInterface::set_joint_torques(const std::vector<std::string> &jointnames, const VectorXd &torques)
{
    _check_sizes(jointnames, torques, "Error in DQ_CoppeliaSimInterface::set_joint_torques: "
                                             "jointnames and torques have incompatible sizes");
    for(std::size_t i=0;i<jointnames.size();i++)
        set_joint_torque(jointnames.at(i), torques(i));
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_torque
 * @param handle
 * @return
 */
double DQ_CoppeliaSimInterface::get_joint_torque(const int &handle) const
{
    _check_client();
    return sim_->getJointForce(handle);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_torque
 * @param jointname
 * @return
 */
double DQ_CoppeliaSimInterface::get_joint_torque(const std::string &jointname)
{
    return get_joint_torque(_get_handle_from_map(jointname));
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_torques
 * @param handles
 * @return
 */
VectorXd DQ_CoppeliaSimInterface::get_joint_torques(const std::vector<int> &handles) const
{
    int n = handles.size();
    VectorXd joint_torques(n);
    for(auto i=0;i<n;i++)
        joint_torques(i)=get_joint_torque(handles.at(i));

    return joint_torques;
}

/**
 * @brief DQ_CoppeliaSimInterface::get_joint_torques
 * @param jointnames
 * @return
 */
VectorXd DQ_CoppeliaSimInterface::get_joint_torques(const std::vector<std::string> &jointnames)
{
    int n = jointnames.size();
    VectorXd joint_torques(n);
    for(auto i=0;i<n;i++)
        joint_torques(i)=get_joint_torque(jointnames[i]);

    return joint_torques;
}

/**
 * @brief DQ_CoppeliaSimInterface::get_object_name
 * @param handle
 * @return
 */
std::string DQ_CoppeliaSimInterface::get_object_name(const int &handle)
{
    _check_client();
    std::string objectname = sim_->getObjectAlias(handle, 1);
    _update_map(objectname, handle);
    return objectname;
}

/**
 * @brief DQ_CoppeliaSimInterface::get_object_names
 * @param handles
 * @return
 */
std::vector<std::string> DQ_CoppeliaSimInterface::get_object_names(const auto &handles)
{
    int n = handles.size();
    std::vector<std::string> objectnames(n);
    for(auto i=0;i<n;i++)
        objectnames.at(i)=get_object_name(handles.at(i));

    return objectnames;
}


/**
 * @brief DQ_CoppeliaSimInterface::get_jointnames_from_base_objectname
 * @param base_objectname
 * @return
 */
std::vector<std::string> DQ_CoppeliaSimInterface::get_jointnames_from_base_objectname(const std::string &base_objectname)
{
    int base_handle = _get_handle_from_map(base_objectname);
    _check_client();
    std::vector<int64_t> jointhandles = sim_->getObjectsInTree(base_handle,
                                        sim_->object_joint_type,
                                        0);
    return get_object_names(jointhandles);

}

std::vector<std::string> DQ_CoppeliaSimInterface::get_linknames_from_base_objectname(const std::string &base_objectname)
{
    int base_handle = _get_handle_from_map(base_objectname);
    _check_client();
    std::vector<int64_t> shapehandles = sim_->getObjectsInTree(base_handle,
                                                               sim_->object_shape_type,
                                                               0);
    return get_object_names(shapehandles);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_angular_and_linear_velocities
 * @param handle
 * @param reference
 * @return
 */
VectorXd DQ_CoppeliaSimInterface::get_angular_and_linear_velocities(const int &handle, const REFERENCE &reference) const
{
    std::vector<int> params = _get_velocity_const_params();
    VectorXd v = VectorXd::Zero(params.size());
    _check_client();
    for (size_t i=0; i < params.size(); i++)
    {
        v(i) = sim_->getObjectFloatParam(handle, params.at(i));
    }
    if (reference == REFERENCE::BODY_FRAME)
    {
        DQ x = get_object_pose(handle);
        DQ r = x.P();
        DQ w_b = r.conj()*DQ(v.head(3))*r;
        DQ p_dot_b = r.conj()*DQ(v.tail(3))*r;
        v.head(3) = w_b.vec3();
        v.tail(3) = p_dot_b.vec3();
    }
    return v;
}

/**
 * @brief DQ_CoppeliaSimInterface::get_angular_and_linear_velocities
 * @param objectname
 * @param reference
 * @return
 */
VectorXd DQ_CoppeliaSimInterface::get_angular_and_linear_velocities(std::string &objectname, const REFERENCE &reference)
{
    return get_angular_and_linear_velocities(_get_handle_from_map(objectname), reference);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_angular_and_linear_velocities
 * @param handle
 * @param w
 * @param p_dot
 * @param reference
 */
void DQ_CoppeliaSimInterface::set_angular_and_linear_velocities(const int &handle, const DQ &w, const DQ &p_dot, const REFERENCE &reference) const
{
    std::vector<int> params = _get_velocity_const_params();
    VectorXd v = VectorXd::Zero(params.size());

    if (reference == REFERENCE::ABSOLUTE_FRAME)
    {
        const DQ& w_a = w;
        const DQ& p_dot_a = p_dot;
        v.head(3) = w_a.vec3();
        v.tail(3) = p_dot_a.vec3();
    }else
    {
        DQ x = get_object_pose(handle);
        DQ r = x.P();
        DQ w_a = r*w*r.conj();
        DQ p_dot_a = r*p_dot*r.conj();
        v.head(3) = w_a.vec3();
        v.tail(3) = p_dot_a.vec3();
    }
    _check_client();
    sim_->resetDynamicObject(handle);
    for (size_t i=0; i < params.size(); i++)
    {
        sim_->setObjectFloatParam(handle, params.at(i), v(i));
    }
}

/**
 * @brief DQ_CoppeliaSimInterface::set_angular_and_linear_velocities
 * @param objectname
 * @param w
 * @param p_dot
 * @param reference
 */
void DQ_CoppeliaSimInterface::set_angular_and_linear_velocities(std::string &objectname, const DQ &w, const DQ &p_dot, const REFERENCE &reference)
{
    set_angular_and_linear_velocities(_get_handle_from_map(objectname), w, p_dot, reference);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_twist
 * @param handle
 * @param reference
 * @return
 */
DQ DQ_CoppeliaSimInterface::get_twist(const int &handle, const REFERENCE &reference) const
{
    VectorXd v = get_angular_and_linear_velocities(handle);
    DQ w = DQ(v.head(3));
    DQ p_dot = DQ(v.tail(3));
    DQ x = get_object_pose(handle);
    DQ twist =  w + E_*(p_dot + cross(x.translation(), w));;
    if (reference == REFERENCE::BODY_FRAME)
    {
       twist =  x.conj()*twist*x;
    }
    return twist;
}

/**
 * @brief DQ_CoppeliaSimInterface::get_twist
 * @param objectname
 * @param reference
 * @return
 */
DQ DQ_CoppeliaSimInterface::get_twist(const std::string &objectname, const REFERENCE &reference)
{
    return get_twist(_get_handle_from_map(objectname), reference);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_twist
 * @param handle
 * @param twist
 * @param reference
 */
void DQ_CoppeliaSimInterface::set_twist(const int &handle, const DQ& twist, const REFERENCE &reference) const
{
    if (!is_pure(twist))
    {
        throw(std::range_error("Bad set_object_twist() call: Not a pure dual quaternion"));
    }
    if (reference == REFERENCE::BODY_FRAME)
    {
        set_angular_and_linear_velocities(handle, twist.P(), twist.D(),
                                          REFERENCE::BODY_FRAME);
    }
    else{
        DQ x = get_object_pose(handle);
        set_angular_and_linear_velocities(handle, twist.P(), twist.D()-cross(x.translation(), twist.P()),
                                          REFERENCE::ABSOLUTE_FRAME);
    }
}

/**
 * @brief DQ_CoppeliaSimInterface::set_twist
 * @param objectname
 * @param twist
 * @param reference
 */
void DQ_CoppeliaSimInterface::set_twist(const std::string &objectname, const DQ &twist, const REFERENCE &reference)
{
    set_twist(_get_handle_from_map(objectname), twist, reference);
}




/**
 * @brief DQ_CoppeliaSimInterface::set_joint_mode
 * @param jointname
 * @param joint_mode
 */
void DQ_CoppeliaSimInterface::set_joint_mode(const std::string &jointname, const JOINT_MODE &joint_mode)
{
    _check_client();
    int jointMode;
    switch (joint_mode)
        {
        case JOINT_MODE::KINEMATIC:
                    jointMode = sim_->jointmode_kinematic;
                    break;
        case JOINT_MODE::DYNAMIC:
                    jointMode = sim_->jointmode_dynamic;
                    break;
        case JOINT_MODE::DEPENDENT:
                    jointMode = sim_->jointmode_dependent;
                    break;
        }
        sim_->setJointMode(_get_handle_from_map(jointname), jointMode, 0);
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
    _check_client();
    int64_t control_mode;
    switch (joint_control_mode)
    {
    case JOINT_CONTROL_MODE::FREE:
        control_mode = sim_->jointdynctrl_free;
        break;
    case JOINT_CONTROL_MODE::FORCE:
        control_mode = sim_->jointdynctrl_force;
        break;
    case JOINT_CONTROL_MODE::VELOCITY:
        control_mode = sim_->jointdynctrl_velocity;
        break;
    case JOINT_CONTROL_MODE::POSITION:
        control_mode = sim_->jointdynctrl_position;
        break;
    case JOINT_CONTROL_MODE::SPRING:
        control_mode = sim_->jointdynctrl_spring;
        break;
    case JOINT_CONTROL_MODE::CUSTOM:
        control_mode = sim_->jointdynctrl_callback;
        break;
    case JOINT_CONTROL_MODE::TORQUE:
        control_mode = sim_->jointdynctrl_velocity;
        break;
    }
    sim_->setObjectInt32Param(_get_handle_from_map(jointname),
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
    {
        set_joint_control_mode(jointnames.at(i), joint_control_mode);
    }
}

/**
 * @brief DQ_CoppeliaSimInterface::enable_dynamics_engine
 * @param flag
 */
void DQ_CoppeliaSimInterface::enable_dynamics(const bool &flag)
{
   _check_client();
   sim_->setBoolParam(sim_->boolparam_dynamics_handling_enabled, flag);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_simulation_time_step
 * @return
 */
double DQ_CoppeliaSimInterface::get_simulation_time_step() const
{
    _check_client();
    return sim_->getFloatParam(sim_->floatparam_simulation_time_step);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_simulation_time_step
 * @param time_step
 */
void DQ_CoppeliaSimInterface::set_simulation_time_step(const double &time_step)
{
    _check_client();
    sim_->setFloatParam(sim_->floatparam_simulation_time_step, time_step);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_physics_time_step
 * @return
 */
double DQ_CoppeliaSimInterface::get_physics_time_step() const
{
    _check_client();
    return sim_->getFloatParam(sim_->floatparam_physicstimestep);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_physics_time_step
 * @param time_step
 */
void DQ_CoppeliaSimInterface::set_physics_time_step(const double &time_step) const
{
    _check_client();
    sim_->setFloatParam(sim_->floatparam_physicstimestep, time_step);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_dynamic_engine
 * @param engine
 */
void DQ_CoppeliaSimInterface::set_dynamic_engine(const ENGINE &engine)
{
    _check_client();
    sim_->setInt32Param(sim_->intparam_dynamic_engine, engines_[engine]);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_gravity
 * @param gravity
 */
void DQ_CoppeliaSimInterface::set_gravity(const DQ &gravity)
{
    VectorXd gravity_vec = gravity.vec3();
    std::vector<double> g = {gravity_vec(0),gravity_vec(1), gravity_vec(2)};
    _check_client();
    sim_->setArrayParam(sim_->arrayparam_gravity, g);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_gravity
 * @return
 */
DQ DQ_CoppeliaSimInterface::get_gravity() const
{
    _check_client();
    std::vector<double> g = sim_->getArrayParam(sim_->arrayparam_gravity);
    return DQ(0, g.at(0), g.at(1), g.at(2));
}

/**
 * @brief DQ_CoppeliaSimInterface::load_scene loads a scene from your computer.
 * @param path_to_filename the path to the scene. This string must containt
 *        the file extension.
 *
 *        Example:
 *
 *        load_scene("/Users/juanjqo/git/space_robot/scenes/space_robot.ttt");
 */
void DQ_CoppeliaSimInterface::load_scene(const std::string &path_to_filename) const
{
    _check_client();
    sim_->loadScene(path_to_filename);
}

/**
 * @brief DQ_CoppeliaSimInterface::save_scene saves the current scene.
 * @param path_to_filename The path where you want to save the scene including
 *        the name of the scene and its file extension.
 *
 *        Example:
 *
 *        save_scene("/Users/juanjqo/git/space_robot/scenes/space_robot2.ttt");
 */
void DQ_CoppeliaSimInterface::save_scene(const std::string &path_to_filename) const
{
    _check_client();
    sim_->saveScene(path_to_filename);
}

/**
 * @brief DQ_CoppeliaSimInterface::close_scene closes the current scene.
 */
void DQ_CoppeliaSimInterface::close_scene() const
{
    _check_client();
    sim_->closeScene();
}

/**
 * @brief DQ_CoppeliaSimInterface::load_model loads a model to
 *        the scene.
 *
 * @param path_to_filename The path to the model.
 * @param desired_model_name The name you want for the loaded model.
 * @param load_model_only_if_missing If the model exists (with the same alias)
 *                                   the model is not loaded. (Default)
 * @param remove_child_script Remove the associated child script of the model
 *                            (Default)
 * @return A boolean flag. True if the model was loaded. False otherwise.
 */
bool DQ_CoppeliaSimInterface::load_model(const std::string &path_to_filename,
                                         const std::string &desired_model_name,
                                         const bool &load_model_only_if_missing,
                                         const bool &remove_child_script)
{
    if (load_model_only_if_missing == true)
    {
        if (!object_exist_on_scene(std::string("/") +
            _remove_first_slash_from_string(desired_model_name)))
        {
            return _load_model(path_to_filename, desired_model_name, remove_child_script);
        }else
            return true;
    }else
    {// Load the model even if the model is already on the scene
        return _load_model(path_to_filename, desired_model_name, remove_child_script);
    }

}

/**
 * @brief DQ_CoppeliaSimInterface::load_from_model_browser loads a model from
 *        the CoppeliaSim model browser.
 *
 *      Ex: load_from_model_browser("/robots/non-mobile/FrankaEmikaPanda.ttm",
                                    "/Franka");
 * @param path_to_filename The path to the model relative to the model browser.
 * @param desired_model_name The name you want for the loaded model.
 * @param load_model_only_if_missing If the model exists (with the same alias)
 *                                   the model is not loaded. (Default)
 * @param remove_child_script Remove the associated child script of the model
 *                            (Default)
 * @return A boolean flag. True if the model was loaded. False otherwise.
 */
bool DQ_CoppeliaSimInterface::load_from_model_browser(const std::string &path_to_filename,
                                                            const std::string &desired_model_name,
                                                            const bool &load_model_only_if_missing,
                                                            const bool &remove_child_script)
{
    _check_client();
    std::string resources_path = sim_->getStringParam(sim_->stringparam_resourcesdir);
    return load_model(resources_path + std::string("/models") + path_to_filename,
                      desired_model_name, load_model_only_if_missing, remove_child_script);
}

/**
 * @brief DQ_CoppeliaSimInterface::remove_child_script_from_object
 *        The script must be located at objectname/script_name
 * @param objectname
 * @param script_name
 */
void DQ_CoppeliaSimInterface::remove_child_script_from_object(const std::string &objectname, const std::string &script_name)
{
    _check_client();
    if (object_exist_on_scene(_get_standard_name(objectname)+script_name))
    {
        int handle = _get_handle_from_map(_get_standard_name(objectname)+script_name);
        sim_->removeObjects({handle}, false);
    }
}

/**
 * @brief DQ_CoppeliaSimInterface::object_exist_on_scene
 * @param objectname
 * @return
 */
bool DQ_CoppeliaSimInterface::object_exist_on_scene(const std::string &objectname) const
{
    std::optional<json> options = {{"noError", false}};
    try {
        _check_client();
        auto rtn = sim_->getObject(_get_standard_name(objectname), options);
        return (rtn != -1) ? true : false;
    } catch (...) {
        return false;
    }

}

void DQ_CoppeliaSimInterface::set_object_name(const int &handle, const std::string &new_object_name) const
{
    _check_client();
    sim_->setObjectAlias(handle, new_object_name);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_object_name
 * @param current_object_name
 * @param new_object_name
 */
void DQ_CoppeliaSimInterface::set_object_name(const std::string &current_object_name, const std::string &new_object_name)
{
    set_object_name(_get_handle_from_map(current_object_name), new_object_name);
}


void DQ_CoppeliaSimInterface::set_object_color(const int &handle,
                                               const std::vector<double> rgba_color) const
{
    _check_client();
    sim_->setShapeColor(handle, "", sim_->colorcomponent_ambient_diffuse, {rgba_color.at(0), rgba_color.at(1),rgba_color.at(2)});
    sim_->setShapeColor(handle, "", sim_->colorcomponent_transparency, {rgba_color.at(3)});
}

void DQ_CoppeliaSimInterface::set_object_color(const std::string &objectname, const std::vector<double> rgba_color)
{
    set_object_color(_get_handle_from_map(objectname), rgba_color);
}

void DQ_CoppeliaSimInterface::set_object_as_respondable(const int &handle, const bool &respondable_object) const
{
    _check_client();
    sim_->setObjectInt32Param(handle,
                              sim_->shapeintparam_respondable,
                              (respondable_object == true ? 1 : 0));
}


void DQ_CoppeliaSimInterface::set_object_as_respondable(const std::string &objectname, const bool &respondable_object)
{
    set_object_as_respondable(_get_handle_from_map(objectname), respondable_object);
}

void DQ_CoppeliaSimInterface::set_object_as_static(const int &handle, const bool &static_object) const
{
    _check_client();
    sim_->setObjectInt32Param(handle,
                              sim_->shapeintparam_static,
                              (static_object == true ? 1 : 0));
}

void DQ_CoppeliaSimInterface::set_object_as_static(const std::string &objectname, const bool &static_object)
{
    set_object_as_static(_get_handle_from_map(objectname), static_object);
}


void DQ_CoppeliaSimInterface::add_primitive(const PRIMITIVE &primitive, const std::string &name,
                                            const std::vector<double> &sizes) const
{
    if (!object_exist_on_scene(name))
    {
        _check_client();
        int shapeHandle = sim_->createPrimitiveShape(get_primitive_identifier(primitive), sizes, 0);
        set_object_name(shapeHandle, _remove_first_slash_from_string(name));
    }

}


/**
 * @brief DQ_CoppeliaSimInterface::set_object_parent
 * @param handle
 * @param parent_handle
 */
void DQ_CoppeliaSimInterface::set_object_parent(const int &handle, const int &parent_handle, const bool &move_child_to_parent_pose)
{
    _check_client();
    sim_->setObjectParent(handle, parent_handle, !move_child_to_parent_pose);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_object_parent
 * @param objectname
 * @param parent_object_name
 */
void DQ_CoppeliaSimInterface::set_object_parent(const std::string &objectname,
                                                const std::string &parent_object_name,
                                                const bool& move_child_to_parent_pose)
{
    set_object_parent(_get_handle_from_map(objectname), _get_handle_from_map(parent_object_name), move_child_to_parent_pose);
}

/**
 * @brief DQ_CoppeliaSimInterface::check_collision
 * @param handle1
 * @param handle2
 * @return
 */
bool DQ_CoppeliaSimInterface::check_collision(const int &handle1, const int &handle2) const
{
    _check_client();
    auto [result, collidingObjectHandles] = sim_->checkCollision(handle1, handle2);
    return result;
}

/**
 * @brief DQ_CoppeliaSimInterface::check_collision
 * @param objectname1
 * @param objectname2
 * @return
 */
bool DQ_CoppeliaSimInterface::check_collision(const std::string &objectname1, const std::string &objectname2)
{
    return check_collision(_get_handle_from_map(objectname1), _get_handle_from_map(objectname2));
}

/**
 * @brief DQ_CoppeliaSimInterface::check_distance
 * @param handle1
 * @param handle2
 * @param threshold
 * @return
 */
std::tuple<double, DQ, DQ> DQ_CoppeliaSimInterface::check_distance(const int &handle1, const int &handle2, const double &threshold) const
{
    _check_client();
    auto [result, data, objectHandlePair] = sim_->checkDistance(handle1, handle2, threshold);
         //[obj1X obj1Y obj1Z obj2X obj2Y obj2Z dist]
    DQ point1 = DQ(0, data.at(0), data.at(1), data.at(2));
    DQ point2 = DQ(0, data.at(3), data.at(4), data.at(5));
    double distance = data.at(6);
    return {distance, point1, point2};
}

/**
 * @brief DQ_CoppeliaSimInterface::check_distance
 * @param objectname1
 * @param objectname2
 * @param threshold
 * @return
 */
std::tuple<double, DQ, DQ> DQ_CoppeliaSimInterface::check_distance(const std::string &objectname1, const std::string &objectname2, const double &threshold)
{
    return check_distance(_get_handle_from_map(objectname1), _get_handle_from_map(objectname2), threshold);
}

/**
 * @brief DQ_CoppeliaSimInterface::compute_distance
 * @param handle1
 * @param handle2
 * @param threshold
 * @return
 */
double DQ_CoppeliaSimInterface::compute_distance(const int &handle1, const int &handle2, const double &threshold) const
{
    return std::get<0>(check_distance(handle1, handle2, threshold));
}

/**
 * @brief DQ_CoppeliaSimInterface::compute_distance
 * @param objectname1
 * @param objectname2
 * @param threshold
 * @return
 */
double DQ_CoppeliaSimInterface::compute_distance(const std::string &objectname1, const std::string &objectname2, const double &threshold)
{
    return compute_distance(_get_handle_from_map(objectname1), _get_handle_from_map(objectname2), threshold);
}


/**
 * @brief DQ_CoppeliaSimInterface::plot_plane
 * @param name
 * @param normal_to_the_plane
 * @param location
 * @param sizes
 * @param rgba_color
 * @param add_normal
 * @param normal_scale
 */
void DQ_CoppeliaSimInterface::plot_plane(const std::string &name,
                                        const DQ &normal_to_the_plane,
                                        const DQ &location,
                                        const std::vector<double> sizes,
                                        const std::vector<double> rgba_color,
                                        const bool &add_normal,
                                        const double &normal_scale)
{
    if (!is_unit(normal_to_the_plane) or !is_quaternion(normal_to_the_plane))
        _throw_runtime_error(static_cast<std::string>(std::source_location::current().function_name())
                             + ". The normal to the plane must be a unit quaternion!");
    if (!is_pure(location) or !is_quaternion(location))
        _throw_runtime_error(static_cast<std::string>(std::source_location::current().function_name())
                             + ". The location must be a pure quaternion!");


    if (!object_exist_on_scene(name))
        {
            add_primitive(PRIMITIVE::PLANE, name,
                          {sizes.at(0), sizes.at(1), sizes.at(1)});
            set_object_color(name, rgba_color);
            set_object_as_respondable(name, false);
            set_object_as_static(name, true);

            if (add_normal)
            {
                double rfc = 0.02*normal_scale;
                std::vector<double> scaled_size = {rfc*sizes.at(0),rfc* sizes.at(1), 0.2*normal_scale*sizes.at(1)};
                _create_static_axis_at_origin(name, scaled_size, AXIS::k, 1);
            }


        }
        if (!is_pure(location) and !is_quaternion(location))
            throw std::runtime_error("Location must be a pure quaternion");
        if (!is_unit(normal_to_the_plane))
            throw std::runtime_error("The normal must be a unit quaternion");

        set_object_pose(name, _get_pose_from_direction(normal_to_the_plane, location));
    }


    /**
 * @brief DQ_CoppeliaSimInterface::plot_line
 * @param name
 * @param line_direction
 * @param location
 * @param thickness_and_length
 * @param rgba_color
 * @param add_arrow
 * @param arrow_scale
 */
void DQ_CoppeliaSimInterface::plot_line(const std::string &name, const DQ &line_direction, const DQ &location, const std::vector<double> thickness_and_length, const std::vector<double> rgba_color, const bool &add_arrow, const double &arrow_scale)
{
    if (!is_unit(line_direction) or !is_quaternion(line_direction))
        _throw_runtime_error(static_cast<std::string>(std::source_location::current().function_name())
                             + ". The line direction must be a unit quaternion!");

    if (!is_pure(location) or !is_quaternion(location))
        _throw_runtime_error(static_cast<std::string>(std::source_location::current().function_name())
                             + ". The location must be a pure quaternion!");

    if (!object_exist_on_scene(name))
    {
        add_primitive(PRIMITIVE::CYLINDER, name,
                      {thickness_and_length.at(0), thickness_and_length.at(0), thickness_and_length.at(1)});
        set_object_color(name, rgba_color);
        set_object_as_respondable(name, false);
        set_object_as_static(name, true);
        if (add_arrow)
        {
            // Add the normal

            double rfc = 2*arrow_scale;
            std::vector<double> arrow_size = {rfc*thickness_and_length.at(0),rfc*thickness_and_length.at(0), 0.02*arrow_scale*thickness_and_length.at(1)};
            std::string arrow_name = _get_standard_name(name)+std::string("_normal");
            add_primitive(PRIMITIVE::CONE, arrow_name, arrow_size);
            _set_static_object_properties(arrow_name,
                                          name,
                                          1+0.5*E_*0.5*thickness_and_length.at(1)*k_,
                                          {0,0,1,1});
        }
    }
    if (!is_pure(location) and !is_quaternion(location))
        throw std::runtime_error("Location must be a pure quaternion");
    if (!is_unit(line_direction))
        throw std::runtime_error("The line direction must be a unit quaternion");

    set_object_pose(name, _get_pose_from_direction(line_direction, location));

}


/**
 * @brief DQ_CoppeliaSimInterface::plot_reference_frame
 * @param name
 * @param pose
 * @param scale
 * @param thickness_and_length
 */
void DQ_CoppeliaSimInterface::plot_reference_frame(const std::string &name,
                                                   const DQ &pose,
                                                   const double &scale,
                                                   const std::vector<double>& thickness_and_length)
{
    _check_client();

    if (!is_unit(pose))
        _throw_runtime_error(static_cast<std::string>(std::source_location::current().function_name())
                             + ". The pose must be a unit dual quaternion!");


    if (!object_exist_on_scene(name))
    {
        add_primitive(PRIMITIVE::SPHEROID, name,
                      {1.5*scale*thickness_and_length.at(0), 1.5*scale*thickness_and_length.at(0), 1.5*scale*thickness_and_length.at(0)});
        set_object_color(name, {1,1,1,0.5});
        set_object_as_respondable(name, false);
        set_object_as_static(name, true);


        std::vector<double> scaled_size = {scale*thickness_and_length.at(0),scale*thickness_and_length.at(0), scale*thickness_and_length.at(1)};
        _create_static_axis_at_origin(name, scaled_size, AXIS::k, 1);
        _create_static_axis_at_origin(name, scaled_size, AXIS::i, 1);
        _create_static_axis_at_origin(name, scaled_size, AXIS::j, 1);

        std::vector<int64_t> shapehandles = sim_->getObjectsInTree(_get_handle_from_map(name),
                                                                   sim_->object_shape_type,
                                                                   0);
        std::reverse(shapehandles.begin(), shapehandles.end());
        sim_->groupShapes(shapehandles, false);


    }
    set_object_pose(name, pose);
}

/**
 * @brief DQ_CoppeliaSimInterface::draw_trajectory
 * @param point
 * @param size
 * @param color
 * @param max_item_count
 */
void DQ_CoppeliaSimInterface::draw_permanent_trajectory(const DQ &point, const double &size, const std::vector<double> &color, const int &max_item_count)
{
    _check_client();
    if (!is_pure(point) or !is_quaternion(point))
        _throw_runtime_error(static_cast<std::string>(std::source_location::current().function_name())
                             + ". The point must be a pure quaternion.");
    VectorXd vpoint = point.vec3();
    std::vector<double> itemdata = {0,0,0,vpoint(0), vpoint(1), vpoint(2)};
    auto drawn_handle = sim_->addDrawingObject(sim_->drawing_lines+sim_->drawing_cyclic,size,0,-1,max_item_count, color);
    sim_->addDrawingObjectItem(
        drawn_handle,
        itemdata
        );
}

/**
 * @brief DQ_CoppeliaSimInterface::add_simulation_lua_script
 * @param script_name
 * @param script_code
 * @return
 */
int DQ_CoppeliaSimInterface::add_simulation_lua_script(const std::string &script_name, const std::string& script_code)
{
    _check_client();
    int scriptHandle = sim_->createScript(sim_->scripttype_simulation,
                                          script_code, 0, "lua");
    set_object_name(scriptHandle, _remove_first_slash_from_string(script_name));
    return scriptHandle;
}


/**
 * @brief DQ_CoppeliaSimInterface::draw_trajectory
 * @param objectname
 * @param size
 * @param color
 * @param max_item_count
 */
void DQ_CoppeliaSimInterface::draw_trajectory(const std::string &objectname,
                                              const double &size,
                                              const std::vector<double> &color,
                                              const int &max_item_count)
{
    if (!object_exist_on_scene(objectname+"/drawer"))
    {
        int r = color.at(0);
        int g = color.at(1);
        int b = color.at(2);

        std::string setting_str = "  dr=sim.addDrawingObject(sim.drawing_lines|sim.drawing_cyclic,"+std::to_string(size)+",0,-1,"+std::to_string(max_item_count)+",{"+std::to_string(r)+","+std::to_string(g)+","+std::to_string(b)+"})" + "\n";

        std::string code =
            "function sysCall_init()" + std::string("\n") +
            "  h=sim.getObjectHandle(sim.handle_self)" + "\n" +
            setting_str +
            "  pt=sim.getObjectPosition(h,-1) " + "\n" +
            "end" + "\n" +
            "function sysCall_sensing()" + "\n" +
            "  local l={pt[1],pt[2],pt[3]} " + "\n" +
            "  pt=sim.getObjectPosition(h,-1)" + "\n" +
            "   l[4]=pt[1]" + "\n" +
            "   l[5]=pt[2]" + "\n" +
            "   l[6]=pt[3]" + "\n" +
            "   sim.addDrawingObjectItem(dr,l)" + "\n" +
            "end                           ";
        add_simulation_lua_script("/drawer", code);
        set_object_parent("/drawer", objectname);
    }
}


/**
 * @brief DQ_CoppeliaSimInterface::get_mass
 * @param handle
 * @return
 */
double DQ_CoppeliaSimInterface::get_mass(const int &handle) const
{
   _check_client();
   return sim_->getShapeMass(handle);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_mass
 * @param object_name
 * @return
 */
double DQ_CoppeliaSimInterface::get_mass(const std::string &object_name)
{
    return get_mass(_get_handle_from_map(object_name));
}

/**
 * @brief DQ_CoppeliaSimInterface::get_center_of_mass
 * @param handle
 * @param reference_frame
 * @return
 */
DQ DQ_CoppeliaSimInterface::get_center_of_mass(const int &handle, const REFERENCE &reference_frame) const
{
    DQ COM_body_frame;
    MatrixXd Inertia_maxtrix_body_frame;
    std::tie(COM_body_frame, Inertia_maxtrix_body_frame) =_get_center_of_mass_and_inertia_matrix(handle);
    if (reference_frame == REFERENCE::BODY_FRAME)
        return COM_body_frame;
    else
    {
        DQ x_0_bodyFrame = get_object_pose(handle);
        DQ x_bodyFrame_com = 1 + 0.5*E_*COM_body_frame;
        return (x_0_bodyFrame*x_bodyFrame_com).translation();
    }
}

/**
 * @brief DQ_CoppeliaSimInterface::get_center_of_mass
 * @param object_name
 * @param reference_frame
 * @return
 */
DQ DQ_CoppeliaSimInterface::get_center_of_mass(const std::string &object_name, const REFERENCE &reference_frame)
{
    return get_center_of_mass(_get_handle_from_map(object_name), reference_frame);
}

/**
 * @brief DQ_CoppeliaSimInterface::get_inertia_matrix
 * @param handle
 * @param reference_frame
 * @return
 */
MatrixXd DQ_CoppeliaSimInterface::get_inertia_matrix(const int &handle, const REFERENCE &reference_frame)
{
    DQ COM_body_frame;
    MatrixXd Inertia_maxtrix_body_frame;
    std::tie(COM_body_frame, Inertia_maxtrix_body_frame) =_get_center_of_mass_and_inertia_matrix(handle);
    if (reference_frame == REFERENCE::BODY_FRAME)
        return Inertia_maxtrix_body_frame;
    else
    {
        DQ x_0_bodyFrame = get_object_pose(handle);
        DQ x_bodyFrame_com = 1 + 0.5*E_*COM_body_frame;
        DQ x_0_com = x_0_bodyFrame*x_bodyFrame_com;
        MatrixXd R_0_COM = _get_rotation_matrix(x_0_com.P());
        return R_0_COM*Inertia_maxtrix_body_frame*R_0_COM.transpose();
    }
}

/**
 * @brief DQ_CoppeliaSimInterface::get_inertia_matrix
 * @param link_name
 * @param reference_frame
 * @return
 */
MatrixXd DQ_CoppeliaSimInterface::get_inertia_matrix(const std::string &link_name, const REFERENCE &reference_frame)
{
    return get_inertia_matrix(_get_handle_from_map(link_name), reference_frame);
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
    handles_map_.try_emplace(objectname, handle);
}


/**
 * @brief DQ_CoppeliaSimInterface::_get_handle_from_map searchs a handle in the map.
 *
 * @param objectname
 * @return a tuple <bool, int>. If the handle is found in the map, returns <true, handle>.
 *                              Otherwise, returns <false, 0>
 */
int DQ_CoppeliaSimInterface::_get_handle_from_map(const std::string &objectname)
{
    auto search = handles_map_.find(objectname);
    if (search != handles_map_.end())
    { // handle found in map
        return search->second;
    }
    else
    {   // handle not found in map. Therefore is taken from CoppeliaSim and the map
        // is updated;
        return get_object_handle(objectname);
    }
}



//---------------Deprecated methods-----------------------------
void DQ_CoppeliaSimInterface::disconnect(){}
void DQ_CoppeliaSimInterface::disconnect_all(){}
void DQ_CoppeliaSimInterface::set_synchronous(const bool &flag){set_stepping_mode(flag);}
int DQ_CoppeliaSimInterface::wait_for_simulation_step_to_end(){return 0;}


//---------------Private methods-----------------------------
std::string DQ_CoppeliaSimInterface::_remove_first_slash_from_string(const std::string &str) const
{
    std::string new_str = str;
    size_t found = str.find('/');
    if (found != std::string::npos)
    {
        if(found == 0) // The string containt the '/'
        {
            new_str.erase(0,1); // remove the '/'
        }
    }
    return new_str;
}

bool DQ_CoppeliaSimInterface::_start_with_slash(const std::string &str) const
{
    /*
    size_t found = str.find('/');
    if(found == 0) // The string containt the '/'
        return true;
    else
        return false;
    */
    return str.starts_with("/");

}

/**
 * @brief DQ_CoppeliaSimInterface::_get_standard_string returns a string that
 *        always start with "/"
 * @param str
 * @return
 */
std::string DQ_CoppeliaSimInterface::_get_standard_name(const std::string &str) const
{
    std::string standard_str = str;
    if (!_start_with_slash(str) && enable_deprecated_name_compatibility_ == true)
        standard_str = std::string("/")+str;
    return standard_str;
}


/**
 * @brief DQ_CoppeliaSimInterface::_get_velocity_const_params
 * @return
 */
std::vector<int> DQ_CoppeliaSimInterface::_get_velocity_const_params() const
{
    std::vector<int> params = {sim_->shapefloatparam_init_velocity_a,
        sim_->shapefloatparam_init_velocity_b,
        sim_->shapefloatparam_init_velocity_g,
        sim_->shapefloatparam_init_velocity_x,
        sim_->shapefloatparam_init_velocity_y,
        sim_->shapefloatparam_init_velocity_z
    };
    return params;
}

/**
 * @brief DQ_CoppeliaSimInterface::_load_model
 * @param path_to_filename
 * @param desired_model_name
 * @return
 */
bool DQ_CoppeliaSimInterface::_load_model(const std::string &path_to_filename,
                                          const std::string &desired_model_name,
                                          const bool &remove_child_script)
{
    int rtn = sim_->loadModel(path_to_filename);
    if (rtn != -1)
    {
        set_object_name(rtn, _remove_first_slash_from_string(desired_model_name));
        if (remove_child_script)
        {
            remove_child_script_from_object(std::string("/")
                                        + _remove_first_slash_from_string(desired_model_name));
        }
        return true;
    }else{
        return false;
    }
}

/**
 * @brief DQ_CoppeliaSimInterface::get_transformation_matrix
 * @param coeff_vector
 * @return
 */
MatrixXd DQ_CoppeliaSimInterface::_get_transformation_matrix(const std::vector<double> &coeff_vector) const
{
    std::vector<double> coeff = coeff_vector;
    MatrixXd TM = Map<VectorXd>(coeff.data(),
                                coeff.size()).reshaped(4,3).transpose();
    return TM;
}

/**
 * @brief This function returns a rotation matrix given a rotation unit quaternion.
 * @param r unit quaternion.
 * @returns The rotation matrix.
 */
MatrixXd DQ_CoppeliaSimInterface::_get_rotation_matrix(const DQ& r) const{
    Matrix<double, 3, 3> R;
    VectorXd vecr = r.vec4();
    double w = vecr(0);
    double a = vecr(1);
    double b = vecr(2);
    double c = vecr(3);
    R << 1-2*(b*b +c*c), 2*(a*b-w*c), 2*(a*c+w*b),
        2*(a*b+w*c), 1-2*(a*a+c*c),  2*(b*c-w*a),
        2*(a*c-w*b), 2*(b*c+w*a),   1-2*(a*a+b*b);

    return R;
}

/**
 * @brief DQ_CoppeliaSimInterface::_get_rotation_from_direction
 * @param direction
 * @return
 */
DQ DQ_CoppeliaSimInterface::_get_pose_from_direction(const DQ& direction, const DQ& point)
{
    DQ base_direction = k_;
    DQ nx;
    double argument = std::round(static_cast<double>(dot(base_direction,direction.P()))*100000)/100000;
    double phi = acos(argument);
    if (phi == 0)
        nx = base_direction;
    else
        nx = cross(base_direction, direction);
    DQ r = cos(phi/2) + nx.normalize()*sin(phi/2);
    return r + 0.5*E_*point*r;
}

void DQ_CoppeliaSimInterface::_create_static_axis_at_origin(const std::string& parent_name,
                                                            const std::vector<double> &sizes,
                                                            const AXIS &axis,
                                                            const double &alpha_color)
{
    std::string name;
    DQ dqaxis;
    std::vector<double> color;
    DQ rotation;
    const double angle = std::numbers::pi/2;
    switch(axis)
    {
    case AXIS::i:
        name = _get_standard_name(parent_name)+std::string("_x");
        dqaxis = i_;
        color = {1,0,0,alpha_color};
        rotation = cos(angle/2) + j_*sin(angle/2);
        break;
    case AXIS::j:
        name = _get_standard_name(parent_name)+std::string("_y");
        dqaxis = j_;
        color = {0,1,0,alpha_color};
        rotation = cos(-angle/2) + i_*sin(-angle/2);
        break;
    case AXIS::k:
        name = _get_standard_name(parent_name)+std::string("_z");
        dqaxis = k_;
        color = {0,0,1,alpha_color};
        rotation = DQ(1);
        break;
    }
    //double rfc = 0.02*scale;
    //std::vector<double> scaled_size = {rfc*sizes.at(0),rfc* sizes.at(1), 0.2*scale*sizes.at(1)};

    add_primitive(PRIMITIVE::CYLINDER,
                  name,
                  sizes);
    _set_static_object_properties(name,
                                  parent_name,
                                  rotation+0.5*E_*0.5*sizes.at(2)*dqaxis*rotation,
                                  color
                                  );
    std::vector<double> arrow_size = {2*sizes.at(0), 2*sizes.at(1), 2*sizes.at(1)};
    std::string arrow_name = _get_standard_name(name)+std::string("_");
    add_primitive(PRIMITIVE::CONE, arrow_name, arrow_size);

    _set_static_object_properties(arrow_name,
                                  parent_name,
                                  rotation+0.5*E_*sizes.at(2)*dqaxis*rotation,
                                  color);
}

void DQ_CoppeliaSimInterface::_set_static_object_properties(const std::string &name,
                                                            const std::string &parent_name,
                                                            const DQ &pose, const std::vector<double> &rgba_color)
{
    set_object_color(name, rgba_color);
    set_object_as_respondable(name, false);
    set_object_as_static(name, true);
    set_object_pose(name, pose);
    set_object_parent(name, parent_name, false);
}

int DQ_CoppeliaSimInterface::get_primitive_identifier(const PRIMITIVE &primitive) const
{
    switch (primitive)
    {
    case PRIMITIVE::PLANE:
        return sim_->primitiveshape_plane;
    case PRIMITIVE::DISC:
        return sim_->primitiveshape_disc;
    case PRIMITIVE::CUBOID:
        return sim_->primitiveshape_cuboid;
    case PRIMITIVE::SPHEROID:
        return sim_->primitiveshape_spheroid;
    case PRIMITIVE::CYLINDER:
        return sim_->primitiveshape_cylinder;
    case PRIMITIVE::CONE:
        return sim_->primitiveshape_cone;
    case PRIMITIVE::CAPSULE:
        return sim_->primitiveshape_capsule;

    }
}

void DQ_CoppeliaSimInterface::_check_client() const
{
    if (!client_created_)
        throw std::runtime_error("Unestablished connection. Did you use connect()?");
}

void DQ_CoppeliaSimInterface::_throw_runtime_error(const std::string &msg)
{
    stop_simulation();
    std::cerr<<"Something went wrong. I stopped the simulation!"<<std::endl;
    throw std::runtime_error(msg);
}


/**
 * @brief DQ_CoppeliaSimInterface::get_center_of_mass_and_inertia_matrix
 * @param handle
 * @return
 */
std::tuple<DQ, MatrixXd> DQ_CoppeliaSimInterface::_get_center_of_mass_and_inertia_matrix(const int &handle) const
{
    std::vector<double> inertia_matrix_coeff;
    std::vector<double> center_of_mass_coeff;
    std::tie(inertia_matrix_coeff, center_of_mass_coeff) = sim_->getShapeInertia(handle);

    MatrixXd Inertia_maxtrix_body_frame = Map<VectorXd>(inertia_matrix_coeff.data(),
                                                        inertia_matrix_coeff.size()).reshaped(3,3);
    MatrixXd COM_body_frame_matrix =  _get_transformation_matrix(center_of_mass_coeff);
    DQ COM_body_frame = DQ(COM_body_frame_matrix.col(3));
    double mass = get_mass(handle);

    return {COM_body_frame, Inertia_maxtrix_body_frame/mass};
}


//--------------------------------------------------------------

