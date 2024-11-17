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
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimZmqInterface.h>
#include <algorithm>
#include <RemoteAPIClient.h>


//-------------------Private components-----------------------//
std::unique_ptr<RemoteAPIClient> client_;
std::unique_ptr<RemoteAPIObject::sim> sim_;

bool _create_client(const std::string& host = "localhost",
                    const int& rpcPort = 23000,
                    const int& cntPort = -1,
                    const int& verbose_ = -1,
                    const bool& client_flag = false);
void _set_status_bar_message(const std::string &message, const int& verbosity_type);

//------------------------------------------------------------//

/**
 * @brief _set_status_bar_message
 * @param message
 * @param verbosity_type
 */
void _set_status_bar_message(const std::string &message, const int& verbosity_type)
{
    sim_->addLog(verbosity_type, message);
}

/**
 * @brief _create_client
 * @param host
 * @param rpcPort
 * @param cntPort
 * @param verbose_
 * @param client_flag
 * @return
 */
bool _create_client(const std::string& host,
                    const int& rpcPort,
                    const int& cntPort,
                    const int& verbose_,
                    const bool& client_flag)
{
    if (!client_flag)
    {
        client_ = std::make_unique<RemoteAPIClient>(host, rpcPort, cntPort, verbose_);
        sim_    = std::make_unique<RemoteAPIObject::sim >(client_->getObject().sim());
    }
    return true;
}
//-------------------------------------------------------------//


/**
 * @brief DQ_CoppeliaSimZmqInterface::DQ_CoppeliaSimZmqInterface
 */
DQ_CoppeliaSimZmqInterface::DQ_CoppeliaSimZmqInterface() //:exp_{std::shared_ptr<DQ_CoppeliaSimZmqInterface>(this)}
    :client_created_{false}
{

}

/**
 * @brief DQ_CoppeliaSimZmqInterface::~DQ_CoppeliaSimZmqInterface
 */
DQ_CoppeliaSimZmqInterface::~DQ_CoppeliaSimZmqInterface()
{
    _join_if_joinable_chronometer_thread();
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::_join_if_joinable_chronometer_thread
 */
void DQ_CoppeliaSimZmqInterface::_join_if_joinable_chronometer_thread()
{
    if (chronometer_thread_.joinable())
        chronometer_thread_.join();
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::_start_chronometer
 */
void DQ_CoppeliaSimZmqInterface::_start_chronometer()
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
 * @brief DQ_CoppeliaSimZmqInterface::_check_connection
 */
void DQ_CoppeliaSimZmqInterface::_check_connection()
{
    if (!client_created_)
    {
        //For C++20
        //std::cerr<<std::format("Unestablished connection at \"{}\" in port {}", host_, rpcPort_ )<<std::endl;
        //std::cerr<<std::format("You used a timeout of {}ms. Is enough time for your system?", MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_)<<std::endl;
        std::cerr<<"Unestablished connection at \""+ host_ +"\" in port "<<rpcPort_<<std::endl;
        std::cerr<<"You used a timeout of "<<MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_<<"ms. Is enough time for your system?"<<std::endl;
        if(rpcPort_ != 23000)
        {
            std::cerr<<""<<std::endl;
            //For C++20
            //std::cerr<<std::format("is CoppeliaSim running with the port {} enabled?", rpcPort_)<<std::endl;
            std::cerr<<"is CoppeliaSim running with the port "<<rpcPort_<<" enabled?"<<std::endl;
            std::cerr<<""<<std::endl;
            std::cerr<<"Example: using the terminal, open CoppeliaSim with arguments:"<<std::endl;
            std::cerr<<"----------------------------------------"<<std::endl;
            //For C++20
            //std::cerr<<std::format("coppeliasim -GzmqRemoteApi.rpcPort={}", rpcPort_)<<std::endl;
            std::cerr<<"coppeliasim -GzmqRemoteApi.rpcPort="<<rpcPort_<<std::endl;
            std::cerr<<"----------------------------------------"<<std::endl;
            if (rpcPort_ < 23000)
            {
                if (rpcPort_ == 19997)
                    std::cerr<<"The port "<<rpcPort_<<" is commonly used in the legacy API. However it is not compatible with the ZMQ Remote API."<<std::endl;
                std::cerr<<"The ZMQ Remote API uses the port "<<23000<<" by default."<<std::endl;
            }
        }
        std::cerr<<""<<std::endl;
        throw std::runtime_error("Unestablished connection.");
    }
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::connect establish a connection between the client (your code) and
 *                                         the host (the computer running the CoppeliaSim scene).
 * @param host    eg. 'localhost' if the host is running in the same
 *                machine in which is running the client.
 * @param rpcPort The port to establish a connection. (e.g. 23000, 23001, 23002, 23003...).
 * @param MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION The timeout to establish the connection.
 * @param cntPort
 * @param verbose
 * @return
 */
bool DQ_CoppeliaSimZmqInterface::connect(const std::string &host, const int &rpcPort, const int &MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION, const int &cntPort, const int &verbose)
{
    try
    {
        host_ = host;
        rpcPort_ = rpcPort;
        cntPort_ = cntPort;
        verbose_ = verbose;
        MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_ = MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION;

        _join_if_joinable_chronometer_thread();
        chronometer_thread_ = std::thread(&DQ_CoppeliaSimZmqInterface::_start_chronometer, this);


        client_created_ = _create_client(host, rpcPort, cntPort, verbose, client_created_);

        _join_if_joinable_chronometer_thread();
        set_status_bar_message("       ");
        _set_status_bar_message("DQ Robotics established a connection on port " + std::to_string(rpcPort),
                                sim_->verbosity_warnings);
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Runtime error in DQ_CoppeliaSimZmqInterface::connect. "
                  << e.what() << std::endl;
    }
    return client_created_;
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::connect establish a connection between the client (your code) and
 *             the host (the computer running the CoppeliaSim scene).
 *             Calling this method is required before anything else can happen.
 * @param host The IP address of the computer that hosts the CoppeliaSim simulation. If the client (your code)
 *             and the simulation are running in the same computer, you can use "localhost".
 * @param port The port to establish a connection. (e.g. 23000, 23001, 23002, 23003...).
 * @param TIMEOUT_IN_MILISECONDS The timeout to establish the connection.
 * @return true if the connection is established. False otherwise.
 */
bool DQ_CoppeliaSimZmqInterface::connect(const std::string &host, const int &port, const int &TIMEOUT_IN_MILISECONDS)
{
    return connect(host, port, TIMEOUT_IN_MILISECONDS, -1, -1);
}


bool DQ_CoppeliaSimZmqInterface::connect(const int &port, const int &TIMEOUT_IN_MILISECONDS, const int &MAX_TRY_COUNT)
{
    int auxport = port;
    if (auxport == 19997)
    {
        auxport = 23000;
        std::cerr<<"The port "<<port<<" is commonly used in the legacy API. However it is not compatible with the ZMQ Remote API."<<std::endl;
        std::cerr<<"I changed the port to "<<23000<<std::endl;
    }

    return connect("localhost", auxport, TIMEOUT_IN_MILISECONDS);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::_map_simulation_state
 * @param state
 * @return
 */
std::string DQ_CoppeliaSimZmqInterface::_map_simulation_state(const int &state)
{
    return simulation_status_[state];
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::start_simulation starts the CoppeliaSim simulation.
 */
void DQ_CoppeliaSimZmqInterface::start_simulation() const
{
    _check_client();
    sim_->startSimulation();
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::pause_simulation pauses the CoppeliaSim simulation.
 */
void DQ_CoppeliaSimZmqInterface::pause_simulation() const
{
    _check_client();
    sim_->pauseSimulation();

}


/**
 * @brief DQ_CoppeliaSimZmqInterface::stop_simulation stops the  CoppeliaSim simulation.
 */
void DQ_CoppeliaSimZmqInterface::stop_simulation() const
{
    _check_client();
    sim_->stopSimulation();
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::set_stepping_mode enables or disables the stepping mode
 *        (formerly known as synchronous mode).
 * @param flag. Eg: set_stepping_mode(true)  // enables the stepping mode
 *                  set_stepping_mode(false)  // disables the stepping mode
 */
void DQ_CoppeliaSimZmqInterface::set_stepping_mode(const bool &flag)
{
    _check_client();
    sim_->setStepping(flag);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_simulation_time returns the simulation time.
 *        This time does not correspond to the real-time necessarily.
 * @return The simulation time.
 */
double DQ_CoppeliaSimZmqInterface::get_simulation_time() const
{
    _check_client();
    return sim_->getSimulationTime();
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::trigger_next_simulation_step This method sends a trigger
 *        signal to the CoppeliaSim scene, which performs a simulation step when the stepping mode is used.
 */
void DQ_CoppeliaSimZmqInterface::trigger_next_simulation_step() const
{
    _check_client();
    sim_->step();
}



/**
 * @brief DQ_CoppeliaSimZmqInterface::is_simulation_running checks if the simulation is running.
 * @return True if the simulation is running. False otherwise.
 */
bool DQ_CoppeliaSimZmqInterface::is_simulation_running() const
{
    _check_client();
    return (sim_->getSimulationState() > sim_->simulation_paused);
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::get_simulation_state
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
int DQ_CoppeliaSimZmqInterface::get_simulation_state() const
{
    _check_client();
    return sim_->getSimulationState();
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_status_bar_message sends a message to CoppeliaSim to be
 *        displayed in the status bar.
 *
 * @param message
 */
void DQ_CoppeliaSimZmqInterface::set_status_bar_message(const std::string &message) const
{
    _check_client();
    _set_status_bar_message(message, sim_->verbosity_undecorated);
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::get_object_handle gets the object handle from
 *        CoppeliaSim.
 * @param objectname The name of the object in the CoppeliaSim scene.
 * @return the object handle.
 */
int DQ_CoppeliaSimZmqInterface::get_object_handle(const std::string &objectname)
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
        handle = sim_->getObject(standard_objectname);
        //If the handle is not included in the map, then the map is updated.
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
 * @brief DQ_CoppeliaSimZmqInterface::get_object_handles returns a vector containing the object handles.
 * @param objectnames The vector that contains the object names in the CoppeliaSim scene.
 * @return The desired vector of handles.
 */
std::vector<int> DQ_CoppeliaSimZmqInterface::get_object_handles(const std::vector<std::string> &objectnames)
{
    int n = objectnames.size();
    std::vector<int> handles(n);
    for(auto i=0;i<n;i++)
        handles[i]=get_object_handle(objectnames[i]);

    return handles;
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::get_object_translation returns a pure quaternion that represents the position
 *        of an object in the CoppeliaSim scene with respect to the absolute frame.
 * @param handle The handle of the object.
 * @return The position of the handle.
 */
DQ DQ_CoppeliaSimZmqInterface::get_object_translation(const int &handle) const
{
    _check_client();
    auto position = sim_->getObjectPosition(handle, sim_->handle_world);
    const DQ t = DQ(0, position.at(0),position.at(1),position.at(2));
    return t;
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::get_object_translation returns a pure quaternion that represents the position
 *        of an object in the CoppeliaSim scene with respect to the absolute frame.
 * @param objectname The name of the object.
 * @return The position of the object.
 */
DQ DQ_CoppeliaSimZmqInterface::get_object_translation(const std::string &objectname)
{
    return get_object_translation(_get_handle_from_map(objectname));
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::set_object_translation sets the translation of an object
 *        in the CoppeliaSim scene..
 * @param handle the object handle
 * @param t The pure quaternion that represents the desired position with respect to the absolute frame..
 */
void DQ_CoppeliaSimZmqInterface::set_object_translation(const int &handle, const DQ &t)
{
    VectorXd vec_t = t.vec3();
    std::vector<double> position = {vec_t[0], vec_t[1],vec_t[2]};
    _check_client();
    sim_->setObjectPosition(handle, position,sim_->handle_world);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_object_translation sets the translation of an object
 *        in the CoppeliaSim scene.
 * @param objectname the name of the object
 * @param t The pure quaternion that represents the desired position with respect to the absolute frame..
 */
void DQ_CoppeliaSimZmqInterface::set_object_translation(const std::string &objectname, const DQ &t)
{
    set_object_translation(_get_handle_from_map(objectname), t);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_object_rotation returns a unit quaternion that represents the rotation
 *        of an object in the CoppeliaSim scene with respect to the absolute frame.
 * @param handle the object handle
 * @return The object rotation.
 */
DQ DQ_CoppeliaSimZmqInterface::get_object_rotation(const int &handle) const
{
    _check_client();
    auto rotation = sim_->getObjectQuaternion(handle +
                                              sim_->handleflag_wxyzquat,
                                              sim_->handle_world);

    return DQ(rotation.at(0), rotation.at(1), rotation.at(2), rotation.at(3));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_object_rotation returns a unit quaternion that represents the rotation
 *        of an object in the CoppeliaSim scene with respect to the absolute frame.
 * @param objectname the name of the object.
 * @return The object rotation
 */
DQ DQ_CoppeliaSimZmqInterface::get_object_rotation(const std::string &objectname)
{
    return get_object_rotation(_get_handle_from_map(objectname));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_object_rotation sets the rotation of an object in the CoppeliaSim scene.
 * @param handle the object handle
 * @param r A unit quaternion that represents the desired rotation with respect to the absolute frame..
 */
void DQ_CoppeliaSimZmqInterface::set_object_rotation(const int &handle, const DQ &r)
{

    VectorXd vec_r = r.vec4();
    std::vector<double> rotation= {vec_r[0], vec_r[1],vec_r[2], vec_r[3]};
    _check_client();
    sim_->setObjectQuaternion(handle + sim_->handleflag_wxyzquat, rotation, sim_->handle_world);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_object_rotation sets the rotation of an object in the CoppeliaSim scene.
 * @param objectname the name of the object
 * @param r A unit quaternion that represents the desired rotation with respect to the absolute frame.
 */
void DQ_CoppeliaSimZmqInterface::set_object_rotation(const std::string &objectname, const DQ &r)
{
    set_object_rotation(_get_handle_from_map(objectname), r);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_object_pose returns a unit dual quaternion that represents
 *        the object pose in the CoppeliaSim scene with respect to the absolute frame..
 * @param handle The object handle
 * @return The desired object pose.
 */
DQ DQ_CoppeliaSimZmqInterface::get_object_pose(const int &handle) const
{
    DQ t = get_object_translation(handle);
    DQ r = get_object_rotation(handle);
    return r + 0.5*E_*t*r;
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_object_pose returns a unit dual quaternion that represents
 *        the object pose in the CoppeliaSim scene with respect to the absolute frame.
 * @param objectname The name of the object in the CoppeliaSim scene.
 * @return the desired object pose
 */
DQ DQ_CoppeliaSimZmqInterface::get_object_pose(const std::string &objectname)
{
    return get_object_pose(_get_handle_from_map(objectname));
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::set_object_pose sets the pose of an object in the CoppeliaSim scene.
 * @param handle the object handle
 * @param h A unit dual qualternion that represents the desired object pose with respect to the absolute frame.
 */
void DQ_CoppeliaSimZmqInterface::set_object_pose(const int &handle, const DQ &h) const
{

    VectorXd vec_r = h.P().vec4();
    VectorXd vec_p = h.translation().vec3();
    std::vector<double> pose = {vec_p[0], vec_p[1],vec_p[2],vec_r[0], vec_r[1],vec_r[2], vec_r[3]};
    _check_client();
    sim_->setObjectPose(handle + sim_->handleflag_wxyzquat, pose, sim_->handle_world);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_object_pose sets the pose of an object in the CoppeliaSim scene.
 * @param objectname The name of the object.
 * @param h A unit dual qualternion that represents the desired object pose with respect to the absolute frame.
 */
void DQ_CoppeliaSimZmqInterface::set_object_pose(const std::string &objectname, const DQ &h)
{
    // For C++20
    // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
    std::string function_name = {"DQ_CoppeliaSimInterface::set_object_pose"};
    if (!is_unit(h))
        _throw_runtime_error(function_name + ". The pose must be a unit dual quaternion!");
    int handle = _get_handle_from_map(objectname);
    set_object_pose(handle, h);
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::get_joint_position gets the joint position in the CoppeliaSim scene
 * @param handle The joint handle
 * @return The joint position
 */
double DQ_CoppeliaSimZmqInterface::get_joint_position(const int &handle) const
{
    _check_client();
    return double(sim_->getJointPosition(handle));
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::get_joint_position gets the joint position in the CoppeliaSim scene
 * @param jointname the joint name
 * @return The joint position
 */
double DQ_CoppeliaSimZmqInterface::get_joint_position(const std::string &jointname)
{
    return get_joint_position(_get_handle_from_map(jointname));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_joint_positions gets the joint positions in the CoppeliaSim scene
 * @param handles A vector containing the handles of the joints.
 * @return The joint positions
 */
VectorXd DQ_CoppeliaSimZmqInterface::get_joint_positions(const std::vector<int> &handles) const
{
    int n = handles.size();
    VectorXd joint_positions(n);
    for(auto i=0;i<n;i++)
        joint_positions(i)=get_joint_position(handles.at(i));

    return joint_positions;
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_joint_positions gets the joint positions in the CoppeliaSim scene.
 * @param jointnames A vector containing the names of the joints.
 * @return The joint positions
 */
VectorXd DQ_CoppeliaSimZmqInterface::get_joint_positions(const std::vector<std::string> &jointnames)
{
    int n = jointnames.size();
    VectorXd joint_positions(n);
    for(auto i=0;i<n;i++)
        joint_positions(i)=get_joint_position(jointnames[i]);

    return joint_positions;
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_position sets the joint position in the CoppeliaSim scene
 * @param handle The joint handle
 * @param angle_rad The desired joint position
 */
void DQ_CoppeliaSimZmqInterface::set_joint_position(const int &handle, const double &angle_rad) const
{
    _check_client();
    sim_->setJointPosition(handle, angle_rad);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_position sets the joint position in the CoppeliaSim scene
 * @param jointname The joint name
 * @param angle_rad The desired joint position
 */
void DQ_CoppeliaSimZmqInterface::set_joint_position(const std::string &jointname, const double &angle_rad)
{
    set_joint_position(_get_handle_from_map(jointname), angle_rad);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_positions sets the joint positions in the CoppeliaSim scene
 * @param handles A vector containing the joint handles
 * @param angles_rad The desired joint positions
 */
void DQ_CoppeliaSimZmqInterface::set_joint_positions(const std::vector<int> &handles, const VectorXd &angles_rad) const
{
    for(std::size_t i=0;i<handles.size();i++)
        set_joint_position(handles.at(i), angles_rad(i));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_positions sets the joint positions in the CoppeliaSim scene
 * @param jointnames A vector containing the joint names.
 * @param angles_rad The desired joint positions.
 */
void DQ_CoppeliaSimZmqInterface::set_joint_positions(const std::vector<std::string> &jointnames, const VectorXd &angles_rad)
{
     _check_sizes(jointnames, angles_rad, "Error in DQ_CoppeliaSimInterface::set_joint_positions: "
                                          "jointnames and angles_rad have incompatible sizes");
    for(std::size_t i=0;i<jointnames.size();i++)
         set_joint_position(jointnames.at(i), angles_rad(i));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_target_position sets the joint target position
 * @param handle
 * @param angle_rad
 */
void DQ_CoppeliaSimZmqInterface::set_joint_target_position(const int &handle, const double &angle_rad) const
{
    _check_client();
    sim_->setJointTargetPosition(handle, angle_rad);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_target_position
 * @param jointname
 * @param angle_rad
 */
void DQ_CoppeliaSimZmqInterface::set_joint_target_position(const std::string &jointname, const double &angle_rad)
{
    set_joint_target_position(_get_handle_from_map(jointname), angle_rad);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_target_positions
 * @param handles
 * @param angles_rad
 */
void DQ_CoppeliaSimZmqInterface::set_joint_target_positions(const std::vector<int> &handles, const VectorXd &angles_rad) const
{
    for(std::size_t i=0;i<handles.size();i++)
        set_joint_target_position(handles.at(i), angles_rad(i));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_target_positions
 * @param jointnames
 * @param angles_rad
 */
void DQ_CoppeliaSimZmqInterface::set_joint_target_positions(const std::vector<std::string> &jointnames, const VectorXd &angles_rad)
{
    _check_sizes(jointnames, angles_rad, "Error in DQ_CoppeliaSimInterface::set_joint_target_positions: "
                                         "jointnames and angles_rad have incompatible sizes");
    for(std::size_t i=0;i<jointnames.size();i++)
        set_joint_target_position(jointnames.at(i), angles_rad(i));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_joint_velocity
 * @param handle
 * @return
 */
double DQ_CoppeliaSimZmqInterface::get_joint_velocity(const int &handle) const
{
    _check_client();
    return sim_->getObjectFloatParam(handle, sim_->jointfloatparam_velocity);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_joint_velocity
 * @param jointname
 * @return
 */
double DQ_CoppeliaSimZmqInterface::get_joint_velocity(const std::string &jointname)
{
    return get_joint_velocity(_get_handle_from_map(jointname));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_joint_velocties
 * @param handles
 * @return
 */
VectorXd DQ_CoppeliaSimZmqInterface::get_joint_velocities(const std::vector<int> &handles) const
{
    int n = handles.size();
    VectorXd joint_velocities(n);
    for(auto i=0;i<n;i++)
        joint_velocities(i)=get_joint_velocity(handles.at(i));

    return joint_velocities;
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_joint_velocities
 * @param jointnames
 * @return
 */
VectorXd DQ_CoppeliaSimZmqInterface::get_joint_velocities(const std::vector<std::string> &jointnames)
{
    int n = jointnames.size();
    VectorXd joint_velocities(n);
    for(auto i=0;i<n;i++)
        joint_velocities(i)=get_joint_velocity(jointnames[i]);

    return joint_velocities;
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_target_velocity
 * @param handle
 * @param angle_rad_dot
 */
void DQ_CoppeliaSimZmqInterface::set_joint_target_velocity(const int &handle, const double &angle_rad_dot) const
{
    _check_client();
    sim_->setJointTargetVelocity(handle, angle_rad_dot);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_target_velocity
 * @param jointname
 * @param angle_rad_dot
 */
void DQ_CoppeliaSimZmqInterface::set_joint_target_velocity(const std::string &jointname, const double &angle_rad_dot)
{
    set_joint_target_velocity(_get_handle_from_map(jointname), angle_rad_dot);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_target_velocities
 * @param handles
 * @param angles_rad_dot
 */
void DQ_CoppeliaSimZmqInterface::set_joint_target_velocities(const std::vector<int> &handles, const VectorXd &angles_rad_dot) const
{
    for(std::size_t i=0;i<handles.size();i++)
        set_joint_target_velocity(handles.at(i), angles_rad_dot(i));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_target_velocities
 * @param jointnames
 * @param angles_rad_dot
 */
void DQ_CoppeliaSimZmqInterface::set_joint_target_velocities(const std::vector<std::string> &jointnames, const VectorXd &angles_rad_dot)
{
    _check_sizes(jointnames, angles_rad_dot, "Error in DQ_CoppeliaSimInterface::set_joint_target_velocities: "
                                             "jointnames and angles_rad_Dot have incompatible sizes");
    for(std::size_t i=0;i<jointnames.size();i++)
        set_joint_target_velocity(jointnames.at(i), angles_rad_dot(i));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_torque
 * @param handle
 * @param torque
 */
void DQ_CoppeliaSimZmqInterface::set_joint_torque(const int &handle, const double &torque) const
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
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_torque
 * @param jointname
 * @param torque
 */
void DQ_CoppeliaSimZmqInterface::set_joint_torque(const std::string &jointname, const double &torque)
{
    set_joint_torque(_get_handle_from_map(jointname), torque);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_torques
 * @param handles
 * @param torques
 */
void DQ_CoppeliaSimZmqInterface::set_joint_torques(const std::vector<int> &handles, const VectorXd &torques) const
{
    for(std::size_t i=0;i<handles.size();i++)
        set_joint_torque(handles.at(i), torques(i));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_torques
 * @param jointnames
 * @param torques
 */
void DQ_CoppeliaSimZmqInterface::set_joint_torques(const std::vector<std::string> &jointnames, const VectorXd &torques)
{
    _check_sizes(jointnames, torques, "Error in DQ_CoppeliaSimInterface::set_joint_torques: "
                                             "jointnames and torques have incompatible sizes");
    for(std::size_t i=0;i<jointnames.size();i++)
        set_joint_torque(jointnames.at(i), torques(i));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_joint_torque
 * @param handle
 * @return
 */
double DQ_CoppeliaSimZmqInterface::get_joint_torque(const int &handle) const
{
    _check_client();
    return sim_->getJointForce(handle);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_joint_torque
 * @param jointname
 * @return
 */
double DQ_CoppeliaSimZmqInterface::get_joint_torque(const std::string &jointname)
{
    return get_joint_torque(_get_handle_from_map(jointname));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_joint_torques
 * @param handles
 * @return
 */
VectorXd DQ_CoppeliaSimZmqInterface::get_joint_torques(const std::vector<int> &handles) const
{
    int n = handles.size();
    VectorXd joint_torques(n);
    for(auto i=0;i<n;i++)
        joint_torques(i)=get_joint_torque(handles.at(i));

    return joint_torques;
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_joint_torques
 * @param jointnames
 * @return
 */
VectorXd DQ_CoppeliaSimZmqInterface::get_joint_torques(const std::vector<std::string> &jointnames)
{
    int n = jointnames.size();
    VectorXd joint_torques(n);
    for(auto i=0;i<n;i++)
        joint_torques(i)=get_joint_torque(jointnames[i]);

    return joint_torques;
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_object_name
 * @param handle
 * @return
 */
std::string DQ_CoppeliaSimZmqInterface::get_object_name(const int &handle)
{
    _check_client();
    std::string objectname = sim_->getObjectAlias(handle, 1);
    _update_map(objectname, handle);
    return objectname;
}




/*
 * ---------------------------------------------------------------------------------------
for C++20
std::vector<std::string> DQ_CoppeliaSimInterface::get_object_names(const auto &handles)
{
    int n = handles.size();
    std::vector<std::string> objectnames(n);
    for(auto i=0;i<n;i++)
        objectnames.at(i)=get_object_name(handles.at(i));

    return objectnames;
}
*/
/**
 * @brief DQ_CoppeliaSimZmqInterface::get_object_names
 * @param handles
 * @return
 */
template<typename T>
std::vector<std::string> DQ_CoppeliaSimZmqInterface::get_object_names(const T &handles)
{
    int n = handles.size();
    std::vector<std::string> objectnames(n);
    for(auto i=0;i<n;i++)
        objectnames.at(i)=get_object_name(handles.at(i));

    return objectnames;
}
//-----------------------------------------------------------------------------------------

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_jointnames_from_parent_object returns a vector containing all the joint names
 *                                  in which a specified object is its parent.
 * @param parent_objectname The name of the object on CoppeliaSim that is the parent of the desired joints.
 * @return a vector containing the desired joint names
 */
std::vector<std::string> DQ_CoppeliaSimZmqInterface::get_jointnames_from_parent_object(const std::string &parent_objectname)
{
    int base_handle = _get_handle_from_map(parent_objectname);
    _check_client();
    std::vector<int64_t> jointhandles = sim_->getObjectsInTree(base_handle,
                                        sim_->object_joint_type,
                                        0);
    return get_object_names(jointhandles);

}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_shapenames_from_parent_object returns a vector containing all the shape names
 *                                  in which a specified object is its parent.
 * @param parent_objectname The name of the object on CoppeliaSim that is the parent of the desired shapes.
 * @param shape_type
 * @return a vector containing the desired shape names
 */
std::vector<std::string> DQ_CoppeliaSimZmqInterface::get_shapenames_from_parent_object(const std::string &parent_objectname,
                                                                                    const SHAPE_TYPE &shape_type)
{
    int base_handle = _get_handle_from_map(parent_objectname);
    _check_client();
    std::vector<int64_t> shapehandles = sim_->getObjectsInTree(base_handle,
                                                               sim_->object_shape_type,
                                                               0);
    size_t handlesizes = shapehandles.size();
    std::vector<int64_t> dynamic_features(handlesizes, 0);

    for (size_t i=0; i<handlesizes;i++) // 0 is dynamic.  1 is static
        dynamic_features.at(i) = sim_->getObjectInt32Param(shapehandles.at(i), sim_->shapeintparam_static);

    std::vector<int64_t> aux_shapehandles;
    switch (shape_type) {
    case SHAPE_TYPE::DYNAMIC:
        for (size_t i=0; i<handlesizes; i++)
            if (dynamic_features.at(i) == 0)
                aux_shapehandles.push_back(shapehandles.at(i));
        return get_object_names(aux_shapehandles);
    case SHAPE_TYPE::STATIC:
        for (size_t i=0; i<handlesizes; i++)
            if (dynamic_features.at(i) == 1)
                aux_shapehandles.push_back(shapehandles.at(i));
        return get_object_names(aux_shapehandles);
    case SHAPE_TYPE::ANY:
        return get_object_names(shapehandles);
    default: // This line is required in GNU/Linux
        _throw_runtime_error("wrong argument!");
    }


}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_angular_and_linear_velocities
 * @param handle
 * @param reference
 * @return
 */
VectorXd DQ_CoppeliaSimZmqInterface::get_angular_and_linear_velocities(const int &handle, const REFERENCE &reference) const
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
 * @brief DQ_CoppeliaSimZmqInterface::get_angular_and_linear_velocities
 * @param objectname
 * @param reference
 * @return
 */
VectorXd DQ_CoppeliaSimZmqInterface::get_angular_and_linear_velocities(std::string &objectname, const REFERENCE &reference)
{
    return get_angular_and_linear_velocities(_get_handle_from_map(objectname), reference);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_angular_and_linear_velocities
 * @param handle
 * @param w
 * @param p_dot
 * @param reference
 */
void DQ_CoppeliaSimZmqInterface::set_angular_and_linear_velocities(const int &handle, const DQ &w, const DQ &p_dot, const REFERENCE &reference) const
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
 * @brief DQ_CoppeliaSimZmqInterface::set_angular_and_linear_velocities
 * @param objectname
 * @param w
 * @param p_dot
 * @param reference
 */
void DQ_CoppeliaSimZmqInterface::set_angular_and_linear_velocities(std::string &objectname, const DQ &w, const DQ &p_dot, const REFERENCE &reference)
{
    set_angular_and_linear_velocities(_get_handle_from_map(objectname), w, p_dot, reference);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_twist
 * @param handle
 * @param reference
 * @return
 */
DQ DQ_CoppeliaSimZmqInterface::get_twist(const int &handle, const REFERENCE &reference) const
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
 * @brief DQ_CoppeliaSimZmqInterface::get_twist
 * @param objectname
 * @param reference
 * @return
 */
DQ DQ_CoppeliaSimZmqInterface::get_twist(const std::string &objectname, const REFERENCE &reference)
{
    return get_twist(_get_handle_from_map(objectname), reference);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_twist
 * @param handle
 * @param twist
 * @param reference
 */
void DQ_CoppeliaSimZmqInterface::set_twist(const int &handle, const DQ& twist, const REFERENCE &reference) const
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
 * @brief DQ_CoppeliaSimZmqInterface::set_twist
 * @param objectname
 * @param twist
 * @param reference
 */
void DQ_CoppeliaSimZmqInterface::set_twist(const std::string &objectname, const DQ &twist, const REFERENCE &reference)
{
    set_twist(_get_handle_from_map(objectname), twist, reference);
}




/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_mode
 * @param jointname
 * @param joint_mode
 */
void DQ_CoppeliaSimZmqInterface::set_joint_mode(const std::string &jointname, const JOINT_MODE &joint_mode)
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
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_modes
 * @param jointnames
 * @param joint_mode
 */
void DQ_CoppeliaSimZmqInterface::set_joint_modes(const std::vector<std::string> &jointnames, const JOINT_MODE &joint_mode)
{
    for(std::size_t i=0;i<jointnames.size();i++)
        set_joint_mode(jointnames.at(i), joint_mode);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_control_mode
 * @param jointname
 * @param joint_control_mode
 */
void DQ_CoppeliaSimZmqInterface::set_joint_control_mode(const std::string &jointname, const JOINT_CONTROL_MODE &joint_control_mode)
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
 * @brief DQ_CoppeliaSimZmqInterface::set_joint_control_modes
 * @param jointnames
 * @param joint_control_mode
 */
void DQ_CoppeliaSimZmqInterface::set_joint_control_modes(const std::vector<std::string> &jointnames, const JOINT_CONTROL_MODE &joint_control_mode)
{
    for(std::size_t i=0;i<jointnames.size();i++)
    {
        set_joint_control_mode(jointnames.at(i), joint_control_mode);
    }
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::enable_dynamics_engine
 * @param flag
 */
void DQ_CoppeliaSimZmqInterface::enable_dynamics(const bool &flag)
{
   _check_client();
   sim_->setBoolParam(sim_->boolparam_dynamics_handling_enabled, flag);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_simulation_time_step
 * @return
 */
double DQ_CoppeliaSimZmqInterface::get_simulation_time_step() const
{
    _check_client();
    return sim_->getFloatParam(sim_->floatparam_simulation_time_step);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_simulation_time_step
 * @param time_step
 */
void DQ_CoppeliaSimZmqInterface::set_simulation_time_step(const double &time_step)
{
    _check_client();
    sim_->setFloatParam(sim_->floatparam_simulation_time_step, time_step);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_physics_time_step
 * @return
 */
double DQ_CoppeliaSimZmqInterface::get_physics_time_step() const
{
    _check_client();
    return sim_->getFloatParam(sim_->floatparam_physicstimestep);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_physics_time_step
 * @param time_step
 */
void DQ_CoppeliaSimZmqInterface::set_physics_time_step(const double &time_step) const
{
    _check_client();
    sim_->setFloatParam(sim_->floatparam_physicstimestep, time_step);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_dynamic_engine
 * @param engine
 */
void DQ_CoppeliaSimZmqInterface::set_engine(const ENGINE &engine)
{
    _check_client();
    sim_->setInt32Param(sim_->intparam_dynamic_engine, engines_.at(engine));
}

std::string DQ_CoppeliaSimZmqInterface::get_engine()
{
    switch (_get_engine()){
    case ENGINE::BULLET:
        return "BULLET";
    case ENGINE::ODE:
        return "ODE";
    case ENGINE::VORTEX:
        return "VORTEX";
    case ENGINE::NEWTON:
        return "NEWTON";
    case ENGINE::MUJOCO:
        return "MUJOCO";
    default: // This line is required in GNU/Linux
        _throw_runtime_error("wrong argument");
    }
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_gravity
 * @param gravity
 */
void DQ_CoppeliaSimZmqInterface::set_gravity(const DQ &gravity)
{
    VectorXd gravity_vec = gravity.vec3();
    std::vector<double> g = {gravity_vec(0),gravity_vec(1), gravity_vec(2)};
    _check_client();
    sim_->setArrayParam(sim_->arrayparam_gravity, g);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_gravity
 * @return
 */
DQ DQ_CoppeliaSimZmqInterface::get_gravity() const
{
    _check_client();
    std::vector<double> g = sim_->getArrayParam(sim_->arrayparam_gravity);
    return DQ(0, g.at(0), g.at(1), g.at(2));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::load_scene loads a scene from your computer.
 * @param path_to_filename the path to the scene. This string must containt
 *        the file extension.
 *
 *        Example:
 *
 *        load_scene("/Users/juanjqo/git/space_robot/scenes/space_robot.ttt");
 */
void DQ_CoppeliaSimZmqInterface::load_scene(const std::string &path_to_filename) const
{
    _check_client();
    sim_->loadScene(path_to_filename);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::save_scene saves the current scene.
 * @param path_to_filename The path where you want to save the scene including
 *        the name of the scene and its file extension.
 *
 *        Example:
 *
 *        save_scene("/Users/juanjqo/git/space_robot/scenes/space_robot2.ttt");
 */
void DQ_CoppeliaSimZmqInterface::save_scene(const std::string &path_to_filename) const
{
    _check_client();
    sim_->saveScene(path_to_filename);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::close_scene closes the current scene.
 */
void DQ_CoppeliaSimZmqInterface::close_scene() const
{
    _check_client();
    sim_->closeScene();
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::remove_child_script_from_object
 *        The script must be located at objectname/script_name
 * @param objectname
 * @param script_name
 */
void DQ_CoppeliaSimZmqInterface::_remove_child_script_from_object(const std::string &objectname, const std::string &script_name)
{
    _check_client();
    if (_object_exist_on_scene(_get_standard_name(objectname)+script_name))
    {
        int handle = _get_handle_from_map(_get_standard_name(objectname)+script_name);
        sim_->removeObjects({handle}, false);
    }
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::object_exist_on_scene
 * @param objectname
 * @return
 */
bool DQ_CoppeliaSimZmqInterface::_object_exist_on_scene(const std::string &objectname) const
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

void DQ_CoppeliaSimZmqInterface::_set_object_name(const int &handle, const std::string &new_object_name) const
{
    _check_client();
    sim_->setObjectAlias(handle, new_object_name);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_object_name
 * @param current_object_name
 * @param new_object_name
 */
void DQ_CoppeliaSimZmqInterface::_set_object_name(const std::string &current_object_name, const std::string &new_object_name)
{
    _set_object_name(_get_handle_from_map(current_object_name), new_object_name);
}


void DQ_CoppeliaSimZmqInterface::_set_object_color(const int &handle,
                                               const std::vector<double> &rgba_color) const
{
    _check_client();
    sim_->setShapeColor(handle, "", sim_->colorcomponent_ambient_diffuse, {rgba_color.at(0), rgba_color.at(1),rgba_color.at(2)});
    sim_->setShapeColor(handle, "", sim_->colorcomponent_transparency, {rgba_color.at(3)});
}

void DQ_CoppeliaSimZmqInterface::_set_object_color(const std::string &objectname, const std::vector<double>& rgba_color)
{
    // For C++20
    // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
    std::string function_name = {"DQ_CoppeliaSimInterface::set_object_color"};
    if (rgba_color.size() != 4)
        _throw_runtime_error(function_name + ". The rgba_color must be a vector of size 4.");

    _set_object_color(_get_handle_from_map(objectname), rgba_color);
}

void DQ_CoppeliaSimZmqInterface::_set_object_as_respondable(const int &handle, const bool &respondable_object) const
{
    _check_client();
    sim_->setObjectInt32Param(handle,
                              sim_->shapeintparam_respondable,
                              (respondable_object == true ? 1 : 0));
}


void DQ_CoppeliaSimZmqInterface::_set_object_as_respondable(const std::string &objectname, const bool &respondable_object)
{
    _set_object_as_respondable(_get_handle_from_map(objectname), respondable_object);
}

void DQ_CoppeliaSimZmqInterface::_set_object_as_static(const int &handle, const bool &static_object) const
{
    _check_client();
    sim_->setObjectInt32Param(handle,
                              sim_->shapeintparam_static,
                              (static_object == true ? 1 : 0));
}

void DQ_CoppeliaSimZmqInterface::_set_object_as_static(const std::string &objectname, const bool &static_object)
{
    _set_object_as_static(_get_handle_from_map(objectname), static_object);
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::set_object_parent
 * @param handle
 * @param parent_handle
 */
void DQ_CoppeliaSimZmqInterface::_set_object_parent(const int &handle, const int &parent_handle, const bool &move_child_to_parent_pose) const
{
    _check_client();
    sim_->setObjectParent(handle, parent_handle, !move_child_to_parent_pose);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_object_parent
 * @param objectname
 * @param parent_object_name
 */
void DQ_CoppeliaSimZmqInterface::_set_object_parent(const std::string &objectname,
                                                const std::string &parent_object_name,
                                                const bool& move_child_to_parent_pose)
{
    _set_object_parent(_get_handle_from_map(objectname), _get_handle_from_map(parent_object_name), move_child_to_parent_pose);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::check_collision
 * @param handle1
 * @param handle2
 * @return
 */
bool DQ_CoppeliaSimZmqInterface::_check_collision(const int &handle1, const int &handle2) const
{
    _check_client();
    auto [result, collidingObjectHandles] = sim_->checkCollision(handle1, handle2);
    return result;
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::check_collision
 * @param objectname1
 * @param objectname2
 * @return
 */
bool DQ_CoppeliaSimZmqInterface::_check_collision(const std::string &objectname1, const std::string &objectname2)
{
    return _check_collision(_get_handle_from_map(objectname1), _get_handle_from_map(objectname2));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::check_distance
 * @param handle1
 * @param handle2
 * @param threshold
 * @return
 */
std::tuple<double, DQ, DQ> DQ_CoppeliaSimZmqInterface::_check_distance(const int &handle1, const int &handle2, const double &threshold) const
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
 * @brief DQ_CoppeliaSimZmqInterface::check_distance
 * @param objectname1
 * @param objectname2
 * @param threshold
 * @return
 */
std::tuple<double, DQ, DQ> DQ_CoppeliaSimZmqInterface::_check_distance(const std::string &objectname1, const std::string &objectname2, const double &threshold)
{
    return _check_distance(_get_handle_from_map(objectname1), _get_handle_from_map(objectname2), threshold);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::compute_distance
 * @param handle1
 * @param handle2
 * @param threshold
 * @return
 */
double DQ_CoppeliaSimZmqInterface::_compute_distance(const int &handle1, const int &handle2, const double &threshold) const
{
    return std::get<0>(_check_distance(handle1, handle2, threshold));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::compute_distance
 * @param objectname1
 * @param objectname2
 * @param threshold
 * @return
 */
double DQ_CoppeliaSimZmqInterface::_compute_distance(const std::string &objectname1, const std::string &objectname2, const double &threshold)
{
    return _compute_distance(_get_handle_from_map(objectname1), _get_handle_from_map(objectname2), threshold);
}




void DQ_CoppeliaSimZmqInterface::_create_plane(const std::string &name, const std::vector<double> &sizes, const std::vector<double> &rgba_color, const bool &add_normal, const double &normal_scale) const
{
    int primitive_handle = _add_primitive(PRIMITIVE::PLANE, name,
                                         {sizes.at(0), sizes.at(1), sizes.at(1)});
    _set_object_color(primitive_handle, rgba_color);
    _set_object_as_respondable(primitive_handle, false);
    _set_object_as_static(primitive_handle, true);
    std::vector<std::string> children_names;

    if (add_normal)
    {
        double rfc = 0.02*normal_scale;
        std::vector<double> scaled_size = {rfc*sizes.at(0),rfc* sizes.at(1), 0.2*normal_scale*sizes.at(1)};
        children_names = _create_static_axis_at_origin(primitive_handle, name, scaled_size, AXIS::k, 1);
    }
    _merge_shapes(primitive_handle);
    //_update_created_handles_map(name, children_names);
}



void DQ_CoppeliaSimZmqInterface::_create_line(const std::string &name, const std::vector<double> &thickness_and_length, const std::vector<double> &rgba_color, const bool &add_arrow, const double &arrow_scale) const
{
    int primitive_handle = _add_primitive(PRIMITIVE::CYLINDER, name,
                          {thickness_and_length.at(0), thickness_and_length.at(0), thickness_and_length.at(1)});
    _set_object_color(primitive_handle, rgba_color);
    _set_object_as_respondable(primitive_handle, false);
    _set_object_as_static(primitive_handle, true);
    std::vector<std::string> children_names;
    if (add_arrow)
    {
        // Add the normal

        double rfc = 2*arrow_scale;
        std::vector<double> arrow_size = {rfc*thickness_and_length.at(0),rfc*thickness_and_length.at(0), 0.02*arrow_scale*thickness_and_length.at(1)};
        std::string arrow_name = _get_standard_name(name)+std::string("_normal");
        int arrow_handle = _add_primitive(PRIMITIVE::CONE, arrow_name, arrow_size);
        children_names.push_back(arrow_name);
        _set_static_object_properties(arrow_handle,
                                      primitive_handle ,
                                      1+0.5*E_*0.5*thickness_and_length.at(1)*k_,
                                      {0,0,1,1});
    }
    _merge_shapes(primitive_handle);
    //_update_created_handles_map(name, children_names);
}




void DQ_CoppeliaSimZmqInterface::_create_cylinder(const std::string &name, const std::vector<double> &width_and_length, const std::vector<double> &rgba_color, const bool &add_line, const double &line_scale) const
{
    int primitive_handle = _add_primitive(PRIMITIVE::CYLINDER, name,
                            {width_and_length.at(0), width_and_length.at(0), width_and_length.at(1)});
    _set_object_color(primitive_handle, rgba_color);
    _set_object_as_respondable(primitive_handle, false);
    _set_object_as_static(primitive_handle, true);
    std::vector<std::string> children_names;
    std::string line_name = _get_standard_name(name)+std::string("_line");
    if (add_line)
    {
        double wscale = 0.05*line_scale;
        double lscale = 1.1*line_scale;
        int line_handle = _add_primitive(PRIMITIVE::CYLINDER, line_name,
                      {wscale*width_and_length.at(0), wscale*width_and_length.at(0), lscale*width_and_length.at(1)});
        children_names.push_back(line_name);
        _set_static_object_properties(line_handle,
                                      primitive_handle,
                                      DQ(1),
                                      {0,0,1,1});

        double rfc = 2*wscale*line_scale;
        std::vector<double> arrow_size = {rfc*width_and_length.at(0),rfc*width_and_length.at(0), 0.02*line_scale*width_and_length.at(1)};
        std::string arrow_name = _get_standard_name(name)+std::string("_normal");
        int arrow_handle = _add_primitive(PRIMITIVE::CONE, arrow_name, arrow_size);
        _set_static_object_properties(arrow_handle,
                                      primitive_handle,
                                      1+0.5*E_*0.5*lscale*width_and_length.at(1)*k_,
                                      {0,0,1,1});
        children_names.push_back(arrow_name);

    }
    _merge_shapes(primitive_handle);
    //_update_created_handles_map(name, children_names);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::_merge_shapes
 * @param parent_handle
 */
void DQ_CoppeliaSimZmqInterface::_merge_shapes(const int &parent_handle) const
{
    std::vector<int64_t> shapehandles = sim_->getObjectsInTree(parent_handle, //_get_handle_from_map(name),
                                                               sim_->object_shape_type,
                                                               0);
    std::reverse(shapehandles.begin(), shapehandles.end());
    sim_->groupShapes(shapehandles, false);
}



void DQ_CoppeliaSimZmqInterface::_create_reference_frame(const std::string &name, const double &scale, const std::vector<double> &thickness_and_length) const
{
    int primitive_handle = _add_primitive(PRIMITIVE::SPHEROID, name,
                                         {1.5*scale*thickness_and_length.at(0), 1.5*scale*thickness_and_length.at(0), 1.5*scale*thickness_and_length.at(0)});
    _set_object_color(primitive_handle, {1,1,1,0.5});
    _set_object_as_respondable(primitive_handle, false);
    _set_object_as_static(primitive_handle, true);
    std::vector<std::string> children_names;
    std::vector<std::string> auxdest;

    std::vector<double> scaled_size = {scale*thickness_and_length.at(0),scale*thickness_and_length.at(0), scale*thickness_and_length.at(1)};
    auto cnames1 = _create_static_axis_at_origin(primitive_handle, name, scaled_size, AXIS::k, 1);
    auto cnames2 = _create_static_axis_at_origin(primitive_handle, name, scaled_size, AXIS::i, 1);
    auto cnames3 = _create_static_axis_at_origin(primitive_handle, name, scaled_size, AXIS::j, 1);

    /*
    std::set_union(cnames1.cbegin(), cnames1.cend(),
                   cnames2.cbegin(), cnames2.cend(),
                   std::back_inserter(auxdest));
    std::set_union(auxdest.cbegin(), auxdest.cend(),
                   cnames3.cbegin(), cnames3.cend(),
                   std::back_inserter(children_names));
    //_update_created_handles_map(name, children_names);
    */

    _merge_shapes(primitive_handle);
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::draw_trajectory
 * @param point
 * @param size
 * @param color
 * @param max_item_count
 */
void DQ_CoppeliaSimZmqInterface::_draw_permanent_trajectory(const DQ &point, const double &size, const std::vector<double> &color, const int &max_item_count)
{
    _check_client();
    if (!is_pure(point) or !is_quaternion(point))
    {
        // For C++20
        // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
        std::string function_name = {"DQ_CoppeliaSimInterface::draw_permanent_trajectory"};
        _throw_runtime_error(function_name + ". The point must be a pure quaternion.");
    }
    VectorXd vpoint = point.vec3();
    std::vector<double> itemdata = {0,0,0,vpoint(0), vpoint(1), vpoint(2)};
    auto drawn_handle = sim_->addDrawingObject(sim_->drawing_lines+sim_->drawing_cyclic,size,0,-1,max_item_count, color);
    sim_->addDrawingObjectItem(
        drawn_handle,
        itemdata
        );
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::add_simulation_lua_script
 * @param script_name
 * @param script_code
 * @return
 */
int DQ_CoppeliaSimZmqInterface::_add_simulation_lua_script(const std::string &script_name, const std::string& script_code)
{
    _check_client();
    int scriptHandle = sim_->createScript(sim_->scripttype_simulation,
                                          script_code, 0, "lua");
    _set_object_name(scriptHandle, _remove_first_slash_from_string(script_name));
    return scriptHandle;
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::draw_trajectory
 * @param objectname
 * @param size
 * @param rgb_color
 * @param max_item_count
 */
void DQ_CoppeliaSimZmqInterface::_draw_trajectory(const std::string &objectname,
                                              const double &size,
                                              const std::vector<double> &rgb_color,
                                              const int &max_item_count)
{

    // For C++20
    // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
    std::string function_name = {"DQ_CoppeliaSimInterface::draw_trajectory"};
    if (!_object_exist_on_scene(objectname))
        _throw_runtime_error(function_name + ". The object " +objectname+ " is not on the scene.");

    if (!_object_exist_on_scene(objectname+"/drawer"))
    {
        if (rgb_color.size() != 3)
            _throw_runtime_error(function_name + ". The rgb_color must be vector of size 3.");
        int r = rgb_color.at(0);
        int g = rgb_color.at(1);
        int b = rgb_color.at(2);

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
        _add_simulation_lua_script("/drawer", code);
        _set_object_parent("/drawer", objectname);
    }
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::remove_object
 * @param objectname
 * @param remove_children
 */
void DQ_CoppeliaSimZmqInterface::_remove_object(const std::string& objectname, const bool &remove_children)
{
    _check_client();
    auto standard_objectname = _get_standard_name(objectname);
    auto handle = _get_handle_from_map(standard_objectname);

    if (remove_children)
    {
        auto handles = sim_->getObjectsInTree(handle, sim_->handle_all, 0);
        if (handles.size() > 0)
        {
            auto objectnames = get_object_names(handles);
            sim_->removeObjects(handles, false);
            for (std::size_t i=0; i<objectnames.size();i++)
                _update_map(_get_standard_name(objectnames.at(i)), handles.at(i), UPDATE_MAP::REMOVE);
        }
    }else
    {
        sim_->removeObjects({handle}, false);
        _update_map(standard_objectname, handle, UPDATE_MAP::REMOVE);
    }
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_bounding_box_size
 * @param handle
 * @return
 */
std::vector<double> DQ_CoppeliaSimZmqInterface::_get_bounding_box_size(const int &handle) const
{
    _check_client();
    auto [size, pose] = sim_->getShapeBB(handle);
    return size;
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_bounding_box_size
 * @param objectname
 * @return
 */
std::vector<double> DQ_CoppeliaSimZmqInterface::_get_bounding_box_size(const std::string &objectname)
{
    return _get_bounding_box_size(_get_handle_from_map(objectname));
}


bool DQ_CoppeliaSimZmqInterface::_mujoco_is_used()
{
    return _get_engine() == ENGINE::MUJOCO ? true : false;
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_mujoco_global_impratio This attribute determines the ratio of
 *        frictional-to-normal constraint impedance for elliptic friction cones.
 *        The setting of solimp determines a single impedance value for
 *        all contact dimensions, which is then modulated by this attribute.
 *        Settings larger than 1 cause friction forces to be “harder” than
 *        normal forces, having the general effect of preventing slip,
 *        without increasing the actual friction coefficient.
 *        For pyramidal friction cones the situation is more complex because
 *        the pyramidal approximation mixes normal and frictional dimensions
 *        within each basis vector; it is not recommended to use high impratio
 *        values with pyramidal cones.
 *
 * @param impratio
 */
void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_impratio(const double &impratio)
{
    sim_->setEngineFloatParam(sim_->mujoco_global_impratio,-1, impratio);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_mujoco_global_wind Velocity vector of the medium (i.e., wind).
 *                      This vector is subtracted from the 3D translational velocity of each body,
 *                      and the result is used to compute viscous, lift and drag forces acting on the body;
 * @param wind
 */
void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_wind(const std::vector<double> &wind)
{
    std::vector<int64_t> mujoco_global_param= {sim_->mujoco_global_wind1,
                                               sim_->mujoco_global_wind2,
                                               sim_->mujoco_global_wind3};
    _check_sizes(wind, mujoco_global_param, "Error in DQ_CoppeliaSimInterface::set_mujoco_global_wind: "
                                            "argument must be a vector of size "+std::to_string(mujoco_global_param.size()));
    for (size_t i=0;i<wind.size();i++)
        sim_->setEngineFloatParam(mujoco_global_param.at(i),-1, wind.at(i));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_mujoco_global_density Density of the medium, not to be confused with the geom density used to infer masses and inertias.
 *                                 This parameter is used to simulate lift and drag forces, which scale quadratically with velocity.
 *                                 In SI units the density of air is around 1.2 while the density of water is around 1000 depending on temperature. Setting density to 0 disables lift and drag forces.
 * @param density
 */
void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_density(const double &density)
{
    sim_->setEngineFloatParam(sim_->mujoco_global_density,-1, density);
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::set_mujoco_global_viscosity Viscosity of the medium. This parameter is used to simulate viscous forces,
 *                      which scale linearly with velocity.
 *                      In SI units the viscosity of air is around 0.00002 while the viscosity of water is around 0.0009 depending on temperature.
 *                      Setting viscosity to 0 disables viscous forces.
 *                      Note that the default Euler integrator handles damping in
 *                      the joints implicitly – which improves stability and accuracy.
 *                      It does not presently do this with body viscosity.
 *                      Therefore, if the goal is merely to create a damped simulation
 *                      (as opposed to modeling the specific effects of viscosity),
 *                      we recommend using joint damping rather than body viscosity,
 *                      or switching to the implicit or implicitfast integrators.
 * @param viscosity
 */
void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_viscosity(const double &viscosity)
{
    sim_->setEngineFloatParam(sim_->mujoco_global_viscosity,-1, viscosity);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_boundmass(const double &boundmass)
{
    sim_->setEngineFloatParam(sim_->mujoco_global_boundmass,-1, boundmass);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_boundinertia(const double &boundinertia)
{
    sim_->setEngineFloatParam(sim_->mujoco_global_boundinertia,-1, boundinertia);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_overridemargin(const double &overridemargin)
{
    sim_->setEngineFloatParam(sim_->mujoco_global_overridemargin,-1, overridemargin);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_overridesolref(const std::vector<double> &overridesolref)
{
    std::vector<int64_t> mujoco_global_param = {sim_->mujoco_global_overridesolref1,
                                                sim_->mujoco_global_overridesolref2};
    _check_sizes(overridesolref, mujoco_global_param, "Error in DQ_CoppeliaSimInterface::set_mujoco_global_overridesolref: "
                                                      "argument must be a vector of size "+std::to_string(mujoco_global_param.size()));
    for (size_t i=0;i<overridesolref.size();i++)
        sim_->setEngineFloatParam(mujoco_global_param.at(i),-1, overridesolref.at(i));
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_overridesolimp(const std::vector<double> &overridesolimp)
{
    std::vector<int64_t> mujoco_global_param = {sim_->mujoco_global_overridesolimp1,
                                                sim_->mujoco_global_overridesolimp2,
                                                sim_->mujoco_global_overridesolimp3,
                                                sim_->mujoco_global_overridesolimp4,
                                                sim_->mujoco_global_overridesolimp5,
                                                };
    _check_sizes(overridesolimp, mujoco_global_param, "Error in DQ_CoppeliaSimInterface::set_mujoco_global_overridesolimp: "
                                                      "argument must be a vector of size "+std::to_string(mujoco_global_param.size()));
    for (size_t i=0;i<overridesolimp.size();i++)
        sim_->setEngineFloatParam(mujoco_global_param.at(i),-1, overridesolimp.at(i));
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_iterations(const int &iterations)
{
    sim_->setEngineInt32Param(sim_->mujoco_global_iterations,-1, iterations);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_integrator(const int &integrator)
{
    sim_->setEngineInt32Param(sim_->mujoco_global_integrator,-1, integrator);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_solver(const int &solver)
{
    sim_->setEngineInt32Param(sim_->mujoco_global_solver,-1, solver);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_njmax(const int &njmax)
{
    sim_->setEngineInt32Param(sim_->mujoco_global_njmax,-1, njmax);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_nstack(const int &nstack)
{
    sim_->setEngineInt32Param(sim_->mujoco_global_nstack,-1, nstack);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_nconmax(const int &nconmax)
{
    sim_->setEngineInt32Param(sim_->mujoco_global_nconmax,-1, nconmax);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_cone(const int &cone)
{
    sim_->setEngineInt32Param(sim_->mujoco_global_cone,-1, cone);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_overridekin(const int &overridekin)
{
    sim_->setEngineInt32Param(sim_->mujoco_global_overridekin,-1, overridekin);
}

/*
void DQ_CoppeliaSimZmqInterface::set_mujoco_global_rebuildcondition(const int &rebuildcondition)
{
   sim_->setEngineInt32Param(sim_->mujoco_global_rebuildcondition,-1, rebuildcondition);
}
*/

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_computeinertias(const bool &computeinertias)
{
   sim_->setEngineBoolParam(sim_->mujoco_global_computeinertias,-1, computeinertias);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_multithreaded(const bool &multithreaded)
{
   sim_->setEngineBoolParam(sim_->mujoco_global_multithreaded,-1, multithreaded);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_multiccd(const bool &multiccd)
{
   sim_->setEngineBoolParam(sim_->mujoco_global_multiccd,-1, multiccd);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_balanceinertias(const bool &balanceinertias)
{
   sim_->setEngineBoolParam(sim_->mujoco_global_balanceinertias,-1, balanceinertias);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_global_overridecontacts(const bool &overridecontacts)
{
    sim_->setEngineBoolParam(sim_->mujoco_global_overridecontacts,-1, overridecontacts);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_mujoco_joint_stiffness
 *                         Joint stiffness.
 *                         If this value is positive, a spring will be created with equilibrium position
 *                         given by springref below.
 *                         The spring force is computed along with the other passive forces.
 * @param jointname
 * @param stiffness
 */
void DQ_CoppeliaSimZmqInterface::_set_mujoco_joint_stiffness(const std::string &jointname, const double &stiffness)
{
   sim_->setEngineFloatParam(sim_->mujoco_joint_stiffness,_get_handle_from_map(jointname), stiffness);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_mujoco_joint_stiffness
 *                         Joint stiffness.
 *                         If this value is positive, a spring will be created with equilibrium position
 *                         given by springref below.
 *                         The spring force is computed along with the other passive forces.
 * @param jointnames
 * @param stiffness
 */
void DQ_CoppeliaSimZmqInterface::_set_mujoco_joint_stiffnesses(const std::vector<std::string> &jointnames,
                                                           const double &stiffness)
{
    for (size_t i=0;i<jointnames.size();i++)
        _set_mujoco_joint_stiffness(jointnames.at(i), stiffness);

}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_joint_damping(const std::string &jointname, const double &damping)
{
    sim_->setEngineFloatParam(sim_->mujoco_joint_damping, _get_handle_from_map(jointname), damping);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_joint_dampings(const std::vector<std::string> &jointnames,
                                                        const double &damping)
{
    for (size_t i=0;i<jointnames.size();i++)
        _set_mujoco_joint_damping(jointnames.at(i), damping);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::set_mujoco_joint_armature Armature inertia (or rotor inertia, or reflected inertia)
 *                      of all degrees of freedom created by this joint. These are constants added to the
 *                      diagonal of the inertia matrix in generalized coordinates.
 *                      They make the simulation more stable, and often increase physical realism.
 *                      This is because when a motor is attached to the system with a transmission
 *                      that amplifies the motor force by c, the inertia of the rotor
 *                      (i.e., the moving part of the motor) is amplified by c*c.
 *                      The same holds for gears in the early stages of planetary gear boxes.
 *                      These extra inertias often dominate the inertias of the robot parts that are
 *                      represented explicitly in the model, and the armature attribute is the way to model them.
 * @param jointname
 * @param armature
 */
void DQ_CoppeliaSimZmqInterface::_set_mujoco_joint_armature(const std::string &jointname, const double &armature)
{
   sim_->setEngineFloatParam(sim_->mujoco_joint_armature,_get_handle_from_map(jointname), armature);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_joint_armatures(const std::vector<std::string> &jointnames,
                                                         const double &armature)
{
    for (size_t i=0;i<jointnames.size();i++)
        _set_mujoco_joint_armature(jointnames.at(i), armature);
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_body_friction(const std::string &bodyname, const std::vector<double> &friction)
{
    std::vector<int64_t> mujoco_body_param = {sim_->mujoco_body_friction1,
                                              sim_->mujoco_body_friction2,
                                              sim_->mujoco_body_friction3};
    _check_sizes(friction, mujoco_body_param, "Error in DQ_CoppeliaSimInterface::set_mujoco_body_friction: "
                                              "friction must be a vector of size "+std::to_string(mujoco_body_param.size()));
    for (size_t i=0;i<mujoco_body_param.size();i++)
        sim_->setEngineFloatParam(mujoco_body_param.at(i), _get_handle_from_map(bodyname),friction.at(i));
}

void DQ_CoppeliaSimZmqInterface::_set_mujoco_body_frictions(const std::vector<std::string> &bodynames, const std::vector<double> &friction)
{
    for (auto& bodyname : bodynames)
        _set_mujoco_body_friction(bodyname, friction);
}




/**
 * @brief DQ_CoppeliaSimZmqInterface::get_mass
 * @param handle
 * @return
 */
double DQ_CoppeliaSimZmqInterface::get_mass(const int &handle) const
{
   _check_client();
   return sim_->getShapeMass(handle);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_mass
 * @param object_name
 * @return
 */
double DQ_CoppeliaSimZmqInterface::get_mass(const std::string &object_name)
{
    return get_mass(_get_handle_from_map(object_name));
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_center_of_mass
 * @param handle
 * @param reference_frame
 * @return
 */
DQ DQ_CoppeliaSimZmqInterface::get_center_of_mass(const int &handle, const REFERENCE &reference_frame) const
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
 * @brief DQ_CoppeliaSimZmqInterface::get_center_of_mass
 * @param object_name
 * @param reference_frame
 * @return
 */
DQ DQ_CoppeliaSimZmqInterface::get_center_of_mass(const std::string &object_name, const REFERENCE &reference_frame)
{
    return get_center_of_mass(_get_handle_from_map(object_name), reference_frame);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_inertia_matrix
 * @param handle
 * @param reference_frame
 * @return
 */
MatrixXd DQ_CoppeliaSimZmqInterface::get_inertia_matrix(const int &handle, const REFERENCE &reference_frame)
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
 * @brief DQ_CoppeliaSimZmqInterface::get_inertia_matrix
 * @param link_name
 * @param reference_frame
 * @return
 */
MatrixXd DQ_CoppeliaSimZmqInterface::get_inertia_matrix(const std::string &link_name, const REFERENCE &reference_frame)
{
    return get_inertia_matrix(_get_handle_from_map(link_name), reference_frame);
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::_update_map updates the map if and only if
 *        the objectname is not in the map.
 * @param objectname
 * @param handle
 */
void DQ_CoppeliaSimZmqInterface::_update_map(const std::string &objectname,
                                          const int &handle, const UPDATE_MAP &mode)
{
    if (mode == DQ_CoppeliaSimZmqInterface::UPDATE_MAP::ADD)
        handles_map_.try_emplace(objectname, handle);
    else
        handles_map_.erase(objectname);

}


/**
 * @brief DQ_CoppeliaSimZmqInterface::_get_handle_from_map searchs a handle in the map.
 *
 * @param objectname
 * @return a tuple <bool, int>. If the handle is found in the map, returns <true, handle>.
 *                              Otherwise, returns <false, 0>
 */
int DQ_CoppeliaSimZmqInterface::_get_handle_from_map(const std::string &objectname)
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
void DQ_CoppeliaSimZmqInterface::disconnect(){}
void DQ_CoppeliaSimZmqInterface::disconnect_all(){}
void DQ_CoppeliaSimZmqInterface::set_synchronous(const bool &flag){set_stepping_mode(flag);}
int DQ_CoppeliaSimZmqInterface::wait_for_simulation_step_to_end(){return 0;}

bool DQ_CoppeliaSimZmqInterface::connect(const std::string &ip, const int &port, const int &TIMEOUT_IN_MILISECONDS,
                                         const int &MAX_TRY_COUNT)
{
    return connect(ip, port, TIMEOUT_IN_MILISECONDS);
}


//---------------Private methods-----------------------------
/**
 * @brief DQ_CoppeliaSimZmqInterface::_remove_first_slash_from_string this method removes the slash at the beginning of a string.
 *
 *             Example: _remove_first_slash_from_string("/reference") returns "reference"
 *                      _remove_first_slash_from_string("reference")  returns "reference"
 *
 * @param str
 * @return
 */
std::string DQ_CoppeliaSimZmqInterface::_remove_first_slash_from_string(const std::string &str) const
{
    std::string new_str = str;
    auto found = str.find('/');
    if (found != std::string::npos && found == 0) // The string containt the '/' in the first position
            new_str.erase(0,1); // remove the '/'
    return new_str;
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::_start_with_slash returns true if the first character of a string is a slash.
 *                  Otherwise returns false
 * @param str
 * @return
 */
bool DQ_CoppeliaSimZmqInterface::_start_with_slash(const std::string &str) const
{

    size_t found = str.find('/');
    if(found == 0) // The string containt the '/'
        return true;
    else
        return false;

    //return str.starts_with("/");

}

/**
 * @brief DQ_CoppeliaSimZmqInterface::_get_standard_string returns a string that
 *        always start with "/"
 * @param str
 * @return
 */
std::string DQ_CoppeliaSimZmqInterface::_get_standard_name(const std::string &str) const
{
    std::string standard_str = str;
    if (!_start_with_slash(str) && enable_deprecated_name_compatibility_ == true)
        standard_str = std::string("/")+str;
    return standard_str;
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::_get_engine returns the current engine in the simulation scene.
 * @return
 */
DQ_CoppeliaSimZmqInterface::ENGINE DQ_CoppeliaSimZmqInterface::_get_engine()
{
    _check_client();
    int64_t eng = sim_->getInt32Param(sim_->intparam_dynamic_engine);
    return engines_invmap.at(eng);
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::_get_velocity_const_params
 * @return
 */
std::vector<int> DQ_CoppeliaSimZmqInterface::_get_velocity_const_params() const
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

std::string DQ_CoppeliaSimZmqInterface::_get_resources_path() const
{
    return sim_->getStringParam(sim_->stringparam_resourcesdir);
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::_load_model
 * @param path_to_filename
 * @param desired_model_name
 * @return
 */
bool DQ_CoppeliaSimZmqInterface::_load_model(const std::string &path_to_filename,
                                          const std::string &desired_model_name,
                                          const bool &remove_child_script)
{
    int rtn = sim_->loadModel(path_to_filename);
    if (rtn != -1)
    {
        _set_object_name(rtn, _remove_first_slash_from_string(desired_model_name));
        _get_handle_from_map(_get_standard_name(desired_model_name)); // This is to update the map only.
        if (remove_child_script)
        {
            _remove_child_script_from_object(std::string("/")
                                        + _remove_first_slash_from_string(desired_model_name));
        }
        return true;
    }else{
        return false;
    }
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::get_transformation_matrix
 * @param coeff_vector
 * @return
 */
MatrixXd DQ_CoppeliaSimZmqInterface::_get_transformation_matrix(const std::vector<double> &coeff_vector) const
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
MatrixXd DQ_CoppeliaSimZmqInterface::_get_rotation_matrix(const DQ& r) const{
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
 * @brief DQ_CoppeliaSimZmqInterface::_get_rotation_from_direction
 * @param direction
 * @return
 */
DQ DQ_CoppeliaSimZmqInterface::_get_pose_from_direction(const DQ& direction, const DQ& point)
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

int DQ_CoppeliaSimZmqInterface::_add_primitive(const PRIMITIVE &primitive, const std::string &name, const std::vector<double> &sizes) const
{
    if (!_object_exist_on_scene(name))
    {
        _check_client();
        int shapeHandle = sim_->createPrimitiveShape(_get_primitive_identifier(primitive), sizes, 0);
        _set_object_name(shapeHandle, _remove_first_slash_from_string(name));
        return shapeHandle;
    }else
        return -1;
}

std::vector<std::string> DQ_CoppeliaSimZmqInterface::_create_static_axis_at_origin(const int& parent_handle,
                                                           const std::string& parent_name,
                                                           const std::vector<double>& sizes,
                                                           const AXIS& axis,
                                                                                const double& alpha_color) const
{
    std::vector<std::string> created_primitives{};
    std::string name;
    DQ dqaxis;
    std::vector<double> color;
    DQ rotation;
    const double angle = M_PI/2; // std::numbers::pi/2;
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
    int primitive_handle  = _add_primitive(PRIMITIVE::CYLINDER,name,sizes);
    created_primitives.push_back(name);
    _set_static_object_properties(primitive_handle,
                                  parent_handle,
                                  rotation+0.5*E_*0.5*sizes.at(2)*dqaxis*rotation,
                                  color
                                  );
    std::vector<double> arrow_size = {2*sizes.at(0), 2*sizes.at(1), 2*sizes.at(1)};
    std::string arrow_name = _get_standard_name(name)+std::string("_");
    int primitive_arrow = _add_primitive(PRIMITIVE::CONE, arrow_name, arrow_size);
    created_primitives.push_back(arrow_name);
    _set_static_object_properties(primitive_arrow,
                                  parent_handle ,
                                  rotation+0.5*E_*sizes.at(2)*dqaxis*rotation,
                                  color);
    return created_primitives;
}

void DQ_CoppeliaSimZmqInterface::_set_static_object_properties(const std::string &name,
                                                               const std::string &parent_name,
                                                               const DQ &pose,
                                                               const std::vector<double> &rgba_color)
{
    _set_object_color(name, rgba_color);
    _set_object_as_respondable(name, false);
    _set_object_as_static(name, true);
    set_object_pose(name, pose);
    _set_object_parent(name, parent_name, false);
}

void DQ_CoppeliaSimZmqInterface::_set_static_object_properties(const int &handle,
                                                               const int &parent_handle,
                                                               const DQ &pose,
                                                               const std::vector<double> &rgba_color) const
{
    _set_object_color(handle, rgba_color);
    _set_object_as_respondable(handle, false);
    _set_object_as_static(handle, true);
    set_object_pose(handle, pose);
    _set_object_parent(handle, parent_handle, false);
}

int DQ_CoppeliaSimZmqInterface::_get_primitive_identifier(const PRIMITIVE &primitive) const
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
    default: // This line is required in GNU/Linux
        _throw_runtime_error("wrong argument");

    }
}

/**
 * @brief DQ_CoppeliaSimZmqInterface::_check_client checks if the client is initialized.
 */
void DQ_CoppeliaSimZmqInterface::_check_client() const
{
    if (!client_created_)
        throw std::runtime_error("Unestablished connection. Did you use connect()?");
}

[[noreturn]] void DQ_CoppeliaSimZmqInterface::_throw_runtime_error(const std::string &msg) const
{
    stop_simulation();
    std::cerr<<"Something went wrong. I stopped the simulation!"<<std::endl;
    throw std::runtime_error(msg);
}


/**
 * @brief DQ_CoppeliaSimZmqInterface::get_center_of_mass_and_inertia_matrix
 * @param handle
 * @return
 */
std::tuple<DQ, MatrixXd> DQ_CoppeliaSimZmqInterface::_get_center_of_mass_and_inertia_matrix(const int &handle) const
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


DQ_CoppeliaSimZmqInterface::experimental::experimental(const std::shared_ptr<DQ_CoppeliaSimZmqInterface> &smptr)
    :smptr_{smptr}
{

}

void DQ_CoppeliaSimZmqInterface::experimental::plot_reference_frame(const std::string &name,
                                                                    const DQ &pose,
                                                                    const double &scale,
                                                                    const std::vector<double> &thickness_and_length)
{
    smptr_->_check_client();

    // For C++20
    // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
    std::string function_name = {"DQ_CoppeliaSimInterface::plot_reference_frame"};

    if (!is_unit(pose))
        smptr_->_throw_runtime_error(function_name  + ". The pose must be a unit dual quaternion!");

    if (thickness_and_length.size() != 2)
        smptr_->_throw_runtime_error(function_name  + ". The thickness_and_length must be vector of size 2.");

    if (!smptr_->_object_exist_on_scene(name))
    {
        smptr_->_create_reference_frame(name, scale, thickness_and_length);
    }
    smptr_->set_object_pose(name, pose);
}

void DQ_CoppeliaSimZmqInterface::experimental::plot_plane(const std::string &name, const DQ &normal_to_the_plane, const DQ &location, const std::vector<double> &sizes, const std::vector<double> &rgba_color, const bool &add_normal, const double &normal_scale)
{
    // For C++20
    // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
    std::string function_name = {"DQ_CoppeliaSimInterface::plot_plane"};

    if (!is_unit(normal_to_the_plane) or !is_quaternion(normal_to_the_plane))
        smptr_->_throw_runtime_error(function_name + ". The normal to the plane must be a unit quaternion!");
    if (!is_pure(location) or !is_quaternion(location))
        smptr_->_throw_runtime_error(function_name + ". The location must be a pure quaternion!");

    if (sizes.size() != 2)
        smptr_->_throw_runtime_error(function_name + ". The sizes must be vector of size 2.");

    if (rgba_color.size() != 4)
        smptr_->_throw_runtime_error(function_name + ". The rgba_color must be vector of size 4.");

    if (!smptr_->_object_exist_on_scene(name))
    {
        smptr_->_create_plane(name, sizes, rgba_color, add_normal, normal_scale);
    }
    smptr_->set_object_pose(name, smptr_->_get_pose_from_direction(normal_to_the_plane, location));
}


void DQ_CoppeliaSimZmqInterface::experimental::plot_line(const std::string &name, const DQ &line_direction, const DQ &location, const std::vector<double> &thickness_and_length, const std::vector<double> &rgba_color, const bool &add_arrow, const double &arrow_scale)
{
    // For C++20
    // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
    std::string function_name = {"DQ_CoppeliaSimInterface::plot_line"};

    if (!is_unit(line_direction) or !is_quaternion(line_direction))
        smptr_->_throw_runtime_error(function_name + ". The line direction must be a unit quaternion!");

    if (!is_pure(location) or !is_quaternion(location))
        smptr_->_throw_runtime_error(function_name + ". The location must be a pure quaternion!");

    if (thickness_and_length.size() != 2)
        smptr_->_throw_runtime_error(function_name + ". The thickness_and_length must be vector of size 2.");

    if (rgba_color.size() != 4)
        smptr_->_throw_runtime_error(function_name + ". The rgba_color must be vector of size 4.");

    if (!smptr_->_object_exist_on_scene(name))
    {
        smptr_->_create_line(name, thickness_and_length, rgba_color, add_arrow, arrow_scale);
    }
    smptr_->set_object_pose(name, smptr_->_get_pose_from_direction(line_direction, location));
}


void DQ_CoppeliaSimZmqInterface::experimental::plot_cylinder(const std::string &name, const DQ &direction, const DQ &location, const std::vector<double> &width_and_length, const std::vector<double> &rgba_color, const bool &add_line, const double &line_scale)
{
    // For C++20
    // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
    std::string function_name = {"DQ_CoppeliaSimInterface::plot_cylinder"};
    if (!is_unit(direction) or !is_quaternion(direction))
        smptr_->_throw_runtime_error(function_name + ". The line direction must be a unit quaternion!");

    if (!is_pure(location) or !is_quaternion(location))
        smptr_->_throw_runtime_error(function_name + ". The location must be a pure quaternion!");

    if (width_and_length.size() != 2)
        smptr_->_throw_runtime_error(function_name + ". The thickness_and_length must be vector of size 2.");

    if (rgba_color.size() != 4)
        smptr_->_throw_runtime_error(function_name + ". The rgba_color must be vector of size 4.");

    if (!smptr_->_object_exist_on_scene(name))
    {
        smptr_->_create_cylinder(name, width_and_length, rgba_color, add_line, line_scale);
    }
    smptr_->set_object_pose(name, smptr_->_get_pose_from_direction(direction, location));
}

void DQ_CoppeliaSimZmqInterface::experimental::plot_sphere(const std::string &name, const DQ &location, const double &size, const std::vector<double> rgba_color)
{
    // For C++20
    // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
    std::string function_name = {"DQ_CoppeliaSimInterface::plot_sphere"};

    if (!is_pure(location) or !is_quaternion(location))
        smptr_->_throw_runtime_error(function_name + ". The location must be a pure quaternion!");
    if (rgba_color.size() != 4)
        smptr_->_throw_runtime_error(function_name + ". The rgba_color must be vector of size 4.");

    if (!smptr_->_object_exist_on_scene(name))
    {
        int primitive_handle = smptr_->_add_primitive(PRIMITIVE::SPHEROID, name,{size, size, size});
        smptr_->_set_object_color(primitive_handle, rgba_color);
        smptr_->_set_object_as_respondable(primitive_handle, false);
        smptr_->_set_object_as_static(primitive_handle, true);
        //std::vector<std::string> children_names;
        //_update_created_handles_map(name, children_names);
    }
    smptr_->set_object_pose(name, 1+0.5*E_*location);
}


bool DQ_CoppeliaSimZmqInterface::experimental::load_model(const std::string &path_to_filename, const std::string &desired_model_name, const bool &load_model_only_if_missing, const bool &remove_child_script)
{
    if (load_model_only_if_missing == true)
    {
        if (!smptr_->_object_exist_on_scene(std::string("/") +
                                   smptr_->_remove_first_slash_from_string(desired_model_name)))
        {
            return smptr_->_load_model(path_to_filename, desired_model_name, remove_child_script);
        }else
            return true;
    }else
    {// Load the model even if the model is already on the scene
        return smptr_->_load_model(path_to_filename, desired_model_name, remove_child_script);
    }
}

bool DQ_CoppeliaSimZmqInterface::experimental::load_from_model_browser(const std::string &path_to_filename, const std::string &desired_model_name, const bool &load_model_only_if_missing, const bool &remove_child_script)
{
    smptr_->_check_client();
    return load_model(smptr_->_get_resources_path() + std::string("/models") + path_to_filename,
                      desired_model_name, load_model_only_if_missing, remove_child_script);
}

int DQ_CoppeliaSimZmqInterface::experimental::add_primitive(const PRIMITIVE &primitive, const std::string &name, const std::vector<double> &sizes)
{
    return smptr_->_add_primitive(primitive, name, sizes);
}

bool DQ_CoppeliaSimZmqInterface::experimental::object_exist_on_scene(const std::string &objectname) const
{
    return smptr_->_object_exist_on_scene(objectname);
}

void DQ_CoppeliaSimZmqInterface::experimental::remove_object(const std::string &objectname, const bool &remove_children)
{
    smptr_->_remove_object(objectname, remove_children);
}

void DQ_CoppeliaSimZmqInterface::experimental::set_mujoco_joint_stiffness(const std::string &jointname, const double &stiffness)
{
    smptr_->_set_mujoco_joint_stiffness(jointname, stiffness);
}

void DQ_CoppeliaSimZmqInterface::experimental::set_mujoco_joint_stiffnesses(const std::vector<std::string> &jointnames, const double &stiffness)
{
    smptr_->_set_mujoco_joint_stiffnesses(jointnames, stiffness);
}

void DQ_CoppeliaSimZmqInterface::experimental::set_mujoco_joint_damping(const std::string &jointname, const double &damping)
{
    smptr_->_set_mujoco_joint_damping(jointname, damping);
}

void DQ_CoppeliaSimZmqInterface::experimental::set_mujoco_joint_dampings(const std::vector<std::string> &jointnames, const double &damping)
{
    smptr_->_set_mujoco_joint_dampings(jointnames, damping);
}

void DQ_CoppeliaSimZmqInterface::experimental::set_mujoco_joint_armature(const std::string &jointname, const double &armature)
{
    smptr_->_set_mujoco_joint_armature(jointname, armature);
}

void DQ_CoppeliaSimZmqInterface::experimental::set_mujoco_joint_armatures(const std::vector<std::string> &jointnames, const double &armature)
{
    smptr_->_set_mujoco_joint_armatures(jointnames, armature);
}

void DQ_CoppeliaSimZmqInterface::experimental::set_mujoco_body_friction(const std::string &bodyname, const std::vector<double> &friction)
{
    smptr_->_set_mujoco_body_friction(bodyname, friction);
}

void DQ_CoppeliaSimZmqInterface::experimental::set_mujoco_body_frictions(const std::vector<std::string> &bodynames, const std::vector<double> &friction)
{
    smptr_->_set_mujoco_body_frictions(bodynames, friction);
}

