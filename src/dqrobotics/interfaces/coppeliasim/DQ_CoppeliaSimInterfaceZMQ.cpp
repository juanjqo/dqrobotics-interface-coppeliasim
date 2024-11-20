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
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <algorithm>
#include "internal/_zmq_wrapper.h"


/**
 * @brief _set_status_bar_message
 * @param message
 * @param verbosity_type
 */
void DQ_CoppeliaSimInterfaceZMQ::__set_status_bar_message(const std::string &message, const int& verbosity_type) const
{
    _ZMQWrapper::get_sim()->addLog(verbosity_type, message);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::DQ_CoppeliaSimInterfaceZMQ
 */
DQ_CoppeliaSimInterfaceZMQ::DQ_CoppeliaSimInterfaceZMQ()
    :client_created_{false}
{

}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::~DQ_CoppeliaSimInterfaceZMQ
 */
DQ_CoppeliaSimInterfaceZMQ::~DQ_CoppeliaSimInterfaceZMQ()
{
    _join_if_joinable_chronometer_thread();
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::_join_if_joinable_chronometer_thread
 */
void DQ_CoppeliaSimInterfaceZMQ::_join_if_joinable_chronometer_thread()
{
    if (chronometer_thread_.joinable())
        chronometer_thread_.join();
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::_start_chronometer
 */
void DQ_CoppeliaSimInterfaceZMQ::_start_chronometer()
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
 * @brief DQ_CoppeliaSimInterfaceZMQ::_check_connection
 */
void DQ_CoppeliaSimInterfaceZMQ::_check_connection()
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
 * @brief DQ_CoppeliaSimInterfaceZMQ::connect establishes a connection between the client (your code) and
 *                                         the host (the computer running the CoppeliaSim scene).
 * @param host    eg. 'localhost' if the host is running in the same
 *                machine in which is running the client.
 * @param rpcPort The port to establish a connection. (e.g. 23000, 23001, 23002, 23003...).
 * @param MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION The timeout to establish the connection.
 * @param cntPort
 * @param verbose
 * @return
 */
bool DQ_CoppeliaSimInterfaceZMQ::_connect(const std::string &host, const int &rpcPort, const int &MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION, const int &cntPort, const int &verbose)
{
    try
    {
        host_ = host;
        rpcPort_ = rpcPort;
        cntPort_ = cntPort;
        verbose_ = verbose;
        MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_ = MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION;

        _join_if_joinable_chronometer_thread();
        chronometer_thread_ = std::thread(&DQ_CoppeliaSimInterfaceZMQ::_start_chronometer, this);


        client_created_ = _ZMQWrapper::create_client(host, rpcPort, cntPort, verbose);//_create_client(host, rpcPort, cntPort, verbose, client_created_);

        _join_if_joinable_chronometer_thread();
        _set_status_bar_message("       ");
        __set_status_bar_message("DQ Robotics established a connection on port " + std::to_string(rpcPort),
                                _ZMQWrapper::get_sim()->verbosity_warnings);
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Runtime error in DQ_CoppeliaSimZmqInterface::connect. "
                  << e.what() << std::endl;
    }
    return client_created_;
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::connect establish a connection between the client (your code) and
 *             the host (the computer running the CoppeliaSim scene).
 *             Calling this method is required before anything else can happen.
 * @param host The IP address of the computer that hosts the CoppeliaSim simulation. If the client (your code)
 *             and the simulation are running in the same computer, you can use "localhost".
 * @param port The port to establish a connection. (e.g. 23000, 23001, 23002, 23003...).
 * @param TIMEOUT_IN_MILISECONDS The timeout to establish the connection.
 * @return true if the connection is established. False otherwise.
 */
bool DQ_CoppeliaSimInterfaceZMQ::connect(const std::string &host, const int &port, const int &TIMEOUT_IN_MILISECONDS)
{
    return _connect(host, port, TIMEOUT_IN_MILISECONDS, -1, -1);
}

int DQ_CoppeliaSimInterfaceZMQ::_get_port_from_deprecated_default_port(const int &port)
{
    int auxport = port;
    if (auxport == 19997 or auxport == 19998 or auxport == 19999 or auxport == 20000)
    {
        auxport = 23000;
        std::cerr<<"The port "<<port<<" is commonly used in the legacy API. However it is not compatible with the ZMQ Remote API."<<std::endl;
        std::cerr<<"I changed the port to "<<auxport<<std::endl;
    }
    return auxport;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::_map_simulation_state
 * @param state
 * @return
 */
std::string DQ_CoppeliaSimInterfaceZMQ::_map_simulation_state(const int &state)
{
    return simulation_status_[state];
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::start_simulation starts the CoppeliaSim simulation.
 */
void DQ_CoppeliaSimInterfaceZMQ::start_simulation() const
{
    _check_client();
    _ZMQWrapper::get_sim()->startSimulation();
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::pause_simulation pauses the CoppeliaSim simulation.
 */
void DQ_CoppeliaSimInterfaceZMQ::_pause_simulation() const
{
    _check_client();
    _ZMQWrapper::get_sim()->pauseSimulation();

}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::stop_simulation stops the  CoppeliaSim simulation.
 */
void DQ_CoppeliaSimInterfaceZMQ::stop_simulation() const
{
    _check_client();
    _ZMQWrapper::get_sim()->stopSimulation();
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_stepping_mode enables or disables the stepping mode
 *        (formerly known as synchronous mode).
 * @param flag. Eg: set_stepping_mode(true)  // enables the stepping mode
 *                  set_stepping_mode(false)  // disables the stepping mode
 */
void DQ_CoppeliaSimInterfaceZMQ::set_stepping_mode(const bool &flag) const
{
    _check_client();
    _ZMQWrapper::get_sim()->setStepping(flag);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_simulation_time returns the simulation time.
 *        This time does not correspond to the real-time necessarily.
 * @return The simulation time.
 */
double DQ_CoppeliaSimInterfaceZMQ::_get_simulation_time() const
{
    _check_client();
    return _ZMQWrapper::get_sim()->getSimulationTime();
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::trigger_next_simulation_step This method sends a trigger
 *        signal to the CoppeliaSim scene, which performs a simulation step when the stepping mode is used.
 */
void DQ_CoppeliaSimInterfaceZMQ::trigger_next_simulation_step() const
{
    _check_client();
    _ZMQWrapper::get_sim()->step();
}



/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::is_simulation_running checks if the simulation is running.
 * @return True if the simulation is running. False otherwise.
 */
bool DQ_CoppeliaSimInterfaceZMQ::_is_simulation_running() const
{
    _check_client();
    return (_ZMQWrapper::get_sim()->getSimulationState() > _ZMQWrapper::get_sim()->simulation_paused);
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_simulation_state returns the simulation state
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
int DQ_CoppeliaSimInterfaceZMQ::_get_simulation_state() const
{
    _check_client();
    return _ZMQWrapper::get_sim()->getSimulationState();
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_status_bar_message sends a message to CoppeliaSim to be
 *        displayed in the status bar.
 *
 * @param message
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_status_bar_message(const std::string &message) const
{
    _check_client();
    __set_status_bar_message(message, _ZMQWrapper::get_sim()->verbosity_undecorated);
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_object_handle gets the object handle from
 *        CoppeliaSim.
 * @param objectname The name of the object in the CoppeliaSim scene.
 * @return the object handle.
 */
int DQ_CoppeliaSimInterfaceZMQ::get_object_handle(const std::string &objectname)
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
        handle = _ZMQWrapper::get_sim()->getObject(standard_objectname);
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
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_object_handles returns a vector containing the object handles.
 * @param objectnames The vector that contains the object names in the CoppeliaSim scene.
 * @return The desired vector of handles.
 */
std::vector<int> DQ_CoppeliaSimInterfaceZMQ::get_object_handles(const std::vector<std::string> &objectnames)
{
    int n = objectnames.size();
    std::vector<int> handles(n);
    for(auto i=0;i<n;i++)
        handles[i]=get_object_handle(objectnames[i]);

    return handles;
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_object_translation returns a pure quaternion that represents the position
 *        of an object in the CoppeliaSim scene with respect to the absolute frame.
 * @param handle The handle of the object.
 * @return The position of the handle.
 */
DQ DQ_CoppeliaSimInterfaceZMQ::_get_object_translation(const int &handle) const
{
    _check_client();
    auto position = _ZMQWrapper::get_sim()->getObjectPosition(handle, _ZMQWrapper::get_sim()->handle_world);
    const DQ t = DQ(0, position.at(0),position.at(1),position.at(2));
    return t;
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_object_translation returns a pure quaternion that represents the position
 *        of an object in the CoppeliaSim scene with respect to the absolute frame.
 * @param objectname The name of the object.
 * @return The position of the object.
 */
DQ DQ_CoppeliaSimInterfaceZMQ::get_object_translation(const std::string &objectname)
{
    return _get_object_translation(_get_handle_from_map(objectname));
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_object_translation sets the translation of an object
 *        in the CoppeliaSim scene..
 * @param handle the object handle
 * @param t The pure quaternion that represents the desired position with respect to the absolute frame..
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_object_translation(const int &handle, const DQ &t)
{
    VectorXd vec_t = t.vec3();
    std::vector<double> position = {vec_t[0], vec_t[1],vec_t[2]};
    _check_client();
    _ZMQWrapper::get_sim()->setObjectPosition(handle, position,_ZMQWrapper::get_sim()->handle_world);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_object_translation sets the translation of an object
 *        in the CoppeliaSim scene.
 * @param objectname the name of the object
 * @param t The pure quaternion that represents the desired position with respect to the absolute frame..
 */
void DQ_CoppeliaSimInterfaceZMQ::set_object_translation(const std::string &objectname, const DQ &t)
{
    _set_object_translation(_get_handle_from_map(objectname), t);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_object_rotation returns a unit quaternion that represents the rotation
 *        of an object in the CoppeliaSim scene with respect to the absolute frame.
 * @param handle the object handle
 * @return The object rotation.
 */
DQ DQ_CoppeliaSimInterfaceZMQ::_get_object_rotation(const int &handle) const
{
    _check_client();
    auto rotation = _ZMQWrapper::get_sim()->getObjectQuaternion(handle +
                                              _ZMQWrapper::get_sim()->handleflag_wxyzquat,
                                              _ZMQWrapper::get_sim()->handle_world);

    return DQ(rotation.at(0), rotation.at(1), rotation.at(2), rotation.at(3));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_object_rotation returns a unit quaternion that represents the rotation
 *        of an object in the CoppeliaSim scene with respect to the absolute frame.
 * @param objectname the name of the object.
 * @return The object rotation
 */
DQ DQ_CoppeliaSimInterfaceZMQ::get_object_rotation(const std::string &objectname)
{
    return _get_object_rotation(_get_handle_from_map(objectname));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_object_rotation sets the rotation of an object in the CoppeliaSim scene.
 * @param handle the object handle
 * @param r A unit quaternion that represents the desired rotation with respect to the absolute frame..
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_object_rotation(const int &handle, const DQ &r)
{

    VectorXd vec_r = r.vec4();
    std::vector<double> rotation= {vec_r[0], vec_r[1],vec_r[2], vec_r[3]};
    _check_client();
    _ZMQWrapper::get_sim()->setObjectQuaternion(handle + _ZMQWrapper::get_sim()->handleflag_wxyzquat, rotation, _ZMQWrapper::get_sim()->handle_world);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_object_rotation sets the rotation of an object in the CoppeliaSim scene.
 * @param objectname the name of the object
 * @param r A unit quaternion that represents the desired rotation with respect to the absolute frame.
 */
void DQ_CoppeliaSimInterfaceZMQ::set_object_rotation(const std::string &objectname, const DQ &r)
{
    _set_object_rotation(_get_handle_from_map(objectname), r);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_object_pose returns a unit dual quaternion that represents
 *        the object pose in the CoppeliaSim scene with respect to the absolute frame..
 * @param handle The object handle
 * @return The desired object pose.
 */
DQ DQ_CoppeliaSimInterfaceZMQ::_get_object_pose(const int &handle) const
{
    DQ t = _get_object_translation(handle);
    DQ r = _get_object_rotation(handle);
    return r + 0.5*E_*t*r;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_object_pose returns a unit dual quaternion that represents
 *        the object pose in the CoppeliaSim scene with respect to the absolute frame.
 * @param objectname The name of the object in the CoppeliaSim scene.
 * @return the desired object pose
 */
DQ DQ_CoppeliaSimInterfaceZMQ::get_object_pose(const std::string &objectname)
{
    return _get_object_pose(_get_handle_from_map(objectname));
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_object_pose sets the pose of an object in the CoppeliaSim scene.
 * @param handle the object handle
 * @param h A unit dual qualternion that represents the desired object pose with respect to the absolute frame.
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_object_pose(const int &handle, const DQ &h) const
{

    VectorXd vec_r = h.P().vec4();
    VectorXd vec_p = h.translation().vec3();
    std::vector<double> pose = {vec_p[0], vec_p[1],vec_p[2],vec_r[0], vec_r[1],vec_r[2], vec_r[3]};
    _check_client();
    _ZMQWrapper::get_sim()->setObjectPose(handle + _ZMQWrapper::get_sim()->handleflag_wxyzquat, pose, _ZMQWrapper::get_sim()->handle_world);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_object_pose sets the pose of an object in the CoppeliaSim scene.
 * @param objectname The name of the object.
 * @param h A unit dual qualternion that represents the desired object pose with respect to the absolute frame.
 */
void DQ_CoppeliaSimInterfaceZMQ::set_object_pose(const std::string &objectname, const DQ &h)
{
    // For C++20
    // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
    std::string function_name = {"DQ_CoppeliaSimInterface::set_object_pose"};
    if (!is_unit(h))
        _throw_runtime_error(function_name + ". The pose must be a unit dual quaternion!");
    int handle = _get_handle_from_map(objectname);
    _set_object_pose(handle, h);
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_joint_position gets the joint position in the CoppeliaSim scene
 * @param handle The joint handle
 * @return The joint position
 */
double DQ_CoppeliaSimInterfaceZMQ::_get_joint_position(const int &handle) const
{
    _check_client();
    return double(_ZMQWrapper::get_sim()->getJointPosition(handle));
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_joint_position gets the joint position in the CoppeliaSim scene
 * @param jointname the joint name
 * @return The joint position
 */
double DQ_CoppeliaSimInterfaceZMQ::_get_joint_position(const std::string &jointname)
{
    return _get_joint_position(_get_handle_from_map(jointname));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_joint_positions gets the joint positions in the CoppeliaSim scene
 * @param handles A vector containing the handles of the joints.
 * @return The joint positions
 */
VectorXd DQ_CoppeliaSimInterfaceZMQ::_get_joint_positions(const std::vector<int> &handles) const
{
    int n = handles.size();
    VectorXd joint_positions(n);
    for(auto i=0;i<n;i++)
        joint_positions(i)=_get_joint_position(handles.at(i));

    return joint_positions;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_joint_positions gets the joint positions in the CoppeliaSim scene.
 * @param jointnames A vector containing the names of the joints.
 * @return The joint positions
 */
VectorXd DQ_CoppeliaSimInterfaceZMQ::get_joint_positions(const std::vector<std::string> &jointnames)
{
    int n = jointnames.size();
    VectorXd joint_positions(n);
    for(auto i=0;i<n;i++)
        joint_positions(i)=_get_joint_position(jointnames[i]);

    return joint_positions;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_position sets the joint position in the CoppeliaSim scene
 * @param handle The joint handle
 * @param angle_rad The desired joint position
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_position(const int &handle, const double &angle_rad) const
{
    _check_client();
    _ZMQWrapper::get_sim()->setJointPosition(handle, angle_rad);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_position sets the joint position in the CoppeliaSim scene
 * @param jointname The joint name
 * @param angle_rad The desired joint position
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_position(const std::string &jointname, const double &angle_rad)
{
    _set_joint_position(_get_handle_from_map(jointname), angle_rad);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_positions sets the joint positions in the CoppeliaSim scene
 * @param handles A vector containing the joint handles
 * @param angles_rad The desired joint positions
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_positions(const std::vector<int> &handles, const VectorXd &angles_rad) const
{
    for(std::size_t i=0;i<handles.size();i++)
        _set_joint_position(handles.at(i), angles_rad(i));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_positions sets the joint positions in the CoppeliaSim scene
 * @param jointnames A vector containing the joint names.
 * @param angles_rad The desired joint positions.
 */
void DQ_CoppeliaSimInterfaceZMQ::set_joint_positions(const std::vector<std::string> &jointnames, const VectorXd &angles_rad)
{
     _check_sizes(jointnames, angles_rad, "Error in DQ_CoppeliaSimInterface::set_joint_positions: "
                                          "jointnames and angles_rad have incompatible sizes");
    for(std::size_t i=0;i<jointnames.size();i++)
         _set_joint_position(jointnames.at(i), angles_rad(i));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_target_position sets the joint target position
 * @param handle
 * @param angle_rad
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_target_position(const int &handle, const double &angle_rad) const
{
    _check_client();
    _ZMQWrapper::get_sim()->setJointTargetPosition(handle, angle_rad);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_target_position
 * @param jointname
 * @param angle_rad
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_target_position(const std::string &jointname, const double &angle_rad)
{
    _set_joint_target_position(_get_handle_from_map(jointname), angle_rad);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_target_positions
 * @param handles
 * @param angles_rad
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_target_positions(const std::vector<int> &handles, const VectorXd &angles_rad) const
{
    for(std::size_t i=0;i<handles.size();i++)
        _set_joint_target_position(handles.at(i), angles_rad(i));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_target_positions sets the joint target positions in the CoppeliaSim scene.
 *                      This method requires a dynamics enabled scene, and joints in dynamic mode with position control mode.
 *                      Check this link for more information about joint modes:
 *                      https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
 *
 * @param jointnames A vector containing the names of the joints.
 * @param angles_rad The desired joint positions.
 */
void DQ_CoppeliaSimInterfaceZMQ::set_joint_target_positions(const std::vector<std::string> &jointnames, const VectorXd &angles_rad)
{
    _check_sizes(jointnames, angles_rad, "Error in DQ_CoppeliaSimInterface::set_joint_target_positions: "
                                         "jointnames and angles_rad have incompatible sizes");
    for(std::size_t i=0;i<jointnames.size();i++)
        _set_joint_target_position(jointnames.at(i), angles_rad(i));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_joint_velocity
 * @param handle
 * @return
 */
double DQ_CoppeliaSimInterfaceZMQ::_get_joint_velocity(const int &handle) const
{
    _check_client();
    return _ZMQWrapper::get_sim()->getObjectFloatParam(handle, _ZMQWrapper::get_sim()->jointfloatparam_velocity);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_joint_velocity
 * @param jointname
 * @return
 */
double DQ_CoppeliaSimInterfaceZMQ::_get_joint_velocity(const std::string &jointname)
{
    return _get_joint_velocity(_get_handle_from_map(jointname));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_joint_velocties
 * @param handles
 * @return
 */
VectorXd DQ_CoppeliaSimInterfaceZMQ::_get_joint_velocities(const std::vector<int> &handles) const
{
    int n = handles.size();
    VectorXd joint_velocities(n);
    for(auto i=0;i<n;i++)
        joint_velocities(i)=_get_joint_velocity(handles.at(i));

    return joint_velocities;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_joint_velocities gets the joint velocities in the CoppeliaSim scene.
 * @param jointnames A vector containing the names of the joints.
 * @return The joint velocities
 */
VectorXd DQ_CoppeliaSimInterfaceZMQ::get_joint_velocities(const std::vector<std::string> &jointnames)
{
    int n = jointnames.size();
    VectorXd joint_velocities(n);
    for(auto i=0;i<n;i++)
        joint_velocities(i)=_get_joint_velocity(jointnames[i]);

    return joint_velocities;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_target_velocity
 * @param handle
 * @param angle_rad_dot
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_target_velocity(const int &handle, const double &angle_rad_dot) const
{
    _check_client();
    _ZMQWrapper::get_sim()->setJointTargetVelocity(handle, angle_rad_dot);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_target_velocity
 * @param jointname
 * @param angle_rad_dot
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_target_velocity(const std::string &jointname, const double &angle_rad_dot)
{
    _set_joint_target_velocity(_get_handle_from_map(jointname), angle_rad_dot);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_target_velocities
 * @param handles
 * @param angles_rad_dot
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_target_velocities(const std::vector<int> &handles, const VectorXd &angles_rad_dot) const
{
    for(std::size_t i=0;i<handles.size();i++)
        _set_joint_target_velocity(handles.at(i), angles_rad_dot(i));
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_target_velocities sets the joint target velocities in the CoppeliaSim scene.
 *                      This method requires a dynamics enabled scene, and joints in dynamic mode with velocity control mode.
 *                      Check this link for more information about joint modes:
 *                      https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
 * @param jointnames    A vector containing the names of the joints.
 * @param angles_rad_dot  The desired joint velocities.
 */
void DQ_CoppeliaSimInterfaceZMQ::set_joint_target_velocities(const std::vector<std::string> &jointnames, const VectorXd &angles_rad_dot)
{
    _check_sizes(jointnames, angles_rad_dot, "Error in DQ_CoppeliaSimInterface::set_joint_target_velocities: "
                                             "jointnames and angles_rad_Dot have incompatible sizes");
    for(std::size_t i=0;i<jointnames.size();i++)
        _set_joint_target_velocity(jointnames.at(i), angles_rad_dot(i));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_torque
 * @param handle
 * @param torque
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_torque(const int &handle, const double &torque) const
{
    _check_client();
    double angle_dot_rad_max = 10000.0;
    if (torque==0)
        angle_dot_rad_max = 0.0;
    else if (torque<0)
        angle_dot_rad_max = -10000.0;

    //simxSetJointTargetVelocity(clientid_,handle,angle_dot_rad_max,_remap_op_mode(opmode));
    //simxSetJointForce(clientid_,handle,abs(torque_f),_remap_op_mode(opmode));
    _ZMQWrapper::get_sim()->setJointTargetVelocity(handle, angle_dot_rad_max);
    _ZMQWrapper::get_sim()->setJointTargetForce(handle, abs(torque));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_torque
 * @param jointname
 * @param torque
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_torque(const std::string &jointname, const double &torque)
{
    _set_joint_torque(_get_handle_from_map(jointname), torque);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_torques
 * @param handles
 * @param torques
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_torques(const std::vector<int> &handles, const VectorXd &torques) const
{
    for(std::size_t i=0;i<handles.size();i++)
        _set_joint_torque(handles.at(i), torques(i));
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_torques sets the joint torques in the CoppeliaSim scene.
 *                      This method requires a dynamics enabled scene, and joints in dynamic mode with force control mode.
 *                      Check this link for more information about joint modes:
 *                      https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
 * @param jointnames A vector containing the names of the joints.
 * @param torques The desired joint torques.
 */
void DQ_CoppeliaSimInterfaceZMQ::set_joint_torques(const std::vector<std::string> &jointnames, const VectorXd &torques)
{
    _check_sizes(jointnames, torques, "Error in DQ_CoppeliaSimInterface::set_joint_torques: "
                                             "jointnames and torques have incompatible sizes");
    for(std::size_t i=0;i<jointnames.size();i++)
        _set_joint_torque(jointnames.at(i), torques(i));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_joint_torque
 * @param handle
 * @return
 */
double DQ_CoppeliaSimInterfaceZMQ::_get_joint_torque(const int &handle) const
{
    _check_client();
    return _ZMQWrapper::get_sim()->getJointForce(handle);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_joint_torque
 * @param jointname
 * @return
 */
double DQ_CoppeliaSimInterfaceZMQ::_get_joint_torque(const std::string &jointname)
{
    return _get_joint_torque(_get_handle_from_map(jointname));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_joint_torques
 * @param handles
 * @return
 */
VectorXd DQ_CoppeliaSimInterfaceZMQ::_get_joint_torques(const std::vector<int> &handles) const
{
    int n = handles.size();
    VectorXd joint_torques(n);
    for(auto i=0;i<n;i++)
        joint_torques(i)=_get_joint_torque(handles.at(i));

    return joint_torques;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_joint_torques gets the joint torques in the CoppeliaSim scene.
 * @param jointnames A vector containing the names of the joints.
 * @return the joint torques.
 */
VectorXd DQ_CoppeliaSimInterfaceZMQ::get_joint_torques(const std::vector<std::string> &jointnames)
{
    int n = jointnames.size();
    VectorXd joint_torques(n);
    for(auto i=0;i<n;i++)
        joint_torques(i)=_get_joint_torque(jointnames[i]);

    return joint_torques;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_object_name gets the name of an object in the CoppeliaSim scene
 * @param handle the object handle.
 * @return The object name
 */
std::string DQ_CoppeliaSimInterfaceZMQ::_get_object_name(const int &handle)
{
    _check_client();
    std::string objectname = _ZMQWrapper::get_sim()->getObjectAlias(handle, 1);
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
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_object_names
 * @param handles
 * @return
 */
template<typename T>
std::vector<std::string> DQ_CoppeliaSimInterfaceZMQ::_get_object_names(const T &handles)
{
    int n = handles.size();
    std::vector<std::string> objectnames(n);
    for(auto i=0;i<n;i++)
        objectnames.at(i)=_get_object_name(handles.at(i));

    return objectnames;
}
//-----------------------------------------------------------------------------------------

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_jointnames_from_parent_object returns a vector containing all the joint names
 *                                  in which a specified object is its parent.
 * @param parent_objectname The name of the object on CoppeliaSim that is the parent of the desired joints.
 * @return a vector containing the desired joint names
 */
std::vector<std::string> DQ_CoppeliaSimInterfaceZMQ::_get_jointnames_from_parent_object(const std::string &parent_objectname)
{
    int base_handle = _get_handle_from_map(parent_objectname);
    _check_client();
    std::vector<int64_t> jointhandles = _ZMQWrapper::get_sim()->getObjectsInTree(base_handle,
                                        _ZMQWrapper::get_sim()->object_joint_type,
                                        0);
    return _get_object_names(jointhandles);

}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_shapenames_from_parent_object returns a vector containing all the shape names
 *                                  in which a specified object is its parent.
 * @param parent_objectname The name of the object on CoppeliaSim that is the parent of the desired shapes.
 * @param shape_type
 * @return a vector containing the desired shape names
 */
std::vector<std::string> DQ_CoppeliaSimInterfaceZMQ::_get_shapenames_from_parent_object(const std::string &parent_objectname,
                                                                                    const SHAPE_TYPE &shape_type)
{
    int base_handle = _get_handle_from_map(parent_objectname);
    _check_client();
    std::vector<int64_t> shapehandles = _ZMQWrapper::get_sim()->getObjectsInTree(base_handle,
                                                               _ZMQWrapper::get_sim()->object_shape_type,
                                                               0);
    size_t handlesizes = shapehandles.size();
    std::vector<int64_t> dynamic_features(handlesizes, 0);

    for (size_t i=0; i<handlesizes;i++) // 0 is dynamic.  1 is static
        dynamic_features.at(i) = _ZMQWrapper::get_sim()->getObjectInt32Param(shapehandles.at(i), _ZMQWrapper::get_sim()->shapeintparam_static);

    std::vector<int64_t> aux_shapehandles;
    switch (shape_type) {
    case SHAPE_TYPE::DYNAMIC:
        for (size_t i=0; i<handlesizes; i++)
            if (dynamic_features.at(i) == 0)
                aux_shapehandles.push_back(shapehandles.at(i));
        return _get_object_names(aux_shapehandles);
    case SHAPE_TYPE::STATIC:
        for (size_t i=0; i<handlesizes; i++)
            if (dynamic_features.at(i) == 1)
                aux_shapehandles.push_back(shapehandles.at(i));
        return _get_object_names(aux_shapehandles);
    case SHAPE_TYPE::ANY:
        return _get_object_names(shapehandles);
    default: // This line is required in GNU/Linux
        _throw_runtime_error("wrong argument!");
    }


}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_angular_and_linear_velocities returns the angular a linear velocities of an
 *                  object in the CoppeliaSim scene
 * @param handle The object handle.
 * @param reference The reference frame
 * @return A vector containg the angular and linear velocities. [wx wy wz x_dot y_dot z_dot]
 */
VectorXd DQ_CoppeliaSimInterfaceZMQ::_get_angular_and_linear_velocities(const int &handle, const REFERENCE &reference) const
{
    std::vector<int> params = _get_velocity_const_params();
    VectorXd v = VectorXd::Zero(params.size());
    _check_client();
    for (size_t i=0; i < params.size(); i++)
    {
        v(i) = _ZMQWrapper::get_sim()->getObjectFloatParam(handle, params.at(i));
    }
    if (reference == REFERENCE::BODY_FRAME)
    {
        DQ x = _get_object_pose(handle);
        DQ r = x.P();
        DQ w_b = r.conj()*DQ(v.head(3))*r;
        DQ p_dot_b = r.conj()*DQ(v.tail(3))*r;
        v.head(3) = w_b.vec3();
        v.tail(3) = p_dot_b.vec3();
    }
    return v;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_angular_and_linear_velocities returns the angular a linear velocities of an
 *                  object in the CoppeliaSim scene.
 * @param objectname The object name
 * @param reference The reference frame.
 * @return A vector containg the angular and linear velocities. [wx wy wz x_dot y_dot z_dot]
 */
VectorXd DQ_CoppeliaSimInterfaceZMQ::_get_angular_and_linear_velocities(std::string &objectname, const REFERENCE &reference)
{
    return _get_angular_and_linear_velocities(_get_handle_from_map(objectname), reference);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_angular_and_linear_velocities
 * @param handle
 * @param w
 * @param p_dot
 * @param reference
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_angular_and_linear_velocities(const int &handle, const DQ &w, const DQ &p_dot, const REFERENCE &reference) const
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
        DQ x = _get_object_pose(handle);
        DQ r = x.P();
        DQ w_a = r*w*r.conj();
        DQ p_dot_a = r*p_dot*r.conj();
        v.head(3) = w_a.vec3();
        v.tail(3) = p_dot_a.vec3();
    }
    _check_client();
    _ZMQWrapper::get_sim()->resetDynamicObject(handle);
    for (size_t i=0; i < params.size(); i++)
    {
        _ZMQWrapper::get_sim()->setObjectFloatParam(handle, params.at(i), v(i));
    }
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_angular_and_linear_velocities
 * @param objectname
 * @param w
 * @param p_dot
 * @param reference
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_angular_and_linear_velocities(std::string &objectname, const DQ &w, const DQ &p_dot, const REFERENCE &reference)
{
    _set_angular_and_linear_velocities(_get_handle_from_map(objectname), w, p_dot, reference);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_twist
 * @param handle
 * @param reference
 * @return
 */
DQ DQ_CoppeliaSimInterfaceZMQ::_get_twist(const int &handle, const REFERENCE &reference) const
{
    VectorXd v = _get_angular_and_linear_velocities(handle);
    DQ w = DQ(v.head(3));
    DQ p_dot = DQ(v.tail(3));
    DQ x = _get_object_pose(handle);
    DQ twist =  w + E_*(p_dot + cross(x.translation(), w));;
    if (reference == REFERENCE::BODY_FRAME)
    {
       twist =  x.conj()*twist*x;
    }
    return twist;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_twist
 * @param objectname
 * @param reference
 * @return
 */
DQ DQ_CoppeliaSimInterfaceZMQ::_get_twist(const std::string &objectname, const REFERENCE &reference)
{
    return _get_twist(_get_handle_from_map(objectname), reference);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_twist
 * @param handle
 * @param twist
 * @param reference
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_twist(const int &handle, const DQ& twist, const REFERENCE &reference) const
{
    if (!is_pure(twist))
    {
        throw(std::range_error("Bad set_object_twist() call: Not a pure dual quaternion"));
    }
    if (reference == REFERENCE::BODY_FRAME)
    {
        _set_angular_and_linear_velocities(handle, twist.P(), twist.D(),
                                          REFERENCE::BODY_FRAME);
    }
    else{
        DQ x = _get_object_pose(handle);
        _set_angular_and_linear_velocities(handle, twist.P(), twist.D()-cross(x.translation(), twist.P()),
                                          REFERENCE::ABSOLUTE_FRAME);
    }
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_twist
 * @param objectname
 * @param twist
 * @param reference
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_twist(const std::string &objectname, const DQ &twist, const REFERENCE &reference)
{
    _set_twist(_get_handle_from_map(objectname), twist, reference);
}




/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_mode
 * @param jointname
 * @param joint_mode
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_mode(const std::string &jointname, const JOINT_MODE &joint_mode)
{
    _check_client();
    int jointMode;
    switch (joint_mode)
        {
        case JOINT_MODE::KINEMATIC:
                    jointMode = _ZMQWrapper::get_sim()->jointmode_kinematic;
                    break;
        case JOINT_MODE::DYNAMIC:
                    jointMode = _ZMQWrapper::get_sim()->jointmode_dynamic;
                    break;
        case JOINT_MODE::DEPENDENT:
                    jointMode = _ZMQWrapper::get_sim()->jointmode_dependent;
                    break;
        }
        _ZMQWrapper::get_sim()->setJointMode(_get_handle_from_map(jointname), jointMode, 0);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_modes
 * @param jointnames
 * @param joint_mode
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_modes(const std::vector<std::string> &jointnames, const JOINT_MODE &joint_mode)
{
    for(std::size_t i=0;i<jointnames.size();i++)
        _set_joint_mode(jointnames.at(i), joint_mode);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_control_mode
 * @param jointname
 * @param joint_control_mode
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_control_mode(const std::string &jointname, const JOINT_CONTROL_MODE &joint_control_mode)
{
    _check_client();
    int64_t control_mode;
    switch (joint_control_mode)
    {
    case JOINT_CONTROL_MODE::FREE:
        control_mode = _ZMQWrapper::get_sim()->jointdynctrl_free;
        break;
    case JOINT_CONTROL_MODE::FORCE:
        control_mode = _ZMQWrapper::get_sim()->jointdynctrl_force;
        break;
    case JOINT_CONTROL_MODE::VELOCITY:
        control_mode = _ZMQWrapper::get_sim()->jointdynctrl_velocity;
        break;
    case JOINT_CONTROL_MODE::POSITION:
        control_mode = _ZMQWrapper::get_sim()->jointdynctrl_position;
        break;
    case JOINT_CONTROL_MODE::SPRING:
        control_mode = _ZMQWrapper::get_sim()->jointdynctrl_spring;
        break;
    case JOINT_CONTROL_MODE::CUSTOM:
        control_mode = _ZMQWrapper::get_sim()->jointdynctrl_callback;
        break;
    case JOINT_CONTROL_MODE::TORQUE:
        control_mode = _ZMQWrapper::get_sim()->jointdynctrl_velocity;
        break;
    }
    _ZMQWrapper::get_sim()->setObjectInt32Param(_get_handle_from_map(jointname),
                              _ZMQWrapper::get_sim()->jointintparam_dynctrlmode,
                              control_mode);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_joint_control_modes
 * @param jointnames
 * @param joint_control_mode
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_joint_control_modes(const std::vector<std::string> &jointnames, const JOINT_CONTROL_MODE &joint_control_mode)
{
    for(std::size_t i=0;i<jointnames.size();i++)
    {
        _set_joint_control_mode(jointnames.at(i), joint_control_mode);
    }
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::enable_dynamics_engine
 * @param flag
 */
void DQ_CoppeliaSimInterfaceZMQ::_enable_dynamics(const bool &flag)
{
   _check_client();
   _ZMQWrapper::get_sim()->setBoolParam(_ZMQWrapper::get_sim()->boolparam_dynamics_handling_enabled, flag);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_simulation_time_step
 * @return
 */
double DQ_CoppeliaSimInterfaceZMQ::_get_simulation_time_step() const
{
    _check_client();
    return _ZMQWrapper::get_sim()->getFloatParam(_ZMQWrapper::get_sim()->floatparam_simulation_time_step);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_simulation_time_step
 * @param time_step
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_simulation_time_step(const double &time_step)
{
    _check_client();
    _ZMQWrapper::get_sim()->setFloatParam(_ZMQWrapper::get_sim()->floatparam_simulation_time_step, time_step);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_physics_time_step
 * @return
 */
double DQ_CoppeliaSimInterfaceZMQ::_get_physics_time_step() const
{
    _check_client();
    return _ZMQWrapper::get_sim()->getFloatParam(_ZMQWrapper::get_sim()->floatparam_physicstimestep);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_physics_time_step
 * @param time_step
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_physics_time_step(const double &time_step) const
{
    _check_client();
    _ZMQWrapper::get_sim()->setFloatParam(_ZMQWrapper::get_sim()->floatparam_physicstimestep, time_step);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_dynamic_engine
 * @param engine
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_engine(const ENGINE &engine)
{
    _check_client();
    _ZMQWrapper::get_sim()->setInt32Param(_ZMQWrapper::get_sim()->intparam_dynamic_engine, engines_.at(engine));
}

std::string DQ_CoppeliaSimInterfaceZMQ::get_engine()
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
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_gravity
 * @param gravity
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_gravity(const DQ &gravity)
{
    VectorXd gravity_vec = gravity.vec3();
    std::vector<double> g = {gravity_vec(0),gravity_vec(1), gravity_vec(2)};
    _check_client();
    _ZMQWrapper::get_sim()->setArrayParam(_ZMQWrapper::get_sim()->arrayparam_gravity, g);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_gravity
 * @return
 */
DQ DQ_CoppeliaSimInterfaceZMQ::_get_gravity() const
{
    _check_client();
    std::vector<double> g = _ZMQWrapper::get_sim()->getArrayParam(_ZMQWrapper::get_sim()->arrayparam_gravity);
    return DQ(0, g.at(0), g.at(1), g.at(2));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::load_scene loads a scene from your computer.
 * @param path_to_filename the path to the scene. This string must containt
 *        the file extension.
 *
 *        Example:
 *
 *        load_scene("/Users/juanjqo/git/space_robot/scenes/space_robot.ttt");
 */
void DQ_CoppeliaSimInterfaceZMQ::_load_scene(const std::string &path_to_filename) const
{
    _check_client();
    _ZMQWrapper::get_sim()->loadScene(path_to_filename);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::save_scene saves the current scene.
 * @param path_to_filename The path where you want to save the scene including
 *        the name of the scene and its file extension.
 *
 *        Example:
 *
 *        save_scene("/Users/juanjqo/git/space_robot/scenes/space_robot2.ttt");
 */
void DQ_CoppeliaSimInterfaceZMQ::_save_scene(const std::string &path_to_filename) const
{
    _check_client();
    _ZMQWrapper::get_sim()->saveScene(path_to_filename);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::close_scene closes the current scene.
 */
void DQ_CoppeliaSimInterfaceZMQ::_close_scene() const
{
    _check_client();
    _ZMQWrapper::get_sim()->closeScene();
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::remove_child_script_from_object
 *        The script must be located at objectname/script_name
 * @param objectname
 * @param script_name
 */
void DQ_CoppeliaSimInterfaceZMQ::_remove_child_script_from_object(const std::string &objectname, const std::string &script_name)
{
    _check_client();
    if (_object_exist_on_scene(_get_standard_name(objectname)+script_name))
    {
        int handle = _get_handle_from_map(_get_standard_name(objectname)+script_name);
        _ZMQWrapper::get_sim()->removeObjects({handle}, false);
    }
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::object_exist_on_scene
 * @param objectname
 * @return
 */
bool DQ_CoppeliaSimInterfaceZMQ::_object_exist_on_scene(const std::string &objectname) const
{
    std::optional<json> options = {{"noError", false}};
    try {
        _check_client();
        auto rtn = _ZMQWrapper::get_sim()->getObject(_get_standard_name(objectname), options);
        return (rtn != -1) ? true : false;
    } catch (...) {
        return false;
    }

}

void DQ_CoppeliaSimInterfaceZMQ::_set_object_name(const int &handle, const std::string &new_object_name) const
{
    _check_client();
    _ZMQWrapper::get_sim()->setObjectAlias(handle, new_object_name);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_object_name
 * @param current_object_name
 * @param new_object_name
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_object_name(const std::string &current_object_name, const std::string &new_object_name)
{
    _set_object_name(_get_handle_from_map(current_object_name), new_object_name);
}


void DQ_CoppeliaSimInterfaceZMQ::_set_object_color(const int &handle,
                                               const std::vector<double> &rgba_color) const
{
    _check_client();
    _ZMQWrapper::get_sim()->setShapeColor(handle, "", _ZMQWrapper::get_sim()->colorcomponent_ambient_diffuse, {rgba_color.at(0), rgba_color.at(1),rgba_color.at(2)});
    _ZMQWrapper::get_sim()->setShapeColor(handle, "", _ZMQWrapper::get_sim()->colorcomponent_transparency, {rgba_color.at(3)});
}

void DQ_CoppeliaSimInterfaceZMQ::_set_object_color(const std::string &objectname, const std::vector<double>& rgba_color)
{
    // For C++20
    // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
    std::string function_name = {"DQ_CoppeliaSimInterface::set_object_color"};
    if (rgba_color.size() != 4)
        _throw_runtime_error(function_name + ". The rgba_color must be a vector of size 4.");

    _set_object_color(_get_handle_from_map(objectname), rgba_color);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_object_as_respondable(const int &handle, const bool &respondable_object) const
{
    _check_client();
    _ZMQWrapper::get_sim()->setObjectInt32Param(handle,
                              _ZMQWrapper::get_sim()->shapeintparam_respondable,
                              (respondable_object == true ? 1 : 0));
}


void DQ_CoppeliaSimInterfaceZMQ::_set_object_as_respondable(const std::string &objectname, const bool &respondable_object)
{
    _set_object_as_respondable(_get_handle_from_map(objectname), respondable_object);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_object_as_static(const int &handle, const bool &static_object) const
{
    _check_client();
    _ZMQWrapper::get_sim()->setObjectInt32Param(handle,
                              _ZMQWrapper::get_sim()->shapeintparam_static,
                              (static_object == true ? 1 : 0));
}

void DQ_CoppeliaSimInterfaceZMQ::_set_object_as_static(const std::string &objectname, const bool &static_object)
{
    _set_object_as_static(_get_handle_from_map(objectname), static_object);
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_object_parent
 * @param handle
 * @param parent_handle
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_object_parent(const int &handle, const int &parent_handle, const bool &move_child_to_parent_pose) const
{
    _check_client();
    _ZMQWrapper::get_sim()->setObjectParent(handle, parent_handle, !move_child_to_parent_pose);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_object_parent
 * @param objectname
 * @param parent_object_name
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_object_parent(const std::string &objectname,
                                                const std::string &parent_object_name,
                                                const bool& move_child_to_parent_pose)
{
    _set_object_parent(_get_handle_from_map(objectname), _get_handle_from_map(parent_object_name), move_child_to_parent_pose);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::check_collision
 * @param handle1
 * @param handle2
 * @return
 */
bool DQ_CoppeliaSimInterfaceZMQ::_check_collision(const int &handle1, const int &handle2) const
{
    _check_client();
    auto [result, collidingObjectHandles] = _ZMQWrapper::get_sim()->checkCollision(handle1, handle2);
    return result;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::check_collision
 * @param objectname1
 * @param objectname2
 * @return
 */
bool DQ_CoppeliaSimInterfaceZMQ::_check_collision(const std::string &objectname1, const std::string &objectname2)
{
    return _check_collision(_get_handle_from_map(objectname1), _get_handle_from_map(objectname2));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::check_distance
 * @param handle1
 * @param handle2
 * @param threshold
 * @return
 */
std::tuple<double, DQ, DQ> DQ_CoppeliaSimInterfaceZMQ::_check_distance(const int &handle1, const int &handle2, const double &threshold) const
{
    _check_client();
    auto [result, data, objectHandlePair] = _ZMQWrapper::get_sim()->checkDistance(handle1, handle2, threshold);
         //[obj1X obj1Y obj1Z obj2X obj2Y obj2Z dist]
    DQ point1 = DQ(0, data.at(0), data.at(1), data.at(2));
    DQ point2 = DQ(0, data.at(3), data.at(4), data.at(5));
    double distance = data.at(6);
    return {distance, point1, point2};
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::check_distance
 * @param objectname1
 * @param objectname2
 * @param threshold
 * @return
 */
std::tuple<double, DQ, DQ> DQ_CoppeliaSimInterfaceZMQ::_check_distance(const std::string &objectname1, const std::string &objectname2, const double &threshold)
{
    return _check_distance(_get_handle_from_map(objectname1), _get_handle_from_map(objectname2), threshold);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::compute_distance
 * @param handle1
 * @param handle2
 * @param threshold
 * @return
 */
double DQ_CoppeliaSimInterfaceZMQ::_compute_distance(const int &handle1, const int &handle2, const double &threshold) const
{
    return std::get<0>(_check_distance(handle1, handle2, threshold));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::compute_distance
 * @param objectname1
 * @param objectname2
 * @param threshold
 * @return
 */
double DQ_CoppeliaSimInterfaceZMQ::_compute_distance(const std::string &objectname1, const std::string &objectname2, const double &threshold)
{
    return _compute_distance(_get_handle_from_map(objectname1), _get_handle_from_map(objectname2), threshold);
}




void DQ_CoppeliaSimInterfaceZMQ::_create_plane(const std::string &name, const std::vector<double> &sizes, const std::vector<double> &rgba_color, const bool &add_normal, const double &normal_scale) const
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
        _create_static_axis_at_origin(primitive_handle, name, scaled_size, AXIS::k, 1);
    }
    _merge_shapes(primitive_handle);
}



void DQ_CoppeliaSimInterfaceZMQ::_create_line(const std::string &name, const std::vector<double> &thickness_and_length, const std::vector<double> &rgba_color, const bool &add_arrow, const double &arrow_scale) const
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




void DQ_CoppeliaSimInterfaceZMQ::_create_cylinder(const std::string &name, const std::vector<double> &width_and_length, const std::vector<double> &rgba_color, const bool &add_line, const double &line_scale) const
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
 * @brief DQ_CoppeliaSimInterfaceZMQ::_merge_shapes
 * @param parent_handle
 */
void DQ_CoppeliaSimInterfaceZMQ::_merge_shapes(const int &parent_handle) const
{
    std::vector<int64_t> shapehandles = _ZMQWrapper::get_sim()->getObjectsInTree(parent_handle, //_get_handle_from_map(name),
                                                               _ZMQWrapper::get_sim()->object_shape_type,
                                                               0);
    std::reverse(shapehandles.begin(), shapehandles.end());
    _ZMQWrapper::get_sim()->groupShapes(shapehandles, false);
}



void DQ_CoppeliaSimInterfaceZMQ::_create_reference_frame(const std::string &name, const double &scale, const std::vector<double> &thickness_and_length) const
{
    int primitive_handle = _add_primitive(PRIMITIVE::SPHEROID, name,
                                         {1.5*scale*thickness_and_length.at(0), 1.5*scale*thickness_and_length.at(0), 1.5*scale*thickness_and_length.at(0)});
    _set_object_color(primitive_handle, {1,1,1,0.5});
    _set_object_as_respondable(primitive_handle, false);
    _set_object_as_static(primitive_handle, true);
    std::vector<std::string> children_names;
    std::vector<std::string> auxdest;

    std::vector<double> scaled_size = {scale*thickness_and_length.at(0),scale*thickness_and_length.at(0), scale*thickness_and_length.at(1)};
    _create_static_axis_at_origin(primitive_handle, name, scaled_size, AXIS::k, 1);
    _create_static_axis_at_origin(primitive_handle, name, scaled_size, AXIS::i, 1);
    _create_static_axis_at_origin(primitive_handle, name, scaled_size, AXIS::j, 1);
    _merge_shapes(primitive_handle);
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::draw_trajectory
 * @param point
 * @param size
 * @param color
 * @param max_item_count
 */
void DQ_CoppeliaSimInterfaceZMQ::_draw_permanent_trajectory(const DQ &point, const double &size, const std::vector<double> &color, const int &max_item_count)
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
    auto drawn_handle = _ZMQWrapper::get_sim()->addDrawingObject(_ZMQWrapper::get_sim()->drawing_lines+_ZMQWrapper::get_sim()->drawing_cyclic,size,0,-1,max_item_count, color);
    _ZMQWrapper::get_sim()->addDrawingObjectItem(
        drawn_handle,
        itemdata
        );
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::add_simulation_lua_script
 * @param script_name
 * @param script_code
 * @return
 */
int DQ_CoppeliaSimInterfaceZMQ::_add_simulation_lua_script(const std::string &script_name, const std::string& script_code)
{
    _check_client();
    int scriptHandle = _ZMQWrapper::get_sim()->createScript(_ZMQWrapper::get_sim()->scripttype_simulation,
                                          script_code, 0, "lua");
    _set_object_name(scriptHandle, _remove_first_slash_from_string(script_name));
    return scriptHandle;
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::draw_trajectory
 * @param objectname
 * @param size
 * @param rgb_color
 * @param max_item_count
 */
void DQ_CoppeliaSimInterfaceZMQ::_draw_trajectory(const std::string &objectname,
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
 * @brief DQ_CoppeliaSimInterfaceZMQ::remove_object
 * @param objectname
 * @param remove_children
 */
void DQ_CoppeliaSimInterfaceZMQ::_remove_object(const std::string& objectname, const bool &remove_children)
{
    _check_client();
    auto standard_objectname = _get_standard_name(objectname);
    auto handle = _get_handle_from_map(standard_objectname);

    if (remove_children)
    {
        auto handles = _ZMQWrapper::get_sim()->getObjectsInTree(handle, _ZMQWrapper::get_sim()->handle_all, 0);
        if (handles.size() > 0)
        {
            auto objectnames = _get_object_names(handles);
            _ZMQWrapper::get_sim()->removeObjects(handles, false);
            for (std::size_t i=0; i<objectnames.size();i++)
                _update_map(_get_standard_name(objectnames.at(i)), handles.at(i), UPDATE_MAP::REMOVE);
        }
    }else
    {
        _ZMQWrapper::get_sim()->removeObjects({handle}, false);
        _update_map(standard_objectname, handle, UPDATE_MAP::REMOVE);
    }
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_bounding_box_size
 * @param handle
 * @return
 */
std::vector<double> DQ_CoppeliaSimInterfaceZMQ::_get_bounding_box_size(const int &handle) const
{
    _check_client();
    auto [size, pose] = _ZMQWrapper::get_sim()->getShapeBB(handle);
    return size;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_bounding_box_size
 * @param objectname
 * @return
 */
std::vector<double> DQ_CoppeliaSimInterfaceZMQ::_get_bounding_box_size(const std::string &objectname)
{
    return _get_bounding_box_size(_get_handle_from_map(objectname));
}




bool DQ_CoppeliaSimInterfaceZMQ::_mujoco_is_used()
{
    return _get_engine() == ENGINE::MUJOCO ? true : false;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_mujoco_global_impratio This attribute determines the ratio of
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
void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_impratio(const double &impratio)
{
    _ZMQWrapper::get_sim()->setEngineFloatParam(_ZMQWrapper::get_sim()->mujoco_global_impratio,-1, impratio);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_mujoco_global_wind Velocity vector of the medium (i.e., wind).
 *                      This vector is subtracted from the 3D translational velocity of each body,
 *                      and the result is used to compute viscous, lift and drag forces acting on the body;
 * @param wind
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_wind(const std::vector<double> &wind)
{
    std::vector<int64_t> mujoco_global_param= {_ZMQWrapper::get_sim()->mujoco_global_wind1,
                                               _ZMQWrapper::get_sim()->mujoco_global_wind2,
                                               _ZMQWrapper::get_sim()->mujoco_global_wind3};
    _check_sizes(wind, mujoco_global_param, "Error in DQ_CoppeliaSimInterface::set_mujoco_global_wind: "
                                            "argument must be a vector of size "+std::to_string(mujoco_global_param.size()));
    for (size_t i=0;i<wind.size();i++)
        _ZMQWrapper::get_sim()->setEngineFloatParam(mujoco_global_param.at(i),-1, wind.at(i));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_mujoco_global_density Density of the medium, not to be confused with the geom density used to infer masses and inertias.
 *                                 This parameter is used to simulate lift and drag forces, which scale quadratically with velocity.
 *                                 In SI units the density of air is around 1.2 while the density of water is around 1000 depending on temperature. Setting density to 0 disables lift and drag forces.
 * @param density
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_density(const double &density)
{
    _ZMQWrapper::get_sim()->setEngineFloatParam(_ZMQWrapper::get_sim()->mujoco_global_density,-1, density);
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_mujoco_global_viscosity Viscosity of the medium. This parameter is used to simulate viscous forces,
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
void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_viscosity(const double &viscosity)
{
    _ZMQWrapper::get_sim()->setEngineFloatParam(_ZMQWrapper::get_sim()->mujoco_global_viscosity,-1, viscosity);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_boundmass(const double &boundmass)
{
    _ZMQWrapper::get_sim()->setEngineFloatParam(_ZMQWrapper::get_sim()->mujoco_global_boundmass,-1, boundmass);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_boundinertia(const double &boundinertia)
{
    _ZMQWrapper::get_sim()->setEngineFloatParam(_ZMQWrapper::get_sim()->mujoco_global_boundinertia,-1, boundinertia);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_overridemargin(const double &overridemargin)
{
    _ZMQWrapper::get_sim()->setEngineFloatParam(_ZMQWrapper::get_sim()->mujoco_global_overridemargin,-1, overridemargin);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_overridesolref(const std::vector<double> &overridesolref)
{
    std::vector<int64_t> mujoco_global_param = {_ZMQWrapper::get_sim()->mujoco_global_overridesolref1,
                                                _ZMQWrapper::get_sim()->mujoco_global_overridesolref2};
    _check_sizes(overridesolref, mujoco_global_param, "Error in DQ_CoppeliaSimInterface::set_mujoco_global_overridesolref: "
                                                      "argument must be a vector of size "+std::to_string(mujoco_global_param.size()));
    for (size_t i=0;i<overridesolref.size();i++)
        _ZMQWrapper::get_sim()->setEngineFloatParam(mujoco_global_param.at(i),-1, overridesolref.at(i));
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_overridesolimp(const std::vector<double> &overridesolimp)
{
    std::vector<int64_t> mujoco_global_param = {_ZMQWrapper::get_sim()->mujoco_global_overridesolimp1,
                                                _ZMQWrapper::get_sim()->mujoco_global_overridesolimp2,
                                                _ZMQWrapper::get_sim()->mujoco_global_overridesolimp3,
                                                _ZMQWrapper::get_sim()->mujoco_global_overridesolimp4,
                                                _ZMQWrapper::get_sim()->mujoco_global_overridesolimp5,
                                                };
    _check_sizes(overridesolimp, mujoco_global_param, "Error in DQ_CoppeliaSimInterface::set_mujoco_global_overridesolimp: "
                                                      "argument must be a vector of size "+std::to_string(mujoco_global_param.size()));
    for (size_t i=0;i<overridesolimp.size();i++)
        _ZMQWrapper::get_sim()->setEngineFloatParam(mujoco_global_param.at(i),-1, overridesolimp.at(i));
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_iterations(const int &iterations)
{
    _ZMQWrapper::get_sim()->setEngineInt32Param(_ZMQWrapper::get_sim()->mujoco_global_iterations,-1, iterations);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_integrator(const int &integrator)
{
    _ZMQWrapper::get_sim()->setEngineInt32Param(_ZMQWrapper::get_sim()->mujoco_global_integrator,-1, integrator);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_solver(const int &solver)
{
    _ZMQWrapper::get_sim()->setEngineInt32Param(_ZMQWrapper::get_sim()->mujoco_global_solver,-1, solver);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_njmax(const int &njmax)
{
    _ZMQWrapper::get_sim()->setEngineInt32Param(_ZMQWrapper::get_sim()->mujoco_global_njmax,-1, njmax);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_nstack(const int &nstack)
{
    _ZMQWrapper::get_sim()->setEngineInt32Param(_ZMQWrapper::get_sim()->mujoco_global_nstack,-1, nstack);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_nconmax(const int &nconmax)
{
    _ZMQWrapper::get_sim()->setEngineInt32Param(_ZMQWrapper::get_sim()->mujoco_global_nconmax,-1, nconmax);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_cone(const int &cone)
{
    _ZMQWrapper::get_sim()->setEngineInt32Param(_ZMQWrapper::get_sim()->mujoco_global_cone,-1, cone);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_overridekin(const int &overridekin)
{
    _ZMQWrapper::get_sim()->setEngineInt32Param(_ZMQWrapper::get_sim()->mujoco_global_overridekin,-1, overridekin);
}

/*
void DQ_CoppeliaSimInterfaceZMQ::set_mujoco_global_rebuildcondition(const int &rebuildcondition)
{
   sim_->setEngineInt32Param(sim_->mujoco_global_rebuildcondition,-1, rebuildcondition);
}
*/

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_computeinertias(const bool &computeinertias)
{
   _ZMQWrapper::get_sim()->setEngineBoolParam(_ZMQWrapper::get_sim()->mujoco_global_computeinertias,-1, computeinertias);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_multithreaded(const bool &multithreaded)
{
   _ZMQWrapper::get_sim()->setEngineBoolParam(_ZMQWrapper::get_sim()->mujoco_global_multithreaded,-1, multithreaded);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_multiccd(const bool &multiccd)
{
   _ZMQWrapper::get_sim()->setEngineBoolParam(_ZMQWrapper::get_sim()->mujoco_global_multiccd,-1, multiccd);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_balanceinertias(const bool &balanceinertias)
{
   _ZMQWrapper::get_sim()->setEngineBoolParam(_ZMQWrapper::get_sim()->mujoco_global_balanceinertias,-1, balanceinertias);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_global_overridecontacts(const bool &overridecontacts)
{
    _ZMQWrapper::get_sim()->setEngineBoolParam(_ZMQWrapper::get_sim()->mujoco_global_overridecontacts,-1, overridecontacts);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_mujoco_joint_stiffness
 *                         Joint stiffness.
 *                         If this value is positive, a spring will be created with equilibrium position
 *                         given by springref below.
 *                         The spring force is computed along with the other passive forces.
 * @param jointname
 * @param stiffness
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_joint_stiffness(const std::string &jointname, const double &stiffness)
{
   _ZMQWrapper::get_sim()->setEngineFloatParam(_ZMQWrapper::get_sim()->mujoco_joint_stiffness,_get_handle_from_map(jointname), stiffness);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_mujoco_joint_stiffness
 *                         Joint stiffness.
 *                         If this value is positive, a spring will be created with equilibrium position
 *                         given by springref below.
 *                         The spring force is computed along with the other passive forces.
 * @param jointnames
 * @param stiffness
 */
void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_joint_stiffnesses(const std::vector<std::string> &jointnames,
                                                           const double &stiffness)
{
    for (size_t i=0;i<jointnames.size();i++)
        _set_mujoco_joint_stiffness(jointnames.at(i), stiffness);

}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_joint_damping(const std::string &jointname, const double &damping)
{
    _ZMQWrapper::get_sim()->setEngineFloatParam(_ZMQWrapper::get_sim()->mujoco_joint_damping, _get_handle_from_map(jointname), damping);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_joint_dampings(const std::vector<std::string> &jointnames,
                                                        const double &damping)
{
    for (size_t i=0;i<jointnames.size();i++)
        _set_mujoco_joint_damping(jointnames.at(i), damping);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::set_mujoco_joint_armature Armature inertia (or rotor inertia, or reflected inertia)
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
void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_joint_armature(const std::string &jointname, const double &armature)
{
   _ZMQWrapper::get_sim()->setEngineFloatParam(_ZMQWrapper::get_sim()->mujoco_joint_armature,_get_handle_from_map(jointname), armature);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_joint_armatures(const std::vector<std::string> &jointnames,
                                                         const double &armature)
{
    for (size_t i=0;i<jointnames.size();i++)
        _set_mujoco_joint_armature(jointnames.at(i), armature);
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_body_friction(const std::string &bodyname, const std::vector<double> &friction)
{
    std::vector<int64_t> mujoco_body_param = {_ZMQWrapper::get_sim()->mujoco_body_friction1,
                                              _ZMQWrapper::get_sim()->mujoco_body_friction2,
                                              _ZMQWrapper::get_sim()->mujoco_body_friction3};
    _check_sizes(friction, mujoco_body_param, "Error in DQ_CoppeliaSimInterface::set_mujoco_body_friction: "
                                              "friction must be a vector of size "+std::to_string(mujoco_body_param.size()));
    for (size_t i=0;i<mujoco_body_param.size();i++)
        _ZMQWrapper::get_sim()->setEngineFloatParam(mujoco_body_param.at(i), _get_handle_from_map(bodyname),friction.at(i));
}

void DQ_CoppeliaSimInterfaceZMQ::_set_mujoco_body_frictions(const std::vector<std::string> &bodynames, const std::vector<double> &friction)
{
    for (auto& bodyname : bodynames)
        _set_mujoco_body_friction(bodyname, friction);
}




/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_mass
 * @param handle
 * @return
 */
double DQ_CoppeliaSimInterfaceZMQ::_get_mass(const int &handle) const
{
   _check_client();
   return _ZMQWrapper::get_sim()->getShapeMass(handle);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_mass
 * @param object_name
 * @return
 */
double DQ_CoppeliaSimInterfaceZMQ::_get_mass(const std::string &object_name)
{
    return _get_mass(_get_handle_from_map(object_name));
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_center_of_mass
 * @param handle
 * @param reference_frame
 * @return
 */
DQ DQ_CoppeliaSimInterfaceZMQ::_get_center_of_mass(const int &handle, const REFERENCE &reference_frame) const
{
    DQ COM_body_frame;
    MatrixXd Inertia_maxtrix_body_frame;
    std::tie(COM_body_frame, Inertia_maxtrix_body_frame) =_get_center_of_mass_and_inertia_matrix(handle);
    if (reference_frame == REFERENCE::BODY_FRAME)
        return COM_body_frame;
    else
    {
        DQ x_0_bodyFrame = _get_object_pose(handle);
        DQ x_bodyFrame_com = 1 + 0.5*E_*COM_body_frame;
        return (x_0_bodyFrame*x_bodyFrame_com).translation();
    }
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_center_of_mass
 * @param object_name
 * @param reference_frame
 * @return
 */
DQ DQ_CoppeliaSimInterfaceZMQ::_get_center_of_mass(const std::string &object_name, const REFERENCE &reference_frame)
{
    return _get_center_of_mass(_get_handle_from_map(object_name), reference_frame);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_inertia_matrix
 * @param handle
 * @param reference_frame
 * @return
 */
MatrixXd DQ_CoppeliaSimInterfaceZMQ::_get_inertia_matrix(const int &handle, const REFERENCE &reference_frame)
{
    DQ COM_body_frame;
    MatrixXd Inertia_maxtrix_body_frame;
    std::tie(COM_body_frame, Inertia_maxtrix_body_frame) =_get_center_of_mass_and_inertia_matrix(handle);
    if (reference_frame == REFERENCE::BODY_FRAME)
        return Inertia_maxtrix_body_frame;
    else
    {
        DQ x_0_bodyFrame = _get_object_pose(handle);
        DQ x_bodyFrame_com = 1 + 0.5*E_*COM_body_frame;
        DQ x_0_com = x_0_bodyFrame*x_bodyFrame_com;
        MatrixXd R_0_COM = _get_rotation_matrix(x_0_com.P());
        return R_0_COM*Inertia_maxtrix_body_frame*R_0_COM.transpose();
    }
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_inertia_matrix
 * @param link_name
 * @param reference_frame
 * @return
 */
MatrixXd DQ_CoppeliaSimInterfaceZMQ::_get_inertia_matrix(const std::string &link_name, const REFERENCE &reference_frame)
{
    return _get_inertia_matrix(_get_handle_from_map(link_name), reference_frame);
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::_update_map update the map. It the objectname is already in the map, and
 *                      If you use the mode UPDATE_MAP::ADD, the map is updated only if the objectname is not in the map.
 *                      In other words, it is not allowed to have a abjectname twice in the map.
 *
 * @param objectname The objectname you want to add or remove from the map.
 * @param handle     The objectname handle.
 * @param mode       The operation mode. Use UPDATE_MAP::ADD or UPDATE_MAP::REMOVE accordingly.
 */
void DQ_CoppeliaSimInterfaceZMQ::_update_map(const std::string &objectname,
                                          const int &handle, const UPDATE_MAP &mode)
{
    if (mode == DQ_CoppeliaSimInterfaceZMQ::UPDATE_MAP::ADD)
        handles_map_.try_emplace(objectname, handle);
    else
        handles_map_.erase(objectname);

}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::_get_handle_from_map searchs a handle in the map. If
 *                              the handle is not found, it is taken from CoppeliaSim and the map is updated
 *                              by using the get_object_handle() method.
 *
 * @param objectname
 * @return The objectname handle.
 */
int DQ_CoppeliaSimInterfaceZMQ::_get_handle_from_map(const std::string &objectname)
{
    auto search = handles_map_.find(objectname);
    // returns a tuple <bool, int>
    //If the handle is found in the map, returns <true, handle>.


    if (search != handles_map_.end())
    { // handle found in map
        return search->second;
    }
    else
    {   // handle not found in map. Therefore, it is taken from CoppeliaSim and the map
        // is updated;
        return get_object_handle(objectname);
    }
}


//---------------Deprecated methods-----------------------------
void DQ_CoppeliaSimInterfaceZMQ::disconnect(){}
void DQ_CoppeliaSimInterfaceZMQ::disconnect_all(){}
void DQ_CoppeliaSimInterfaceZMQ::set_synchronous(const bool &flag){set_stepping_mode(flag);}
int DQ_CoppeliaSimInterfaceZMQ::wait_for_simulation_step_to_end(){return 0;}

bool DQ_CoppeliaSimInterfaceZMQ::connect(const int &port,
                                         const int &TIMEOUT_IN_MILISECONDS,
                                         const int &MAX_TRY_COUNT)
{
    return connect("localhost", _get_port_from_deprecated_default_port(port), TIMEOUT_IN_MILISECONDS);
}

bool DQ_CoppeliaSimInterfaceZMQ::connect(const std::string &ip,
                                         const int &port,
                                         const int &TIMEOUT_IN_MILISECONDS,
                                         const int &MAX_TRY_COUNT)
{
    return connect(ip, _get_port_from_deprecated_default_port(port), TIMEOUT_IN_MILISECONDS);
}


//---------------Private methods-----------------------------
/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::_remove_first_slash_from_string this method removes the slash at the beginning of a string.
 *
 *             Example: _remove_first_slash_from_string("/reference") returns "reference"
 *                      _remove_first_slash_from_string("reference")  returns "reference"
 *
 * @param str
 * @return
 */
std::string DQ_CoppeliaSimInterfaceZMQ::_remove_first_slash_from_string(const std::string &str) const
{
    std::string new_str = str;
    auto found = str.find('/');
    if (found != std::string::npos && found == 0) // The string containt the '/' in the first position
            new_str.erase(0,1); // remove the '/'
    return new_str;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::_start_with_slash returns true if the first character of a string is a slash.
 *                  Otherwise returns false
 * @param str The string
 * @return A boolean. True if the first character of a string is a slash. False otherwise
 */
bool DQ_CoppeliaSimInterfaceZMQ::_start_with_slash(const std::string &str) const
{

    size_t found = str.find('/');
    if(found == 0) // The string containt the '/'
        return true;
    else
        return false;

    //return str.starts_with("/"); // For C++20

}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::_get_standard_string returns a string that
 *        always start with "/"
 * @param str
 * @return
 */
std::string DQ_CoppeliaSimInterfaceZMQ::_get_standard_name(const std::string &str) const
{
    std::string standard_str = str;
    if (!_start_with_slash(str) && enable_deprecated_name_compatibility_ == true)
        standard_str = std::string("/")+str;
    return standard_str;
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::_get_engine returns the current engine in the simulation scene.
 * @return
 */
DQ_CoppeliaSimInterfaceZMQ::ENGINE DQ_CoppeliaSimInterfaceZMQ::_get_engine()
{
    _check_client();
    int64_t eng = _ZMQWrapper::get_sim()->getInt32Param(_ZMQWrapper::get_sim()->intparam_dynamic_engine);
    return engines_invmap.at(eng);
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::_get_velocity_const_params
 * @return
 */
std::vector<int> DQ_CoppeliaSimInterfaceZMQ::_get_velocity_const_params() const
{
    std::vector<int> params = {_ZMQWrapper::get_sim()->shapefloatparam_init_velocity_a,
        _ZMQWrapper::get_sim()->shapefloatparam_init_velocity_b,
        _ZMQWrapper::get_sim()->shapefloatparam_init_velocity_g,
        _ZMQWrapper::get_sim()->shapefloatparam_init_velocity_x,
        _ZMQWrapper::get_sim()->shapefloatparam_init_velocity_y,
        _ZMQWrapper::get_sim()->shapefloatparam_init_velocity_z
    };
    return params;
}

std::string DQ_CoppeliaSimInterfaceZMQ::_get_resources_path() const
{
    return _ZMQWrapper::get_sim()->getStringParam(_ZMQWrapper::get_sim()->stringparam_resourcesdir);
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::_load_model
 * @param path_to_filename
 * @param desired_model_name
 * @return
 */
bool DQ_CoppeliaSimInterfaceZMQ::_load_model(const std::string &path_to_filename,
                                          const std::string &desired_model_name,
                                          const bool &remove_child_script)
{
    int rtn = _ZMQWrapper::get_sim()->loadModel(path_to_filename);
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
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_transformation_matrix
 * @param coeff_vector
 * @return
 */
MatrixXd DQ_CoppeliaSimInterfaceZMQ::_get_transformation_matrix(const std::vector<double> &coeff_vector) const
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
MatrixXd DQ_CoppeliaSimInterfaceZMQ::_get_rotation_matrix(const DQ& r) const{
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
 * @brief DQ_CoppeliaSimInterfaceZMQ::_get_rotation_from_direction
 * @param direction
 * @return
 */
DQ DQ_CoppeliaSimInterfaceZMQ::_get_pose_from_direction(const DQ& direction, const DQ& point)
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

int DQ_CoppeliaSimInterfaceZMQ::_add_primitive(const PRIMITIVE &primitive, const std::string &name, const std::vector<double> &sizes) const
{
    if (!_object_exist_on_scene(name))
    {
        _check_client();
        int shapeHandle = _ZMQWrapper::get_sim()->createPrimitiveShape(_get_primitive_identifier(primitive), sizes, 0);
        _set_object_name(shapeHandle, _remove_first_slash_from_string(name));
        return shapeHandle;
    }else
        return -1;
}

std::vector<std::string> DQ_CoppeliaSimInterfaceZMQ::_create_static_axis_at_origin(const int& parent_handle,
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

void DQ_CoppeliaSimInterfaceZMQ::_set_static_object_properties(const std::string &name,
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

void DQ_CoppeliaSimInterfaceZMQ::_set_static_object_properties(const int &handle,
                                                               const int &parent_handle,
                                                               const DQ &pose,
                                                               const std::vector<double> &rgba_color) const
{
    _set_object_color(handle, rgba_color);
    _set_object_as_respondable(handle, false);
    _set_object_as_static(handle, true);
    _set_object_pose(handle, pose);
    _set_object_parent(handle, parent_handle, false);
}

int DQ_CoppeliaSimInterfaceZMQ::_get_primitive_identifier(const PRIMITIVE &primitive) const
{
    switch (primitive)
    {
    case PRIMITIVE::PLANE:
        return _ZMQWrapper::get_sim()->primitiveshape_plane;
    case PRIMITIVE::DISC:
        return _ZMQWrapper::get_sim()->primitiveshape_disc;
    case PRIMITIVE::CUBOID:
        return _ZMQWrapper::get_sim()->primitiveshape_cuboid;
    case PRIMITIVE::SPHEROID:
        return _ZMQWrapper::get_sim()->primitiveshape_spheroid;
    case PRIMITIVE::CYLINDER:
        return _ZMQWrapper::get_sim()->primitiveshape_cylinder;
    case PRIMITIVE::CONE:
        return _ZMQWrapper::get_sim()->primitiveshape_cone;
    case PRIMITIVE::CAPSULE:
        return _ZMQWrapper::get_sim()->primitiveshape_capsule;
    default: // This line is required in GNU/Linux
        _throw_runtime_error("wrong argument");

    }
}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::_check_client checks if the client is initialized.
 */
void DQ_CoppeliaSimInterfaceZMQ::_check_client() const
{
    if (!client_created_)
        throw std::runtime_error("Unestablished connection. Did you use connect()?");
}

[[noreturn]] void DQ_CoppeliaSimInterfaceZMQ::_throw_runtime_error(const std::string &msg) const
{
    stop_simulation();
    std::cerr<<"Something went wrong. I stopped the simulation!"<<std::endl;
    throw std::runtime_error(msg);
}


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::get_center_of_mass_and_inertia_matrix
 * @param handle
 * @return
 */
std::tuple<DQ, MatrixXd> DQ_CoppeliaSimInterfaceZMQ::_get_center_of_mass_and_inertia_matrix(const int &handle) const
{
    std::vector<double> inertia_matrix_coeff;
    std::vector<double> center_of_mass_coeff;
    std::tie(inertia_matrix_coeff, center_of_mass_coeff) = _ZMQWrapper::get_sim()->getShapeInertia(handle);

    MatrixXd Inertia_maxtrix_body_frame = Map<VectorXd>(inertia_matrix_coeff.data(),
                                                        inertia_matrix_coeff.size()).reshaped(3,3);
    MatrixXd COM_body_frame_matrix =  _get_transformation_matrix(center_of_mass_coeff);
    DQ COM_body_frame = DQ(COM_body_frame_matrix.col(3));
    double mass = _get_mass(handle);

    return {COM_body_frame, Inertia_maxtrix_body_frame/mass};
}



DQ_CoppeliaSimInterfaceZMQ::experimental::experimental(const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> &smptr)
    :smptr_{smptr}
{

}

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::experimental::plot_reference_frame adds a reference frame object
 *                  in the CoppeliaSim scene.
 * @param name The name of the reference frame
 * @param pose The position and orientation of the reference frame
 * @param scale The scale of the reference frame.
 * @param thickness_and_length The thicknessand and length.
 */
void DQ_CoppeliaSimInterfaceZMQ::experimental::plot_reference_frame(const std::string &name,
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


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::experimental::plot_plane adds a plane object in the CoppeliaSim scene
 * @param name
 * @param normal_to_the_plane
 * @param location
 * @param sizes
 * @param rgba_color
 * @param add_normal
 * @param normal_scale
 */
void DQ_CoppeliaSimInterfaceZMQ::experimental::plot_plane(const std::string &name, const DQ &normal_to_the_plane, const DQ &location, const std::vector<double> &sizes, const std::vector<double> &rgba_color, const bool &add_normal, const double &normal_scale)
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

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::experimental::plot_line add a segment line in the CoppeliaSim scene
 * @param name
 * @param line_direction
 * @param location
 * @param thickness_and_length
 * @param rgba_color
 * @param add_arrow
 * @param arrow_scale
 */
void DQ_CoppeliaSimInterfaceZMQ::experimental::plot_line(const std::string &name, const DQ &line_direction, const DQ &location, const std::vector<double> &thickness_and_length, const std::vector<double> &rgba_color, const bool &add_arrow, const double &arrow_scale)
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

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::experimental::plot_cylinder adds a cylinder in the CoppeliaSim scene
 * @param name
 * @param direction
 * @param location
 * @param width_and_length
 * @param rgba_color
 * @param add_line
 * @param line_scale
 */
void DQ_CoppeliaSimInterfaceZMQ::experimental::plot_cylinder(const std::string &name, const DQ &direction, const DQ &location, const std::vector<double> &width_and_length, const std::vector<double> &rgba_color, const bool &add_line, const double &line_scale)
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

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::experimental::plot_sphere adds an sphere object in the CoppeliaSim scene
 * @param name
 * @param location
 * @param size
 * @param rgba_color
 */
void DQ_CoppeliaSimInterfaceZMQ::experimental::plot_sphere(const std::string &name, const DQ &location, const double &size, const std::vector<double> rgba_color)
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


/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::experimental::load_model loads a model to
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
bool DQ_CoppeliaSimInterfaceZMQ::experimental::load_model(const std::string &path_to_filename,
                                                          const std::string &desired_model_name,
                                                          const bool &load_model_only_if_missing,
                                                          const bool &remove_child_script)
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

/**
 * @brief DQ_CoppeliaSimInterfaceZMQ::experimental::::load_from_model_browser loads a model from
 *        the CoppeliaSim model browser.
 *
 *      Ex: load_from_model_browser("/robots/non-mobile/FrankaEmikaPanda.ttm", "/Franka");
 *
 * @param path_to_filename The path to the model relative to the model browser.
 * @param desired_model_name The name you want for the loaded model.
 * @param load_model_only_if_missing If the model exists (with the same alias)
 *                                   the model is not loaded. (Default)
 * @param remove_child_script Remove the associated child script of the model
 *                            (Default)
 * @return A boolean flag. True if the model was loaded. False otherwise.
 */

bool DQ_CoppeliaSimInterfaceZMQ::experimental::load_from_model_browser(const std::string &path_to_filename, const std::string &desired_model_name, const bool &load_model_only_if_missing, const bool &remove_child_script)
{
    smptr_->_check_client();
    return load_model(smptr_->_get_resources_path() + std::string("/models") + path_to_filename,
                      desired_model_name, load_model_only_if_missing, remove_child_script);
}

int DQ_CoppeliaSimInterfaceZMQ::experimental::add_primitive(const PRIMITIVE &primitive, const std::string &name, const std::vector<double> &sizes)
{
    return smptr_->_add_primitive(primitive, name, sizes);
}

bool DQ_CoppeliaSimInterfaceZMQ::experimental::object_exist_on_scene(const std::string &objectname) const
{
    return smptr_->_object_exist_on_scene(objectname);
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::remove_object(const std::string &objectname, const bool &remove_children)
{
    smptr_->_remove_object(objectname, remove_children);
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::draw_trajectory(const std::string &objectname, const double &size, const std::vector<double> &rgb_color, const int &max_item_count)
{
    smptr_->_draw_trajectory(objectname, size, rgb_color, max_item_count);
}

bool DQ_CoppeliaSimInterfaceZMQ::experimental::check_collision(const std::string &objectname1, const std::string &objectname2)
{
    return smptr_->_check_collision(objectname1, objectname2);
}

std::vector<double> DQ_CoppeliaSimInterfaceZMQ::experimental::get_bounding_box_size(const std::string &objectname)
{
    return smptr_->_get_bounding_box_size(objectname);
}


void DQ_CoppeliaSimInterfaceZMQ::experimental::set_mujoco_joint_stiffnesses(const std::vector<std::string> &jointnames, const double &stiffness)
{
    smptr_->_set_mujoco_joint_stiffnesses(jointnames, stiffness);
}


void DQ_CoppeliaSimInterfaceZMQ::experimental::set_mujoco_joint_dampings(const std::vector<std::string> &jointnames, const double &damping)
{
    smptr_->_set_mujoco_joint_dampings(jointnames, damping);
}


void DQ_CoppeliaSimInterfaceZMQ::experimental::set_mujoco_joint_armatures(const std::vector<std::string> &jointnames, const double &armature)
{
    smptr_->_set_mujoco_joint_armatures(jointnames, armature);
}


void DQ_CoppeliaSimInterfaceZMQ::experimental::set_mujoco_body_frictions(const std::vector<std::string> &bodynames, const std::vector<double> &friction)
{
    smptr_->_set_mujoco_body_frictions(bodynames, friction);
}


void DQ_CoppeliaSimInterfaceZMQ::experimental::set_joint_modes(const std::vector<std::string> &jointnames,
                                                               const JOINT_MODE &joint_mode)
{
    smptr_->_set_joint_modes(jointnames, joint_mode);
}


void DQ_CoppeliaSimInterfaceZMQ::experimental::set_joint_control_modes(const std::vector<std::string> &jointnames,
                                                                       const JOINT_CONTROL_MODE &joint_control_mode)
{
    smptr_->_set_joint_control_modes(jointnames, joint_control_mode);
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::enable_dynamics(const bool &flag)
{
    smptr_->_enable_dynamics(flag);
}

double DQ_CoppeliaSimInterfaceZMQ::experimental::get_simulation_time_step() const
{
    return smptr_->_get_simulation_time_step();
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::set_simulation_time_step(const double &time_step)
{
    smptr_->_set_simulation_time_step(time_step);
}

double DQ_CoppeliaSimInterfaceZMQ::experimental::get_physics_time_step() const
{
    return smptr_->_get_physics_time_step();
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::set_physics_time_step(const double &time_step) const
{
    smptr_->_set_physics_time_step(time_step);
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::set_engine(const ENGINE &engine)
{
    smptr_->_set_engine(engine);
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::set_object_color(const std::string &objectname, const std::vector<double> &rgba_color)
{
    smptr_->_set_object_color(objectname, rgba_color);
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::set_object_as_respondable(const std::string &objectname, const bool &respondable_object)
{
    smptr_->_set_object_as_respondable(objectname, respondable_object);
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::set_object_as_static(const std::string &objectname, const bool &static_object)
{
    smptr_->_set_object_as_static(objectname, static_object);
}

std::vector<std::string> DQ_CoppeliaSimInterfaceZMQ::experimental::get_jointnames_from_parent_object(const std::string &parent_objectname)
{
    return smptr_->_get_jointnames_from_parent_object(parent_objectname);
}

std::vector<std::string> DQ_CoppeliaSimInterfaceZMQ::experimental::get_shapenames_from_parent_object(const std::string &parent_objectname,
                                                                                                     const SHAPE_TYPE &shape_type)
{
    return smptr_->_get_shapenames_from_parent_object(parent_objectname, shape_type);
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::set_twist(const std::string &objectname, const DQ &twist, const REFERENCE &reference)
{
    smptr_->_set_twist(objectname, twist, reference);
}

DQ DQ_CoppeliaSimInterfaceZMQ::experimental::get_twist(const std::string &objectname, const REFERENCE &reference)
{
    return smptr_->_get_twist(objectname, reference);
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::set_gravity(const DQ &gravity)
{
    smptr_->_set_gravity(gravity);
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::load_scene(const std::string &path_to_filename) const
{
    smptr_->_load_scene(path_to_filename);
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::save_scene(const std::string &path_to_filename) const
{
    smptr_->_save_scene(path_to_filename);
}

void DQ_CoppeliaSimInterfaceZMQ::experimental::close_scene() const
{
    smptr_->_close_scene();
}

