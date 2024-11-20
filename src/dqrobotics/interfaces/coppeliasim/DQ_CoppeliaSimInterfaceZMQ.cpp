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
 * @brief DQ_CoppeliaSimInterfaceZMQ::start_simulation starts the CoppeliaSim simulation.
 */
void DQ_CoppeliaSimInterfaceZMQ::start_simulation() const
{
    _check_client();
    _ZMQWrapper::get_sim()->startSimulation();
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
 * @brief DQ_CoppeliaSimInterfaceZMQ::trigger_next_simulation_step This method sends a trigger
 *        signal to the CoppeliaSim scene, which performs a simulation step when the stepping mode is used.
 */
void DQ_CoppeliaSimInterfaceZMQ::trigger_next_simulation_step() const
{
    _check_client();
    _ZMQWrapper::get_sim()->step();
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

//-----------------------------------------------------------------------------------------

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
