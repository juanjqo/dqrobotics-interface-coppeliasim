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
 * @brief DQ_CoppeliaSimInterface::connect establish a connection between the client (your code) and
 *                                         the host (the CoppeliaSim scene).
 * @param host    eg. 'localhost' if the host is running in the same
 *                machine in which is running the client.
 * @param rpcPort The port to establish a connection. (e.g. 23000, 23001, 23002, 23003...).
 * @param cntPort
 * @param verbose_
 * @return
 */
bool DQ_CoppeliaSimInterface::connect(const std::string &host, const int &rpcPort, const int &cntPort, const int &verbose_)
{
    bool rtn = false;
    try
    {
        _create_client(host, rpcPort, cntPort, verbose_, client_created_);
        client_created_ = true;
        rtn = true;
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
    return rtn;
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
    std::string additional_error_message = "";
    if (!_string_contain_first_slash(objectname) && enable_deprecated_name_compatibility_ == false)
    {
        additional_error_message = std::string("Did you mean \"/" + objectname + "\"? \n");;

    }
    try
    {
        if (!_string_contain_first_slash(objectname) && enable_deprecated_name_compatibility_ == true)
        {
            handle = sim_->getObject(std::string("/")+objectname);
            _update_map(std::string("/")+objectname, handle);
        }else{
            handle = sim_->getObject(objectname);
            _update_map(objectname, handle);
        }
    }
    catch(const std::runtime_error& e)
    {

        throw std::runtime_error(
            std::string(e.what())
            + " \n"
            + std::string("The object \"")
            + objectname + std::string("\"")
            + std::string(" does not exist in the current scene in CoppeliaSim. \n")
            + additional_error_message
            );
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
    for(int i=0;i<n;i++)
    {
        handles[i]=get_object_handle(objectnames[i]);
    }
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
    sim_->setObjectPose(handle + sim_->handleflag_wxyzquat, pose, sim_->handle_world);
}

/**
 * @brief DQ_CoppeliaSimInterface::set_object_pose
 * @param objectname
 * @param h
 */
void DQ_CoppeliaSimInterface::set_object_pose(const std::string &objectname, const DQ &h)
{
    set_object_pose(_get_handle_from_map(objectname), h);
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
    return get_joint_position(_get_handle_from_map(jointname));
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
    double angle_dot_rad_max = 10000.0;
    if (torque==0)
    {
        angle_dot_rad_max = 0.0;
    }else if (torque<0)
    {
        angle_dot_rad_max = -10000.0;
    }
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
    std::size_t n = handles.size();
    VectorXd joint_torques(n);
    for(std::size_t i=0;i<n;i++)
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
    std::size_t n = jointnames.size();
    VectorXd joint_torques(n);
    for(std::size_t i=0;i<n;i++)
    {
        joint_torques(i)=get_joint_torque(jointnames[i]);
    }
    return joint_torques;
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
 * @brief DQ_CoppeliaSimInterface::get_object_names
 * @param handles
 * @return
 */
template<typename T>
std::vector<std::string> DQ_CoppeliaSimInterface::get_object_names(const T &handles)
{
    std::size_t n = handles.size();
    std::vector<std::string> objectnames(n);
    for(std::size_t i=0;i<n;i++)
    {
        objectnames.at(i)=get_object_name(handles.at(i));
    }
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
    std::vector<int64_t> jointhandles = sim_->getObjectsInTree(base_handle,
                                        sim_->object_joint_type,
                                        0);
    return get_object_names(jointhandles);

}

std::vector<std::string> DQ_CoppeliaSimInterface::get_linknames_from_base_objectname(const std::string &base_objectname)
{
    int base_handle = _get_handle_from_map(base_objectname);
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
    case TORQUE:
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
    sim_->saveScene(path_to_filename);
}

/**
 * @brief DQ_CoppeliaSimInterface::close_scene closes the current scene.
 */
void DQ_CoppeliaSimInterface::close_scene() const
{
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
    std::string resources_path = sim_->getStringParam(sim_->stringparam_resourcesdir);
    return load_model(resources_path + std::string("/models") + path_to_filename,
                      desired_model_name, load_model_only_if_missing, remove_child_script);
}

/**
 * @brief DQ_CoppeliaSimInterface::remove_child_script_from_object
 * @param objectname
 */
void DQ_CoppeliaSimInterface::remove_child_script_from_object(const std::string &objectname)
{
    auto script_handle = sim_->getScript(sim_->scripttype_childscript,
                                         _get_handle_from_map(objectname));
    if (script_handle != -1)
        sim_->removeScript(script_handle);
}

/**
 * @brief DQ_CoppeliaSimInterface::object_exist_on_scene
 * @param objectname
 * @return
 */
bool DQ_CoppeliaSimInterface::object_exist_on_scene(const std::string &objectname)
{
    std::optional<json> options = {{"noError", false}};
    try {
        auto rtn = sim_->getObject(objectname, options);
        return (rtn != -1) ? true : false;
    } catch (...) {
        return false;
    }

}

void DQ_CoppeliaSimInterface::set_object_name(const int &handle, const std::string &new_object_name)
{
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

/**
 * @brief DQ_CoppeliaSimInterface::get_mass
 * @param handle
 * @return
 */
double DQ_CoppeliaSimInterface::get_mass(const int &handle) const
{
   return sim_->getShapeMassAndInertia(handle);
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
    if (reference_frame == BODY_FRAME)
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
    if (reference_frame == BODY_FRAME)
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

std::string DQ_CoppeliaSimInterface::_remove_first_slash_from_string(const std::string &str)
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

bool DQ_CoppeliaSimInterface::_string_contain_first_slash(const std::string &str)
{
    size_t found = str.find('/');
    if(found == 0) // The string containt the '/'
        return true;
    else
        return false;
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

