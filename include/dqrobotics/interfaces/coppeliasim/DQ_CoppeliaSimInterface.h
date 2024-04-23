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
*/

#pragma once
#include <dqrobotics/DQ.h>
#include <map>

using namespace DQ_robotics;
using namespace Eigen;

class DQ_CoppeliaSimInterface
{
public:
    enum JOINT_MODE
    {
        KINEMATIC,
        DYNAMIC,
        DEPENDENT
    };
    enum ENGINE
    {
        BULLET,
        ODE,
        VORTEX,
        NEWTON,
        MUJOCO
    };
    enum JOINT_CONTROL_MODE
    {
        FREE,
        FORCE,
        VELOCITY,
        POSITION,
        SPRING,
        CUSTOM};

    DQ_CoppeliaSimInterface();
    bool connect(const std::string& host = "localhost",
                 const int& rpcPort = 23000,
                 const int& cntPort = -1,
                 const int& verbose_ = -1);

    void   start_simulation() const;
    void   pause_simulation() const;
    void   stop_simulation()  const;
    void   set_stepping_mode(const bool& flag);
    double get_simulation_time() const;
    void   trigger_next_simulation_step() const;
    bool   is_simulation_running() const;
    int    get_simulation_state() const;
    void   set_status_bar_message(const std::string& message) const;

    int get_object_handle(const std::string& objectname);
    std::vector<int> get_object_handles(const std::vector<std::string>& objectnames);


    DQ   get_object_translation(const int& handle) const;
    DQ   get_object_translation(const std::string& objectname);

    void set_object_translation(const int& handle, const DQ& t);
    void set_object_translation(const std::string& objectname, const DQ& t);

    DQ   get_object_rotation(const int& handle) const;
    DQ   get_object_rotation(const std::string& objectname);

    void set_object_rotation(const int& handle, const DQ& r);
    void set_object_rotation(const std::string& objectname, const DQ& r);

    DQ get_object_pose(const int& handle) const;
    DQ get_object_pose(const int& handle, const int& relative_to_handle) const;
    DQ get_object_pose(const std::string& objectname);
    DQ get_object_pose(const std::string& objectname,
                       const std::string& relative_to_objectname);

    void set_object_pose(const int& handle, const DQ& h);
    void set_object_pose(const std::string& objectname, const DQ& h);

    double   get_joint_position(const int& handle) const;
    double   get_joint_position(const std::string& jointname);
    VectorXd get_joint_positions(const std::vector<int>& handles) const;
    VectorXd get_joint_positions(const std::vector<std::string>& jointnames);

    void     set_joint_position(const int& handle, const double& angle_rad) const;
    void     set_joint_position(const std::string& jointname, const double& angle_rad);
    void     set_joint_positions(const std::vector<int>& handles, const VectorXd& angles_rad) const;
    void     set_joint_positions(const std::vector<std::string>& jointnames, const VectorXd& angles_rad);

    void     set_joint_target_position(const int& handle, const double& angle_rad) const;
    void     set_joint_target_position(const std::string& jointname, const double& angle_rad);
    void     set_joint_target_positions(const std::vector<int>& handles, const VectorXd& angles_rad) const;
    void     set_joint_target_positions(const std::vector<std::string>& jointnames, const VectorXd& angles_rad);

    double   get_joint_velocity(const int& handle) const;
    double   get_joint_velocity(const std::string& jointname);
    VectorXd get_joint_velocities(const std::vector<int>& handles) const;
    VectorXd get_joint_velocities(const std::vector<std::string>& jointnames);

    void     set_joint_target_velocity(const int& handle, const double& angle_rad_dot) const;
    void     set_joint_target_velocity(const std::string& jointname, const double& angle_rad_dot);
    void     set_joint_target_velocities(const std::vector<int>& handles, const VectorXd& angles_rad_dot) const;
    void     set_joint_target_velocities(const std::vector<std::string>& jointnames, const VectorXd& angles_rad_dot);

    void     set_joint_torque(const int& handle, const double& torque) const;
    void     set_joint_torque(const std::string& jointname, const double& torque);
    void     set_joint_torques(const std::vector<int>& handles, const VectorXd& torques) const;
    void     set_joint_torques(const std::vector<std::string>& jointnames, const VectorXd& torques);

    double   get_joint_torque(const int& handle) const;
    double   get_joint_torque(const std::string& jointname);
    VectorXd get_joint_torques(const std::vector<int>& handles) const;
    VectorXd get_joint_torques(const std::vector<std::string>& jointnames);

    std::string get_object_name(const int& handle);
    std::vector<std::string> get_jointnames_from_base_objectname(const std::string& base_objectname);

    //------------------settings features-----------------------------------------------------
    void   set_joint_mode(const std::string& jointname, const JOINT_MODE& joint_mode);
    void   set_joint_modes(const std::vector<std::string>& jointnames, const JOINT_MODE& joint_mode);
    void   set_joint_control_mode(const std::string& jointname, const JOINT_CONTROL_MODE& joint_control_mode);
    void   set_joint_control_modes(const std::vector<std::string>& jointnames, const JOINT_CONTROL_MODE& joint_control_mode);
    void   enable_dynamics(const bool& flag);
    double get_simulation_time_step() const;
    void   set_simulation_time_step(const double& time_step);
    double get_physics_time_step() const;
    void   set_physics_time_step(const double& time_step) const;
    void   set_dynamic_engine(const ENGINE& engine);
    void   set_gravity(const DQ& gravity=-9.81*k_);
    DQ     get_gravity() const;
    //----------------------------------------------------------------------------------------

    std::map<std::string, int> get_map();
    void show_map();

    //-----------Deprecated methods---------------------------//
    [[deprecated("This method is not required with ZeroMQ remote API.")]]
    void disconnect();
    [[deprecated("This method is not required with ZeroMQ remote API.")]]
    void disconnect_all();
    [[deprecated("The synchronous mode is now called stepping mode. Consider using set_stepping_mode(flag) instead.")]]
    void set_synchronous(const bool& flag);
    [[deprecated("This method is not required with ZeroMQ remote API.")]]
    int wait_for_simulation_step_to_end();
    //---------------------------------------------------------//




private:
    bool client_created_ = false;

    //-------------------map zone--------------------------------------------
    std::string _map_simulation_state(const int& state) const;
    std::map<std::string, int> set_states_map_;
    void _update_map(const std::string& objectname, const int& handle);
    int _get_handle_from_map(const std::string& objectname);
    //------------------------------------------------------------------------

    template<typename T, typename U>
    void _check_sizes(const T &v1,
                      const U &v2,
                      const std::string error_message) const
    {
        if (static_cast<std::size_t>(v1.size()) != static_cast<std::size_t>(v2.size()))
            throw std::runtime_error(error_message);
    }
};


