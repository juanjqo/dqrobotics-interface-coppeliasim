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

DQ Robotics website: dqrobotics.github.io

Contributors:
- Juan Jose Quiroz Omana
       - Responsible for the original implementation.
         The DQ_CoppeliaSimInterface class is partially based on the DQ_VrepInterface class
         (https://github.com/dqrobotics/cpp-interface-vrep/blob/master/include/dqrobotics/interfaces/vrep/DQ_VrepInterface.h)

*/

#pragma once
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>
#include <thread>
#include <atomic>
#include <unordered_map>


using namespace DQ_robotics;
using namespace Eigen;

class DQ_CoppeliaSimInterfaceZMQ: public DQ_CoppeliaSimInterface
{
public:
    enum class REFERENCE
    {
        BODY_FRAME,
        ABSOLUTE_FRAME
    };
    enum class JOINT_MODE
    {
        KINEMATIC,
        DYNAMIC,
        DEPENDENT
    };
    enum class ENGINE
    {
        BULLET,
        ODE,
        VORTEX,
        NEWTON,
        MUJOCO
    };
    enum class JOINT_CONTROL_MODE
    {
        FREE,
        FORCE,
        VELOCITY,
        POSITION,
        SPRING,
        CUSTOM,
        TORQUE
    };
    enum class PRIMITIVE {
        PLANE,
        DISC,
        CUBOID,
        SPHEROID,
        CYLINDER,
        CONE,
        CAPSULE
    };
    enum class SHAPE_TYPE{
        DYNAMIC,
        STATIC,
        ANY
    };

    DQ_CoppeliaSimInterfaceZMQ();
    ~DQ_CoppeliaSimInterfaceZMQ();

    //-----------Override from DQ_CoppeliaSimInterface ---------------------------------------
    bool connect(const std::string& host = "localhost",
                 const int& port = 23000,
                 const int&TIMEOUT_IN_MILISECONDS = 300) override;

    void   start_simulation() const override;
    void   stop_simulation()  const override;
    void   set_stepping_mode(const bool& flag) const override;
    void   trigger_next_simulation_step() const override;

    int    get_object_handle(const std::string& objectname) override;
    std::vector<int> get_object_handles(const std::vector<std::string>& objectnames) override;
    DQ   get_object_translation(const std::string& objectname) override;
    void set_object_translation(const std::string& objectname, const DQ& t) override;
    DQ   get_object_rotation(const std::string& objectname) override;
    void set_object_rotation(const std::string& objectname, const DQ& r) override;
    void set_object_pose(const std::string& objectname, const DQ& h) override;
    DQ   get_object_pose(const std::string& objectname) override;

    VectorXd get_joint_positions(const std::vector<std::string>& jointnames) override;
    void     set_joint_positions(const std::vector<std::string>& jointnames,
                                 const VectorXd& angles_rad) override;
    void     set_joint_target_positions(const std::vector<std::string>& jointnames,
                                        const VectorXd& angles_rad) override;
    VectorXd get_joint_velocities(const std::vector<std::string>& jointnames) override;
    void     set_joint_target_velocities(const std::vector<std::string>& jointnames,
                                     const VectorXd& angles_rad_dot) override;
    void     set_joint_torques(const std::vector<std::string>& jointnames,
                               const VectorXd& torques) override;
    VectorXd get_joint_torques(const std::vector<std::string>& jointnames) override;

    std::string _get_object_name(const int& handle);

    template<typename T>
    std::vector<std::string> _get_object_names(const T &handles)
    {
        int n = handles.size();
        std::vector<std::string> objectnames(n);
        for(auto i=0;i<n;i++)
            objectnames.at(i)=_get_object_name(handles.at(i));

        return objectnames;
    }

    //-----------------------------------------------------------------------------------------------------//
    //-----------Deprecated methods------------------------------------------------------------------------//
    [[deprecated("This method is not required with ZeroMQ remote API.")]]
    void disconnect();
    [[deprecated("This method is not required with ZeroMQ remote API.")]]
    void disconnect_all();
    [[deprecated("The synchronous mode is now called stepping mode. Consider using set_stepping_mode(flag) instead.")]]
    void set_synchronous(const bool& flag);
    [[deprecated("This method is not required with ZeroMQ remote API.")]]
    int wait_for_simulation_step_to_end();

    // For backward compatibility
    [[deprecated("This signature with MAX_TRY_COUNT is not required with ZeroMQ remote API. "
                 "If you use the port 19997, this signature will change it to 23000.")]]
    bool connect(const std::string& ip, const int& port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);

    // For backward compatibility
    [[deprecated("This signature with MAX_TRY_COUNT is not required with ZeroMQ remote API. "
                 "If you use the port 19997, this signature will change it to 23000.")]]
    bool connect(const int &port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);

protected:
    std::string host_{"localhost"};
    int rpcPort_{23000};
    int cntPort_{-1};
    int verbose_{-1};

    enum class AXIS{i,j,k};
    enum class UPDATE_MAP{ADD, REMOVE};

    bool _connect(const std::string& host,
                 const int& rpcPort,
                 const int& MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION,
                 const int& cntPort,
                 const int& verbose);

    int _get_port_from_deprecated_default_port(const int& port);

    std::atomic<bool> client_created_{false};

    // If true, the class will accept names without a slash in the first character.
    bool enable_deprecated_name_compatibility_{true};

    void _check_client() const;
    [[noreturn]] void _throw_runtime_error(const std::string& msg) const;

    int MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_{300};
    double elapsed_time_ {0};
    std::thread chronometer_thread_;
    void _join_if_joinable_chronometer_thread();
    void _start_chronometer();
    void _check_connection();
    bool _create_client(const std::string& host,
                        const int& rpcPort,
                        const int& cntPort,
                        const int& verbose_,
                        const bool& client_flag);

    void _set_status_bar_message(const std::string& message) const;
    void __set_status_bar_message(const std::string &message, const int& verbosity_type) const;

    //-------------------map zone--------------------------------------------
    std::string _map_simulation_state(const int& state);
    std::unordered_map<std::string, int> handles_map_;
    void _update_map(const std::string& objectname, const int& handle, const UPDATE_MAP& mode = UPDATE_MAP::ADD);
    int _get_handle_from_map(const std::string& objectname);

    //------------------------------------------------------------------------
    std::string _remove_first_slash_from_string(const std::string& str) const;
    bool _start_with_slash(const std::string& str) const;
    std::string _get_standard_name(const std::string& str) const;

    //------------------Additional methods-------------------------------------------------------------//
    DQ   _get_object_translation(const int& handle) const;
    void _set_object_translation(const int& handle, const DQ& t);
    DQ   _get_object_rotation(const int& handle) const;
    void _set_object_rotation(const int& handle, const DQ& r);
    DQ   _get_object_pose(const int& handle) const;
    void _set_object_pose(const int& handle, const DQ& h) const;

    double   _get_joint_position(const int& handle) const;
    double   _get_joint_position(const std::string& jointname);
    VectorXd _get_joint_positions(const std::vector<int>& handles) const;

    void     _set_joint_position(const int& handle, const double& angle_rad) const;
    void     _set_joint_position(const std::string& jointname, const double& angle_rad);
    void     _set_joint_positions(const std::vector<int>& handles, const VectorXd& angles_rad) const ;

    void     _set_joint_target_position(const int& handle, const double& angle_rad) const;
    void     _set_joint_target_position(const std::string& jointname, const double& angle_rad);
    void     _set_joint_target_positions(const std::vector<int>& handles, const VectorXd& angles_rad) const;

    double   _get_joint_velocity(const int& handle) const;
    double   _get_joint_velocity(const std::string& jointname);
    VectorXd _get_joint_velocities(const std::vector<int>& handles) const;

    void     _set_joint_target_velocity(const int& handle, const double& angle_rad_dot) const;
    void     _set_joint_target_velocity(const std::string& jointname, const double& angle_rad_dot);
    void     _set_joint_target_velocities(const std::vector<int>& handles, const VectorXd& angles_rad_dot) const;

    void     _set_joint_torque(const int& handle, const double& torque) const;
    void     _set_joint_torque(const std::string& jointname, const double& torque);
    void     _set_joint_torques(const std::vector<int>& handles, const VectorXd& torques) const;

    double   _get_joint_torque(const int& handle) const;
    double   _get_joint_torque(const std::string& jointname);
    VectorXd _get_joint_torques(const std::vector<int>& handles) const;


    template <typename T, typename U>
    void _check_sizes(const T &v1,
                      const U &v2,
                      const std::string& error_message) const
    {
        if (static_cast<std::size_t>(v1.size()) != static_cast<std::size_t>(v2.size()))
            throw std::runtime_error(error_message);
    }
};





