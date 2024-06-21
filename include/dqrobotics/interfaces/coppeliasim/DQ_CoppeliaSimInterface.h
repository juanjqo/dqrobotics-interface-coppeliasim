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

#pragma once
#include <dqrobotics/DQ.h>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <print> // For future use of C++23 features

using namespace DQ_robotics;
using namespace Eigen;

class DQ_CoppeliaSimInterface
{
public:
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
    enum class REFERENCE
    {
        BODY_FRAME,
        ABSOLUTE_FRAME
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

    DQ_CoppeliaSimInterface();
    ~DQ_CoppeliaSimInterface();
    bool connect(const std::string& host = "localhost",
                 const int& rpcPort = 23000,
                 const int& MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION = 300,
                 const int& cntPort = -1,
                 const int& verbose = -1);

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
    DQ get_object_pose(const std::string& objectname);

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

    std::vector<std::string> get_object_names(const auto& handles);

    std::vector<std::string> get_jointnames_from_base_objectname(const std::string& base_objectname);
    std::vector<std::string> get_linknames_from_base_objectname(const std::string& base_objectname);

    VectorXd get_angular_and_linear_velocities(const int& handle,
                                           const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME) const;

    VectorXd get_angular_and_linear_velocities(std::string& objectname,
                                           const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME);

    void set_angular_and_linear_velocities(const int& handle,
                                           const DQ& w,
                                           const DQ& p_dot,
                                           const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME) const;
    void set_angular_and_linear_velocities(std::string& objectname,
                                           const DQ& w,
                                           const DQ& p_dot,
                                           const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME);
    void set_twist(const int& handle,
                   const DQ& twist,
                   const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME) const;
    void set_twist(const std::string& objectname,
                   const DQ& twist, const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME);
    DQ   get_twist(const int& handle,
                   const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME) const;
    DQ   get_twist(const std::string& objectname,
                   const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME);

    double get_mass(const int& handle) const;
    double get_mass(const std::string& object_name);

    DQ     get_center_of_mass(const int& handle, const REFERENCE& reference_frame=REFERENCE::ABSOLUTE_FRAME) const;
    DQ     get_center_of_mass(const std::string& object_name, const REFERENCE& reference_frame=REFERENCE::ABSOLUTE_FRAME);

    MatrixXd get_inertia_matrix(const int& handle, const REFERENCE& reference_frame=REFERENCE::BODY_FRAME);
    MatrixXd get_inertia_matrix(const std::string& link_name, const REFERENCE& reference_frame=REFERENCE::BODY_FRAME);

    //------------------Exclusive methods---------------------------------------------------------------
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

    void load_scene(const std::string& path_to_filename) const;
    void save_scene(const std::string& path_to_filename) const;
    void close_scene() const;

    bool load_model(const std::string& path_to_filename,
                    const std::string& desired_model_name,
                    const bool& load_model_only_if_missing = true,
                    const bool& remove_child_script = true);
    bool load_from_model_browser(const std::string& path_to_filename,
                                       const std::string& desired_model_name,
                                       const bool& load_model_only_if_missing = true,
                                       const bool& remove_child_script = true);

    void remove_child_script_from_object(const std::string& objectname, const std::string& script_name = "/Script");

    bool object_exist_on_scene(const std::string& objectname) const;

    void set_object_name(const int& handle,
                         const std::string& new_object_name) const;
    void set_object_name(const std::string& current_object_name,
                         const std::string& new_object_name);

    //-------- Methods to be implemented on Matlab---------------

    void set_object_color(const int& handle,
                          const std::vector<double> rgba_color) const;

    void set_object_color(const std::string& objectname,
                          const std::vector<double> rgba_color);

    void set_object_as_respondable(const int& handle,
                                   const bool& respondable_object = true) const;

    void set_object_as_respondable(const std::string& objectname,
                                   const bool& respondable_object = true);

    void set_object_as_static(const int& handle,
                              const bool& static_object = true) const;

    void set_object_as_static(const std::string& objectname,
                              const bool& static_object = true);

    void add_primitive(const PRIMITIVE& primitive,
                       const std::string& name,
                       const std::vector<double>& sizes) const;

    void set_object_parent(const int& handle,
                           const int& parent_handle,
                           const bool& move_child_to_parent_pose);
    void set_object_parent(const std::string& objectname,
                           const std::string& parent_object_name,
                           const bool& move_child_to_parent_pose = true);

    bool check_collision(const int& handle1,
                         const int& handle2) const;
    bool check_collision(const std::string& objectname1,
                         const std::string& objectname2);
    std::tuple<double, DQ, DQ> check_distance(const int& handle1,
                                                const int& handle2,
                                                const double& threshold = 0) const;
    std::tuple<double, DQ, DQ> check_distance(const std::string& objectname1,
                                                const std::string& objectname2,
                                                const double& threshold = 0);

    double compute_distance(const int& handle1,
                            const int& handle2,
                            const double& threshold = 0) const;
    double compute_distance(const std::string& objectname1,
                            const std::string& objectname2,
                            const double& threshold = 0);

    void plot_plane(const std::string& name,
                    const DQ& normal_to_the_plane,
                    const DQ& location,
                    const std::vector<double>& sizes = {0.2,0.2},
                    const std::vector<double>& rgba_color = {1,0,0,0.5},
                    const bool& add_normal = true,
                    const double& normal_scale = 1);

    void plot_line(const std::string& name,
                   const DQ& line_direction,
                   const DQ& location,
                   const std::vector<double>& thickness_and_length = {0.01,1.5},
                   const std::vector<double>& rgba_color = {1,0,0,0.5},
                   const bool& add_arrow = true,
                   const double& arrow_scale = 1);

    void plot_cylinder(const std::string& name,
                       const DQ& direction,
                       const DQ& location,
                       const std::vector<double>& width_and_length = {0.2,1.0},
                       const std::vector<double>& rgba_color = {1,0,0,0.5},
                       const bool& add_line = true,
                       const double& line_scale = 1);

    void plot_sphere(const std::string& name,
                     const DQ& location,
                     const double& size = 0.2,
                     const std::vector<double> rgba_color = {1,0,0,0.5});

    void plot_reference_frame(const std::string& name,
                              const DQ& pose,
                              const double& scale = 1,
                              const std::vector<double>& thickness_and_length = {0.005, 0.1});

    void remove_plotted_object(const std::string& name);


    void draw_permanent_trajectory(const DQ& point,
                         const double& size = 2,
                         const std::vector<double>& color = {1,0,0},
                         const int& max_item_count = 1000);

    int add_simulation_lua_script(const std::string& script_name,
                                  const std::string& script_code);


    void draw_trajectory(const std::string& objectname,
                         const double& size = 2,
                         const std::vector<double>& color = {1,0,1},
                         const int& max_item_count = 1000);

    void remove_object(const std::string& objectname,
                       const bool& remove_children = false);

    std::vector<double> get_bounding_box_size(const int& handle) const;
    std::vector<double> get_bounding_box_size(const std::string& objectname);
    //----------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------

    std::unordered_map<std::string, int> get_map(); //For debug
    void show_map();  // For debug
    void show_created_handle_map();  // For debug

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

    int get_primitive_identifier(const PRIMITIVE& primitive) const;

protected:
    enum class AXIS{i,j,k};


    std::string host_ = "localhost";
    int rpcPort_ {23000};
    int cntPort_ {-1};
    int verbose_ {-1};

private:
    enum class UPDATE_MAP{ADD, REMOVE};

    std::atomic<bool> client_created_ = false;
    bool enable_deprecated_name_compatibility_ = true;
    void _check_client() const;
    [[noreturn]] void _throw_runtime_error(const std::string& msg);

    int MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_{300};
    double elapsed_time_ {0};
    std::thread chronometer_thread_;
    void _join_if_joinable_chronometer_thread();
    void _start_chronometer();
    void _check_connection();
    //-------------------map zone--------------------------------------------
    std::string _map_simulation_state(const int& state);
    std::unordered_map<std::string, int> handles_map_;
    void _update_map(const std::string& objectname, const int& handle, const UPDATE_MAP& mode = UPDATE_MAP::ADD);
    int _get_handle_from_map(const std::string& objectname);

    std::unordered_map<std::string, std::vector<std::string>> created_handles_map_;
    void _update_created_handles_map(const std::string& base_objectname,
                                     const std::vector<std::string>& children_objectnames,
                                     const UPDATE_MAP& mode = UPDATE_MAP::ADD);
    //------------------------------------------------------------------------
    std::string _remove_first_slash_from_string(const std::string& str) const;
    bool _start_with_slash(const std::string& str) const;
    std::string _get_standard_name(const std::string& str) const;


    std::unordered_map<ENGINE, int> engines_ = {{ENGINE::BULLET, 0},
                                                {ENGINE::ODE,    1},
                                                {ENGINE::VORTEX, 2},
                                                {ENGINE::NEWTON, 3},
                                                {ENGINE::MUJOCO, 4}};

    std::unordered_map<int, std::string> simulation_status_ = {{0, "simulation stopped"},
                                                               {8, "simulation paused"},
                                                               {17,"simulation advancing running"},
                                                               {22, "simulation advancing last before stop"},
                                                               {19, "simulation advancing last before pause"},
                                                               {16, "simulation advancing first after stop or simulation advancing"},
                                                               {20, "simulation advancing first after pause"},
                                                               {21, "simulation advancing about to stop"}};

    std::vector<int> _get_velocity_const_params() const;


    bool _load_model(const std::string& path_to_filename,
                     const std::string& desired_model_name,
                     const bool& remove_child_script);

    MatrixXd _get_transformation_matrix(const std::vector<double>& coeff_vector) const;
    MatrixXd _get_rotation_matrix(const DQ& r) const;

    DQ _get_pose_from_direction(const DQ& direction, const DQ& point = DQ(1));

    [[nodiscard("The created primitives must be added to the created_handles_map")]]
    std::vector<std::string> _create_static_axis_at_origin(const std::string& parent_name,
                                                           const std::vector<double>& sizes,
                                                           const AXIS& axis,
                                                           const double& alpha_color = 1);

    void _set_static_object_properties(const std::string& name,
                                       const std::string& parent_name,
                                       const DQ& pose,
                                       const std::vector<double>& rgba_color);



    std::tuple<DQ, MatrixXd> _get_center_of_mass_and_inertia_matrix(const int& handle) const;


    void _check_sizes(const auto &v1,
                      const auto &v2,
                      const std::string error_message) const
    {
        if (static_cast<std::size_t>(v1.size()) != static_cast<std::size_t>(v2.size()))
            throw std::runtime_error(error_message);
    }
};


