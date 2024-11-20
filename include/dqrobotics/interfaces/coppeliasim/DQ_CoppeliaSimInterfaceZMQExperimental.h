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
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <thread>
#include <atomic>
#include <memory>
#include <unordered_map>


using namespace DQ_robotics;
using namespace Eigen;


class DQ_CoppeliaSimInterfaceZMQExperimental: public DQ_CoppeliaSimInterfaceZMQ
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

    DQ_CoppeliaSimInterfaceZMQExperimental();
    ~DQ_CoppeliaSimInterfaceZMQExperimental();


    void plot_reference_frame(const std::string& name,
                              const DQ& pose,
                              const double& scale = 1,
                              const std::vector<double>& thickness_and_length = {0.005, 0.1});

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

    bool load_model(const std::string& path_to_filename,
                    const std::string& desired_model_name,
                    const bool& load_model_only_if_missing = true,
                    const bool& remove_child_script = true);

    bool load_from_model_browser(const std::string& path_to_filename,
                                 const std::string& desired_model_name,
                                 const bool& load_model_only_if_missing = true,
                                 const bool& remove_child_script = true);

    int add_primitive(const PRIMITIVE& primitive,
                      const std::string& name,
                      const std::vector<double>& sizes) const;

    bool object_exist_on_scene(const std::string& objectname) const;

    void remove_object(const std::string& objectname,
                       const bool& remove_children = false);

    void draw_trajectory(const std::string& objectname,
                         const double& size = 2,
                         const std::vector<double>& rgb_color = {1,0,1},
                         const int& max_item_count = 1000);

    bool check_collision(const std::string& objectname1, const std::string& objectname2);
    std::vector<double> get_bounding_box_size(const std::string& objectname);

    //-------Mujoco settings-------------------------------------------------------------
    void set_mujoco_joint_stiffnesses(const std::vector<std::string>& jointnames,
                                      const double& stiffness);

    void set_mujoco_joint_dampings(const std::vector<std::string>& jointnames,
                                   const double& damping);

    void set_mujoco_joint_armatures(const std::vector<std::string>& jointnames,
                                    const double& armature);

    void set_mujoco_body_frictions(const std::vector<std::string>& bodynames,
                                   const std::vector<double>& friction);
    void   set_joint_mode(const std::string& jointname, const JOINT_MODE& joint_mode);
    void   set_joint_modes(const std::vector<std::string>& jointnames, const JOINT_MODE& joint_mode);
    void   set_joint_control_mode(const std::string& jointname, const JOINT_CONTROL_MODE& joint_control_mode);
    void   set_joint_control_modes(const std::vector<std::string>& jointnames, const JOINT_CONTROL_MODE& joint_control_mode);
    void   enable_dynamics(const bool& flag);
    double get_simulation_time_step() const;
    void   set_simulation_time_step(const double& time_step);
    double get_physics_time_step() const;
    void   set_physics_time_step(const double& time_step) const;
    void   set_engine(const ENGINE& engine);

    std::string get_engine();


    void   set_gravity(const DQ& gravity=-9.81*k_);
    DQ     get_gravity() const;

    void load_scene(const std::string& path_to_filename) const;
    void save_scene(const std::string& path_to_filename) const;
    void close_scene() const;

    std::vector<int> get_velocity_const_params() const;

    std::string get_resources_path() const;


    MatrixXd get_transformation_matrix(const std::vector<double>& coeff_vector) const;
    MatrixXd get_rotation_matrix(const DQ& r) const;

    DQ get_pose_from_direction(const DQ& direction, const DQ& point = DQ(1));

    void set_static_object_properties(const std::string& name,
                                      const std::string& parent_name,
                                      const DQ& pose,
                                      const std::vector<double>& rgba_color);


    void set_static_object_properties(const int& handle,
                                      const int& parent_handle,
                                      const DQ& pose,
                                      const std::vector<double>& rgba_color) const;

    void create_reference_frame(const std::string& name,
                                const double& scale = 1,
                                const std::vector<double>& thickness_and_length = {0.005, 0.1}) const;

    void create_plane(const std::string& name,
                      const std::vector<double>& sizes = {0.2,0.2},
                      const std::vector<double>& rgba_color = {1,0,0,0.5},
                      const bool& add_normal = true,
                      const double& normal_scale = 1) const;

    void create_line(const std::string& name,
                     const std::vector<double>& thickness_and_length = {0.01,1.5},
                     const std::vector<double>& rgba_color = {1,0,0,0.5},
                     const bool& add_arrow = true,
                     const double& arrow_scale = 1) const;

    void create_cylinder(const std::string& name,
                         const std::vector<double>& width_and_length = {0.2,1.0},
                         const std::vector<double>& rgba_color = {1,0,0,0.5},
                         const bool& add_line = true,
                         const double& line_scale = 1) const;

    void merge_shapes(const int& parent_handle) const;

    std::tuple<DQ, MatrixXd> get_center_of_mass_and_inertia_matrix(const int& handle) const;

    int get_primitive_identifier(const PRIMITIVE& primitive) const;

    void set_object_parent(const int& handle, const int& parent_handle, const bool& move_child_to_parent_pose) const;
    void set_object_parent(const std::string& objectname, const std::string& parent_object_name,
                           const bool& move_child_to_parent_pose = true);

    void remove_child_script_from_object(const std::string& objectname, const std::string& script_name = "/Script");

    void set_object_name(const int& handle,
                         const std::string& new_object_name) const;
    void set_object_name(const std::string& current_object_name,
                         const std::string& new_object_name);

    void set_object_color(const int& handle,
                          const std::vector<double>& rgba_color) const;

    void set_object_color(const std::string& objectname,
                          const std::vector<double>& rgba_color);

    void set_object_as_respondable(const int& handle,
                                   const bool& respondable_object = true) const;

    void set_object_as_respondable(const std::string& objectname,
                                   const bool& respondable_object = true);

    void set_object_as_static(const int& handle,
                              const bool& static_object = true) const;

    void set_object_as_static(const std::string& objectname,
                              const bool& static_object = true);


    bool check_collision(const int& handle1, const int& handle2) const;
    std::tuple<double, DQ, DQ> check_distance(const int& handle1, const int& handle2, const double& threshold = 0) const;
    std::tuple<double, DQ, DQ> check_distance(const std::string& objectname1, const std::string& objectname2, const double& threshold = 0);


    double compute_distance(const int& handle1,
                            const int& handle2,
                            const double& threshold = 0) const;
    double compute_distance(const std::string& objectname1,
                            const std::string& objectname2,
                            const double& threshold = 0);


    void draw_permanent_trajectory(const DQ& point,
                                   const double& size = 2,
                                   const std::vector<double>& color = {1,0,0},
                                   const int& max_item_count = 1000);

    int add_simulation_lua_script(const std::string& script_name,
                                  const std::string& script_code);

    std::vector<double> get_bounding_box_size(const int& handle) const;


    //-----------------------------------------------------------------------
    // Mujoco settings

    bool mujoco_is_used();
    void set_mujoco_global_impratio(const double& impratio);
    void set_mujoco_global_wind(const std::vector<double>& wind);
    void set_mujoco_global_density(const double& density);
    void set_mujoco_global_viscosity(const double& viscosity);
    void set_mujoco_global_boundmass(const double& boundmass);
    void set_mujoco_global_boundinertia(const double& boundinertia);
    void set_mujoco_global_overridemargin(const double& overridemargin);
    void set_mujoco_global_overridesolref(const std::vector<double>& overridesolref);
    void set_mujoco_global_overridesolimp(const std::vector<double>& overridesolimp);
    void set_mujoco_global_iterations(const int& iterations);
    void set_mujoco_global_integrator(const int& integrator);
    void set_mujoco_global_solver(const int& solver);
    void set_mujoco_global_njmax(const int& njmax);
    void set_mujoco_global_nstack(const int& nstack);
    void set_mujoco_global_nconmax(const int& nconmax);
    void set_mujoco_global_cone(const int& cone);
    void set_mujoco_global_overridekin(const int& overridekin);
    //void set_mujoco_global_rebuildcondition(const int& rebuildcondition);
    void set_mujoco_global_computeinertias(const bool& computeinertias);
    void set_mujoco_global_multithreaded(const bool& multithreaded);
    void set_mujoco_global_multiccd(const bool& multiccd);
    void set_mujoco_global_balanceinertias(const bool& balanceinertias);
    void set_mujoco_global_overridecontacts(const bool& overridecontacts);


    void set_mujoco_joint_stiffness(const std::string& jointname, const double& stiffness);
    void set_mujoco_joint_damping  (const std::string& jointname, const double& damping);
    void set_mujoco_joint_armature (const std::string& jointname, const double& armature);
    void set_mujoco_body_friction  (const std::string& bodyname,  const std::vector<double>& friction);

    void   pause_simulation() const;
    double get_simulation_time() const;

    bool   is_simulation_running() const;
    int    get_simulation_state() const;
    void   set_status_bar_message(const std::string& message) const;

    std::vector<std::string> get_jointnames_from_parent_object(const std::string& parent_objectname);
    std::vector<std::string> get_shapenames_from_parent_object(const std::string& parent_objectname,
                                                               const SHAPE_TYPE& shape_type = SHAPE_TYPE::ANY);

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
                   const DQ& twist,
                   const REFERENCE& reference = REFERENCE::ABSOLUTE_FRAME);

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

protected:

    std::vector<std::string> _create_static_axis_at_origin(const int& parent_handle,
                                                           const std::string& parent_name,
                                                           const std::vector<double>& sizes,
                                                           const AXIS& axis,
                                                           const double& alpha_color) const;

    ENGINE _get_engine();

    bool _load_model(const std::string& path_to_filename,
                     const std::string& desired_model_name,
                     const bool& remove_child_script);

    void   __set_status_bar_message(const std::string &message, const int& verbosity_type) const;

    std::unordered_map<ENGINE, int> engines_ = {{ENGINE::BULLET, 0},
                                                {ENGINE::ODE,    1},
                                                {ENGINE::VORTEX, 2},
                                                {ENGINE::NEWTON, 3},
                                                {ENGINE::MUJOCO, 4}};

    std::unordered_map<int,ENGINE> engines_invmap = {{0,ENGINE::BULLET},
                                                      {1,ENGINE::ODE},
                                                      {2,ENGINE::VORTEX},
                                                      {3,ENGINE::NEWTON},
                                                      {4,ENGINE::MUJOCO}};


    std::unordered_map<int, std::string> simulation_status_ = {{0, "simulation stopped"},
                                                               {8, "simulation paused"},
                                                               {17,"simulation advancing running"},
                                                               {22, "simulation advancing last before stop"},
                                                               {19, "simulation advancing last before pause"},
                                                               {16, "simulation advancing first after stop or simulation advancing"},
                                                               {20, "simulation advancing first after pause"},
                                                               {21, "simulation advancing about to stop"}};



};





