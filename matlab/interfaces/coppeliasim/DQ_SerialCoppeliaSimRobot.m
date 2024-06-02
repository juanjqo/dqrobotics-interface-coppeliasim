classdef DQ_SerialCoppeliaSimRobot < DQ_CoppeliaSimRobot

    properties (Access = protected)
        jointnames_        cell
        base_frame_name_   string
        joint_control_mode_ DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE 
        robot_is_used_as_visualization_tool_ logical
    end

    methods (Access = protected)
        function obj = DQ_SerialCoppeliaSimRobot(robot_name, coppeliasim_interface_sptr)
            obj = obj@DQ_CoppeliaSimRobot(robot_name, coppeliasim_interface_sptr);
         obj.initialize_jointnames_from_coppeliasim_();
         % // By Default, the robot is controlled by joint positions with both the dynamic engine
         % // and the stepping mode enabled.
         obj.joint_control_mode_ = DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.POSITION;
         obj.robot_is_used_as_visualization_tool_ = false;
        end

        function initialize_jointnames_from_coppeliasim_(obj)
            obj.jointnames_ = obj.get_interface_sptr_().get_jointnames_from_base_objectname(obj.robot_name_);
            obj.base_frame_name_ = obj.jointnames_{1};
        end
    end
    
    methods
        function set_operation_modes(obj, joint_mode, joint_control_mode)
             arguments
                 obj
                 joint_mode DQ_CoppeliaSimInterface_JOINT_MODE
                 joint_control_mode DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE
             end
             obj.get_interface_sptr_().set_joint_modes(obj.jointnames_, joint_mode);
             obj.get_interface_sptr_().set_joint_control_modes(obj.jointnames_, joint_control_mode);
        end

        function set_robot_as_visualization_tool(obj)

            obj.get_interface_sptr_().set_stepping_mode(false);
            obj.get_interface_sptr_().enable_dynamics(false);
            obj.get_interface_sptr_().set_joint_modes(obj.jointnames_,...
                                                   DQ_CoppeliaSimInterface_JOINT_MODE.KINEMATIC);
            obj.robot_is_used_as_visualization_tool_ = true;
        end

        function set_joint_control_type(obj, joint_control_mode)
            arguments
                obj 
                joint_control_mode DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE
            end
            obj.joint_control_mode_ = joint_control_mode;
            obj.get_interface_sptr_().enable_dynamics(true);
            obj.get_interface_sptr_().set_stepping_mode(true);
            obj.set_operation_modes(DQ_CoppeliaSimInterface_JOINT_MODE.DYNAMIC, obj.joint_control_mode_);
        end

        function set_control_inputs(obj, u)
            if (obj.robot_is_used_as_visualization_tool_)
                    obj.get_interface_sptr_().set_joint_positions(obj.jointnames_, u);
            else
                switch (obj.joint_control_mode_)
                    case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.FREE
                    case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.FORCE
                    case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.VELOCITY
                         obj.get_interface_sptr_().set_joint_target_velocities(obj.jointnames_, u);
                    case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.POSITION
                         obj.get_interface_sptr_().set_joint_target_positions(obj.jointnames_, u);           
                    case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.SPRING
                    case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.CUSTOM
                    case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.TORQUE
                        obj.get_interface_sptr_().set_joint_torques(obj.jointnames_, u);
                end
                obj.get_interface_sptr_().trigger_next_simulation_step();
            end
            
        end

        function rtn = get_joint_names(obj)
                rtn = obj.jointnames_;
        end

        function set_configuration_space_positions(obj, q)
            obj.get_interface_sptr_().set_joint_positions(obj.jointnames_,q);
        end

        function q = get_configuration_space_positions(obj)
            q =  obj.get_interface_sptr_().get_joint_positions(obj.jointnames_);
        end 

        function set_target_configuration_space_positions(obj, q_target)
            obj.get_interface_sptr_().set_joint_target_positions(obj.jointnames_, q_target);
        end

        function q_dot = get_configuration_space_velocities(obj)
            q_dot = obj.get_interface_sptr_().get_joint_velocities(obj.jointnames_);
        end

        function set_target_configuration_space_velocities(obj, v_target)
            obj.get_interface_sptr_().set_joint_target_velocities(obj.jointnames_, v_target);
        end

        function set_configuration_space_torques(obj, torques)
            obj.get_interface_sptr_().set_joint_torques(obj.jointnames_, torques);
        end


        function torques = get_configuration_space_torques(obj)
            torques = obj.get_interface_sptr_().get_joint_torques(obj.jointnames_);
        end

    end
end

