classdef DQ_CoppeliaSimInterface < handle

    properties (Access = private)
        enable_deprecated_name_compatibility_ = true;
        handles_map_;
        client_created_;
        client_;
        sim_;
    end
    properties
        
    
    end

    methods (Access = private)

        function create_client_(obj, host, rpcPort, cntPort, verbose_)
            arguments 
                obj
                host  string 
                rpcPort uint32
                cntPort uint32
                verbose_ logical
            end

            if obj.client_created_ == false
                obj.client_ = RemoteAPIClient('host', host,'port',rpcPort, 'cntPort', cntPort, 'verbose', verbose_);
                obj.sim_ = obj.client_.require('sim');
            end
        end

        function set_status_bar_message_(obj, message, verbosity_type)
             obj.sim_.addLog(verbosity_type, message);
        end

        function update_map_(obj, objectname, handle)
                obj.handles_map_ = insert(obj.handles_map_, objectname, handle);
        end

        function rtn = get_handle_from_map_(obj, objectname)
            if isConfigured(obj.handles_map_)
                if (isKey(obj.handles_map_ , objectname))
                  rtn = obj.handles_map_(objectname);
                else
                  rtn = obj.get_object_handle(objectname);
                end
            else
                rtn = obj.get_object_handle(objectname);
            end
        end

        function rtn = string_contain_first_slash_(~, objectname)
            k = strfind(objectname,'/');
            n = length(k);
            if (n==0)
                rtn = false;
            else
              if (k(1)==1)
                rtn = true;
              else
                rtn = false;
              end
            end
        end

        function check_sizes_(~, v1, v2, message)
            if (length(v1) ~= length(v2))
                error(message);
            end
        end

    end

    methods

        function obj = DQ_CoppeliaSimInterface()
            obj.client_created_ = false;
            obj.handles_map_ = dictionary;
        end
        function rtn = connect(obj, host, rpcPort, cntPort, verbose_)

            switch nargin
                case 1
                    host = 'localhost';
                    rpcPort = 23000;
                    cntPort = -1;
                    verbose_ = false;
                case 2
                    rpcPort = 23000;
                    cntPort = -1;
                    verbose_ = false;
                case 3
                    cntPort = -1;
                    verbose_ = false;
                case 4
                    verbose_ = false;
                case 5

                otherwise
                  error('Wrong number of arguments');
            end

            obj.create_client_(host, rpcPort, cntPort, verbose_);
            obj.client_created_ = true;
            rtn = true;
            obj.set_status_bar_message("       ");
            obj.set_status_bar_message_("DQ_CoppeliaSimInterface is brought " + ...
                                        "to you by Juan Jose Quiroz Omana", ...
                                        obj.sim_.verbosity_warnings);
        end

        function start_simulation(obj)
            % This method starts the CoppeliaSim simulation.
            obj.sim_.startSimulation();
        end

        function pause_simulation(obj)
            % This method pauses the CoppeliaSim simulation.
            obj.sim_.pauseSimulation();
        end

        function stop_simulation(obj)
            % This method stops the CoppeliaSim simulation.
            obj.sim_.stopSimulation()
        end

        function set_stepping_mode(obj, flag)
            % This method enables or disables the stepping mode
            % (formerly known as synchronous mode).
            obj.sim_.setStepping(flag);
        end

        function simulation_time = get_simulation_time(obj)
            % This method returns the simulation time.
            simulation_time =obj.sim_.getSimulationTime();
        end

        function trigger_next_simulation_step(obj) 
            % This method sends a trigger signal to the CoppeliaSim scene, 
            % which performs a simulation step when the stepping mode is used.
            obj.sim_.step();
        end

        function rtn = is_simulation_running(obj)
            % checks if the simulation is running. Returns true if the simulation 
            % is running. False otherwise.
           rtn = (obj.sim_.getSimulationState() > obj.sim_.simulation_paused);        
        end
 
        function rtn = get_simulation_state(obj)
            % this method returns the simulaton state. 
            % See more in https://manual.coppeliarobotics.com/en/simulation.htm
            % simulation_advancing = 16
            % simulation_advancing_abouttostop = 21
            % simulation_advancing_firstafterpause = 20
            % simulation_advancing_firstafterstop = 16
            % simulation_advancing_lastbeforepause = 19
            % simulation_advancing_lastbeforestop = 22
            % simulation_advancing_running = 17
            % simulation_paused = 8
            % simulation_stopped = 0
            rtn = obj.sim_.getSimulationState();
        end

        function set_status_bar_message(obj, message) 
            % This method sends a message to CoppeliaSim to be displayed 
            % in the status bar.    
            obj.set_status_bar_message_(message, obj.sim_.verbosity_undecorated);         
        end

        function handle = get_object_handle(obj, objectname)
            % get_object_handle gets the object handle from CoppeliaSim. 
            % If the handle is not included in the map, then the map is updated.
            additional_error_message = "";
            if (~obj.string_contain_first_slash_(objectname) && ...
                    obj.enable_deprecated_name_compatibility_ == false)      
                additional_error_message = "Did you mean   " + char(34) + '/' +objectname +  char(34) + '   ?';
            end


            try
                if (~obj.string_contain_first_slash_(objectname) && ...
                        obj.enable_deprecated_name_compatibility_ == true)
                    handle = obj.sim_.getObject('/'+objectname);
                    obj.update_map_('/'+objectname, handle);
                else
                    handle = obj.sim_.getObject(objectname);
                    obj.update_map_(objectname, handle);
                end
               
            catch ME
                
                disp("The object "  + char(34) + objectname  + char(34) + " does not exist in the " + ...
                    "current scene in CoppeliaSim. " + additional_error_message);
                rethrow(ME);
            end            
        end

        function handles = get_object_handles(obj, objectnames)
            % This method returns a cell that contains the handles
            % corresponding to the objectnames.
           arguments 
                obj
                objectnames cell
           end
           n = length(objectnames);
           handles = cell(1,n);
           for i=1:n
               handles{i} = obj.get_object_handle(objectnames{i});
           end
        end

        function rtn = get_map(obj)
            % For debugging
            rtn = obj.handles_map_;
        end

        function t = get_object_translation(obj, objectname)
          arguments 
                obj
                objectname string
           end
             position = obj.sim_.getObjectPosition(obj.get_handle_from_map_(objectname), ...
                 obj.sim_.handle_world);
             t = DQ([position{1},position{2},position{3}]);
        end

        function set_object_translation(obj, objectname, t)
           arguments 
                obj
                objectname string
                t DQ
           end 
            vec_t = vec3(t);
            position = {vec_t(1), vec_t(2), vec_t(3)};
            obj.sim_.setObjectPosition(obj.get_handle_from_map_(objectname), ...
                                       position,obj.sim_.handle_world);
        end

        function r = get_object_rotation(obj, objectname) 
           arguments 
                obj
                objectname string
           end
           rotation = obj.sim_.getObjectQuaternion(obj.get_handle_from_map_(objectname) ...
                        + obj.sim_.handleflag_wxyzquat, obj.sim_.handle_world);

           r = DQ([rotation{1},rotation{2},rotation{3}, rotation{4}]);
        end

        function set_object_rotation(obj, objectname, r)
           arguments 
                obj
                objectname string
                r DQ
           end
           vec_r = vec4(r);
           rotation = {vec_r(1), vec_r(2), vec_r(3), vec_r(4)};
           obj.sim_.setObjectQuaternion(obj.get_handle_from_map_(objectname) ...
                        + obj.sim_.handleflag_wxyzquat, rotation, obj.sim_.handle_world);
        end

        function x = get_object_pose(obj, objectname)
           arguments 
                obj
                objectname string
           end 
           t = obj.get_object_translation(objectname);
           r = obj.get_object_rotation(objectname);
           x = r + 0.5*DQ.E*t*r;
        end

        function set_object_pose(obj, objectname, h)
           arguments 
                obj
                objectname string
                h DQ
           end 
           vec_r = h.P().vec4();
           vec_t = h.translation().vec3(); 
           pose = {vec_t(1), vec_t(2), vec_t(3),vec_r(1), vec_r(2), vec_r(3), vec_r(4)};
           obj.sim_.setObjectPose(obj.get_handle_from_map_(objectname) + obj.sim_.handleflag_wxyzquat, ...
                     pose, obj.sim_.handle_world);
        end

        function theta = get_joint_position(obj, jointname)
           arguments 
                obj
                jointname string
           end  
           theta = double(obj.sim_.getJointPosition(obj.get_handle_from_map_(jointname)));

        end

        function q = get_joint_positions(obj, jointnames)
           arguments 
                obj
                jointnames cell
           end 
           n = length(jointnames);
           q = zeros(n,1);
           for i=1:n
                q(i) = obj.get_joint_position(jointnames{i});
           end
        end

        function set_joint_position(obj, jointname, angle_rad)
           arguments 
                obj
                jointname string
                angle_rad double
           end            
           obj.sim_.setJointPosition(obj.get_handle_from_map_(jointname), angle_rad);
        end

        function set_joint_positions(obj, jointnames, angles_rad)
           arguments 
                obj
                jointnames cell
                angles_rad double
           end 
           message = "Error in DQ_CoppeliaSimInterface.set_joint_positions: " + ...
                     "jointnames and angles_rad have incompatible sizes";
           obj.check_sizes_(jointnames, angles_rad, message);
           n = length(jointnames);
           for i=1:n
               obj.set_joint_position(jointnames{i}, angles_rad(i));
           end
        end

        function set_joint_target_position(obj, jointname, angle_rad)
           arguments 
                obj
                jointname string
                angle_rad double
           end
           obj.sim_.setJointTargetPosition(obj.get_handle_from_map_(jointname), angle_rad);
        end

        function set_joint_target_positions(obj, jointnames, angles_rad)
           arguments 
                obj
                jointnames cell
                angles_rad double
           end 
           message = "Error in DQ_CoppeliaSimInterface.set_joint_target_positions: " + ...
                     "jointnames and angles_rad have incompatible sizes";
           obj.check_sizes_(jointnames, angles_rad, message);   
           n = length(jointnames);
           for i=1:n
               obj.set_joint_target_position(jointnames{i}, angles_rad(i));
           end
        end

        function theta_dot = get_joint_velocity(obj, jointname)
           arguments 
                obj
                jointname string
           end 
            theta_dot = obj.sim_.getObjectFloatParam(obj.get_handle_from_map_(jointname), ...
                        obj.sim_.jointfloatparam_velocity);
        end

        function q_dot = get_joint_velocities(obj, jointnames)
           arguments 
                obj
                jointnames cell
           end  
           n = length(jointnames);
           q_dot = zeros(n,1);
           for i=1:n
               q_dot(i) = obj.get_joint_velocity(jointnames{i});
           end
        end

        function set_joint_target_velocity(obj, jointname, angle_rad_dot)
           arguments 
                obj
                jointname string
                angle_rad_dot double
           end   
           obj.sim_.setJointTargetVelocity(obj.get_handle_from_map_(jointname), angle_rad_dot);
        end

        function set_joint_target_velocities(obj, jointnames, angles_rad_dot)
           arguments 
                obj
                jointnames cell
                angles_rad_dot double
           end 
           message = "Error in DQ_CoppeliaSimInterface.set_joint_target_velocities: " + ...
                     "jointnames and angles_rad_dot have incompatible sizes";
           obj.check_sizes_(jointnames, angles_rad_dot, message);   
           n = length(jointnames);
           for i=1:n
               obj.set_joint_target_velocity(jointnames{i}, angles_rad_dot(i));
           end            
        end


 %%
        function enable_dynamics(obj, flag)
            arguments
                obj
                flag logical
            end
            obj.sim_.setBoolParam(obj.sim_.boolparam_dynamics_handling_enabled, flag);
        end 
    
        function set_joint_mode(obj, jointname, joint_mode)
           arguments 
                obj
                jointname string
                joint_mode JOINT_MODE
           end
           switch(joint_mode)
               case JOINT_MODE.KINEMATIC
                   jointMode = obj.sim_.jointmode_kinematic;
               case JOINT_MODE.DYNAMIC
                   jointMode = obj.sim_.jointmode_dynamic;
               case JOINT_MODE.DEPENDENT
                   jointMode = obj.sim_.jointmode_dependent;
           end
           obj.sim_.setJointMode(obj.get_handle_from_map_(jointname), jointMode, 0);
        end

        function set_joint_modes(obj, jointnames, joint_mode)
           arguments 
                obj
                jointnames cell
                joint_mode JOINT_MODE
           end
           for i=1:length(jointnames)
               obj.set_joint_mode(jointnames{i}, joint_mode);
           end
        end

        function set_joint_control_mode(obj, jointname, joint_control_mode)
           arguments 
                obj
                jointname string
                joint_control_mode JOINT_CONTROL_MODE
           end
           switch (joint_control_mode)
               case JOINT_CONTROL_MODE.FREE
                  control_mode = obj.sim_.jointdynctrl_free;
               case JOINT_CONTROL_MODE.FORCE
                   control_mode = obj.sim_.jointdynctrl_force;
               case JOINT_CONTROL_MODE.VELOCITY
                   control_mode = obj.sim_.jointdynctrl_velocity;
               case JOINT_CONTROL_MODE.POSITION
                    control_mode = obj.sim_.jointdynctrl_position;
               case JOINT_CONTROL_MODE.SPRING
                    control_mode = obj.sim_.ointdynctrl_spring;
               case JOINT_CONTROL_MODE.CUSTOM
                    control_mode = obj.sim_.jointdynctrl_callback;
               case JOINT_CONTROL_MODE.TORQUE
                    control_mode = obj.sim_.jointdynctrl_velocity;
           end
           obj.sim_.setObjectInt32Param(obj.get_handle_from_map_(jointname),...
                              obj.sim_.jointintparam_dynctrlmode, ...
                              control_mode);
        end

        function set_joint_control_modes(obj, jointnames, joint_control_mode)
           arguments 
                obj
                jointnames cell
                joint_control_mode JOINT_CONTROL_MODE
           end
           for i=1:length(jointnames)
               obj.set_joint_control_mode(jointnames{i}, joint_control_mode);
           end
        end
        
        function objectname = get_object_name(obj, handle) 
            objectname = obj.sim_.getObjectAlias(handle, 1);
            obj.update_map_(objectname, handle);
        end

        function objectnames = get_object_names(obj, handles)
            arguments 
               obj
               handles cell
            end
            n = length(handles);
            objectnames = cell(1,n);
            for i=1:n
                objectnames{i}=obj.get_object_name(handles{i});
            end
        end

        function jointnames = get_jointnames_from_base_objectname(obj, base_objectname)
           arguments 
                obj
                base_objectname string
           end
            base_handle = obj.get_handle_from_map_(base_objectname);
            jointhandles = obj.sim_.getObjectsInTree(base_handle,...
                                                obj.sim_.object_joint_type,0);
            jointnames = obj.get_object_names(jointhandles);
        end

        


       %% Deprecated methods
        function set_synchronous(obj, flag)
            % The synchronous mode is now called stepping mode. Consider 
            % using set_stepping_mode(flag) instead."
            obj.set_stepping_mode(flag);
        end


    end
end