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

        function newstr = remove_first_slash_from_string_(obj, str)            
            if obj.string_contain_first_slash_(str)
                 newstr = erase(str,"/");
            else
                 newstr = str;
            end
        end

        function check_sizes_(~, v1, v2, message)
            if (length(v1) ~= length(v2))
                error(message);
            end
        end

        function params = get_velocity_const_params_(obj)
            params =   [obj.sim_.shapefloatparam_init_velocity_a;
                        obj.sim_.shapefloatparam_init_velocity_b;
                        obj.sim_.shapefloatparam_init_velocity_g;
                        obj.sim_.shapefloatparam_init_velocity_x;
                        obj.sim_.shapefloatparam_init_velocity_y;
                        obj.sim_.shapefloatparam_init_velocity_z];
        end

        function status = load_model_(obj, path_to_filename, desired_model_name, remove_child_script)
            rtn = obj.sim_.loadModel(path_to_filename);
             if (rtn ~= -1)
                 obj.set_object_name(obj.get_object_name(rtn), obj.remove_first_slash_from_string_(desired_model_name));
                 if (remove_child_script)
                      obj.remove_child_script_from_object("/" +obj.remove_first_slash_from_string_(desired_model_name));
                 end
                 status = true;
             else
                status = false;
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

        function set_joint_torque(obj, jointname, torque)
           arguments 
                obj
                jointname string
                torque double
           end  
           angle_dot_rad_max = 10000.0;
           if (torque==0)      
                angle_dot_rad_max = 0.0;
           elseif (torque<0)
                angle_dot_rad_max = -10000.0;
           end
           handle = obj.get_handle_from_map_(jointname);
           obj.sim_.setJointTargetVelocity(handle, angle_dot_rad_max);
           obj.sim_.setJointTargetForce(handle, abs(torque));
        end

        function set_joint_torques(obj, jointnames, torques)
           arguments 
                obj
                jointnames cell
                torques double
           end
           message = "Error in DQ_CoppeliaSimInterface.set_joint_torques: " + ...
                     "jointnames and torques have incompatible sizes";
           obj.check_sizes_(jointnames, torques, message);   
           n = length(jointnames);
           for i=1:n
               obj.set_joint_torque(jointnames{i}, torques(i));
           end          
        end

        function  torque = get_joint_torque(obj, jointname)
           arguments 
                obj
                jointname string
           end 

           try
                torque = obj.sim_.getJointForce(obj.get_handle_from_map_(jointname));
           catch
               disp("This method has a bug in CoppeliaSim 4.6.0-rev18. " + ...
                     "https://forum.coppeliarobotics.com/viewtopic.php?p=40811#p40811");
               disp("Meanwhile, to avoid this error, don't read the force in the first simulation step.")
           end
        end

        function torques = get_joint_torques(obj, jointnames)
           arguments 
                obj
                jointnames cell
           end     

           n = length(jointnames);
           torques = zeros(n,1);
           for i=1:n
               torques(i) = obj.get_joint_torque(jointnames{i});
           end 
        end

        function objectname= get_object_name(obj, handle)
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

        function linknames = get_linknames_from_base_objectname(obj, base_objectname)
           arguments 
                obj
                base_objectname string
           end  
           base_handle = obj.get_handle_from_map_(base_objectname);
           shapehandles = obj.sim_.getObjectsInTree(base_handle,obj.sim_.object_shape_type, 0);
           linknames = obj.get_object_names(shapehandles);
        end


        function v = get_angular_and_linear_velocities(obj, objectname, reference)
           arguments 
                obj
                objectname string
                reference DQ_CoppeliaSimInterface_REFERENCE
           end 
           params = obj.get_velocity_const_params_();
           n = length(params);
           v = zeros(n,1);
           for i=1:n
               v(i) = obj.sim_.getObjectFloatParam(obj.get_handle_from_map_(objectname), params(i));
           end
           if reference == DQ_CoppeliaSimInterface_REFERENCE.BODY_FRAME
               x = obj.get_object_pose(objectname);
               r = x.P();
               w_b = r.conj()*DQ(v(1:3))*r;
              p_dot_b = r.conj()*DQ(v(4:6))*r;
              v = [vec3(w_b); vec3(p_dot_b)];
           end
        end


        function set_angular_and_linear_velocities(obj, objectname, w, p_dot, reference)
           arguments 
                obj
                objectname string
                w DQ
                p_dot DQ
                reference DQ_CoppeliaSimInterface_REFERENCE
           end  
           params = obj.get_velocity_const_params_();
           n = length(params);
           if reference == DQ_CoppeliaSimInterface_REFERENCE.ABSOLUTE_FRAME
                 w_a = w;
                 p_dot_a = p_dot;
                 v = [w_a.vec3();
                      p_dot_a.vec3()];
           else
                 x = obj.get_object_pose(objectname);
                 r = x.P();
                 w_a = r*w*r.conj();
                 p_dot_a = r*p_dot*r.conj();
                 v = [w_a.vec3();
                     p_dot_a.vec3()];
           end
           handle = obj.get_handle_from_map_(objectname);
           obj.sim_.resetDynamicObject(handle);
           for i=1:n
               obj.sim_.setObjectFloatParam(handle, params(i), v(i));
           end
        end


        function twist = get_twist(obj, objectname, reference)
           arguments 
                obj
                objectname string
                reference DQ_CoppeliaSimInterface_REFERENCE
           end      
           v = obj.get_angular_and_linear_velocities(objectname, reference);
           w = DQ(v(1:3));
           p_dot = DQ(v(4:6));
           x = obj.get_object_pose(objectname);
           twist =  w + DQ.E*(p_dot + cross(x.translation(), w));
           if (reference == DQ_CoppeliaSimInterface_REFERENCE.BODY_FRAME)
               twist =  x.conj()*twist*x;
           end
        end


        function set_twist(obj, objectname, twist, reference)
           arguments 
                obj
                objectname string
                twist DQ
                reference DQ_CoppeliaSimInterface_REFERENCE
           end   
           if (~is_pure(twist))
                error("Bad set_object_twist() call: Not a pure dual quaternion");
           end
           if (reference == DQ_CoppeliaSimInterface_REFERENCE.BODY_FRAME)
               obj.set_angular_and_linear_velocities(objectname, twist.P(), twist.D(),...
                                                  DQ_CoppeliaSimInterface_REFERENCE.BODY_FRAME);
           else
                x = obj.get_object_pose(objectname);
                obj.set_angular_and_linear_velocities(objectname, twist.P(), twist.D()-cross(x.translation(), twist.P()),...
                                          DQ_CoppeliaSimInterface_REFERENCE.ABSOLUTE_FRAME);
           end
        end


        function set_joint_mode(obj, jointname, joint_mode)
           arguments 
                obj
                jointname string
                joint_mode DQ_CoppeliaSimInterface_JOINT_MODE
           end
           switch(joint_mode)
               case DQ_CoppeliaSimInterface_JOINT_MODE.KINEMATIC
                   jointMode = obj.sim_.jointmode_kinematic;
               case DQ_CoppeliaSimInterface_JOINT_MODE.DYNAMIC
                   jointMode = obj.sim_.jointmode_dynamic;
               case DQ_CoppeliaSimInterface_JOINT_MODE.DEPENDENT
                   jointMode = obj.sim_.jointmode_dependent;
           end
           obj.sim_.setJointMode(obj.get_handle_from_map_(jointname), jointMode, 0);
        end


        function set_joint_modes(obj, jointnames, joint_mode)
           arguments 
                obj
                jointnames cell
                joint_mode DQ_CoppeliaSimInterface_JOINT_MODE
           end
           for i=1:length(jointnames)
               obj.set_joint_mode(jointnames{i}, joint_mode);
           end
        end


        function set_joint_control_mode(obj, jointname, joint_control_mode)
           arguments 
                obj
                jointname string
                joint_control_mode DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE
           end
           switch (joint_control_mode)
               case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.FREE
                  control_mode = obj.sim_.jointdynctrl_free;
               case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.FORCE
                   control_mode = obj.sim_.jointdynctrl_force;
               case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.VELOCITY
                   control_mode = obj.sim_.jointdynctrl_velocity;
               case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.POSITION
                    control_mode = obj.sim_.jointdynctrl_position;
               case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.SPRING
                    control_mode = obj.sim_.ointdynctrl_spring;
               case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.CUSTOM
                    control_mode = obj.sim_.jointdynctrl_callback;
               case DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE.TORQUE
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
                joint_control_mode DQ_CoppeliaSimInterface_JOINT_CONTROL_MODE
           end
           for i=1:length(jointnames)
               obj.set_joint_control_mode(jointnames{i}, joint_control_mode);
           end
        end


        function enable_dynamics(obj, flag)
            arguments
                obj
                flag logical
            end
            obj.sim_.setBoolParam(obj.sim_.boolparam_dynamics_handling_enabled, flag);
        end 

        function t = get_simulation_time_step(obj)
            t = obj.sim_.getFloatParam(obj.sim_.floatparam_simulation_time_step);
        end

        function set_simulation_time_step(obj, time_step)
            obj.sim_.setFloatParam(obj.sim_.floatparam_simulation_time_step, time_step);
        end

        function t = get_physics_time_step(obj)
            t = obj.sim_.getFloatParam(obj.sim_.floatparam_physicstimestep);
        end

        function set_physics_time_step(obj, time_step) 
            obj.sim_.setFloatParam(obj.sim_.floatparam_physicstimestep, time_step);
        end

        function set_dynamic_engine(obj, engine)
            arguments
                obj
                engine DQ_CoppeliaSimInterface_ENGINE
            end
            obj.sim_.setInt32Param(obj.sim_.intparam_dynamic_engine, engine);
        end 

        function set_gravity(obj, gravity)
            arguments
                obj
                gravity DQ
            end
            gravity_vec = gravity.vec3();
            g = {gravity_vec(1),gravity_vec(2), gravity_vec(3)};
            obj.sim_.setArrayParam(obj.sim_.arrayparam_gravity, g);
        end

        function gravity = get_gravity(obj) 
            g = obj.sim_.getArrayParam(obj.sim_.arrayparam_gravity);
            gravity = DQ([0, g{1}, g{2}, g{3}]);
        end


        function load_scene(obj, path_to_filename) 
             % * @brief DQ_CoppeliaSimInterface.load_scene loads a scene from your computer.
             % * @param path_to_filename the path to the scene. This string must containt
             % *        the file extension.
             % *
             % *        Example:
             % *
             % *        load_scene("/Users/juanjqo/git/space_robot/scenes/space_robot.ttt");
            arguments
                obj
                path_to_filename string
            end
            obj.sim_.loadScene(path_to_filename);
        end

        function save_scene(obj, path_to_filename) 
            obj.sim_.saveScene(path_to_filename);
        end

        function close_scene(obj)
            obj.sim_.closeScene();
        end

        function status = load_model(obj, path_to_filename, desired_model_name, load_model_only_if_missing,remove_child_script)
                % /**
                %  * @brief DQ_CoppeliaSimInterface::load_model loads a model to
                %  *        the scene.
                %  *
                %  * @param path_to_filename The path to the model.
                %  * @param desired_model_name The name you want for the loaded model.
                %  * @param load_model_only_if_missing If the model exists (with the same alias)
                %  *                                   the model is not loaded. (Default)
                %  * @param remove_child_script Remove the associated child script of the model
                %  *                            (Default)
                %  * @return A boolean flag. True if the model was loaded. False otherwise.
                %  */
                switch (nargin)
                   case 3
                       load_model_only_if_missing = true;
                       remove_child_script = true;
                   case 4  
                       remove_child_script = true;
                   case 5

                   otherwise
                       error("Wrong number of parameters");
                end
                if (load_model_only_if_missing == true)
                    if (~obj.object_exist_on_scene("/"+obj.remove_first_slash_from_string_(desired_model_name)))
                        status = obj.load_model_(path_to_filename, desired_model_name, remove_child_script);
                    else
                        status = true;
                    end
                else
                    status = obj.load_model_(path_to_filename, desired_model_name, remove_child_script);
                end
        end


        function status = load_from_model_browser(obj, path_to_filename, desired_model_name, load_model_only_if_missing, remove_child_script)
                % /**
                % * @brief DQ_CoppeliaSimInterface::load_from_model_browser loads a model from
                % *        the CoppeliaSim model browser.
                % *
                % *      Ex: load_from_model_browser("/robots/non-mobile/FrankaEmikaPanda.ttm",
                %                                    "/Franka");
                % * @param path_to_filename The path to the model relative to the model browser.
                % * @param desired_model_name The name you want for the loaded model.
                % * @param load_model_only_if_missing If the model exists (with the same alias)
                % *                                   the model is not loaded. (Default)
                % * @param remove_child_script Remove the associated child script of the model
                % *                            (Default)
                % * @return A boolean flag. True if the model was loaded. False otherwise.
                % */
                switch (nargin)
                   case 3
                       load_model_only_if_missing = true;
                       remove_child_script = true;
                   case 4  
                       remove_child_script = true;
                   otherwise
                       error("Wrong number of parameters");
                end
                resources_path = obj.sim_.getStringParam(obj.sim_.stringparam_resourcesdir);
                status = obj.load_model(resources_path + "/models" + path_to_filename, ...
                              desired_model_name, load_model_only_if_missing, remove_child_script);
        end
        

        function remove_child_script_from_object(obj, objectname)
            arguments
                obj
                objectname string
            end
            script_handle = obj.sim_.getScript(obj.sim_.scripttype_childscript,...
                                         obj.get_handle_from_map_(objectname));
            if (script_handle ~= -1)
                obj.sim_.removeScript(script_handle);
            end
        end


        function status = object_exist_on_scene(obj, objectname)

            options = {{"noError", false}};
            try 
                rtn = obj.sim_.getObject(objectname, options);
                if (rtn ~= -1)
                    status = true;
                else
                    status = false;
                end
            catch 
                status = false;
            end
        end

        function set_object_name(obj, current_object_name, new_object_name)
            arguments
                obj
                current_object_name string
                new_object_name string
            end
            obj.sim_.setObjectAlias(obj.get_handle_from_map_(current_object_name), new_object_name);
        end

        % function mass = get_mass(obj, object_name)
        % end
        % 
        % function get_center_of_mass
        % end
        % 
        % function get_inertia_matrix()
        % end




       %% Deprecated methods
        function set_synchronous(obj, flag)
            % The synchronous mode is now called stepping mode. Consider 
            % using set_stepping_mode(flag) instead."
            obj.set_stepping_mode(flag);
        end


    end
end