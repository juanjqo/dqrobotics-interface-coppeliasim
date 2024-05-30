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
            if (k==1)
                rtn = true;
            else
                rtn = false;
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
               else
                   handle = obj.sim_.getObject(objectname);
               end
            catch ME
                
                disp("The object "  + char(34) + objectname  + char(34) + " does not exist in the " + ...
                    "current scene in CoppeliaSim. " + additional_error_message);
                rethrow(ME);
            
      
            end
            obj.update_map_(objectname, handle);
        end

        function handles = get_object_handles(obj, objectnames)
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


       %% Deprecated methods
        function set_synchronous(obj, flag)
            % The synchronous mode is now called stepping mode. Consider 
            % using set_stepping_mode(flag) instead."
            obj.set_stepping_mode(flag);
        end


    end
end