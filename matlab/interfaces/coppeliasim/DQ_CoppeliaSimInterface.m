classdef DQ_CoppeliaSimInterface < handle

    properties (Access = private)
        handles_map_;
        client_created_;
        client_;
        sim_;
    end
    properties
        
    
    end

    methods (Access = private)

        function create_client(obj, host, rpcPort, cntPort, verbose_)
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
    end

    methods
        function obj = DQ_CoppeliaSimInterface()
            obj.client_created_ = false;
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

            obj.create_client(host, rpcPort, cntPort, verbose_);
            obj.client_created_ = true;
            rtn = true;
        end

        function start_simulation(obj)
            % This method starts the CoppeliaSim simulation.
            obj.sim_.startSimulation();
        end

        function stop_simulation(obj)
            % This method stops the CoppeliaSim simulation.
            obj.sim_.stopSimulation()
        end

        function simulation_time = get_simulation_time(obj)
            % This method returns the simulation time.
            simulation_time =obj.sim_.getSimulationTime();
        end

        function set_stepping_mode(obj, flag)
            % This method enables or disables the stepping mode
            % (formerly known as synchronous mode).
            obj.sim_.setStepping(flag);
        end

        function set_synchronous(obj, flag)
            % The synchronous mode is now called stepping mode. Consider 
            % using set_stepping_mode(flag) instead."
            obj.set_stepping_mode(flag);
        end

        function trigger_next_simulation_step(obj) 
            % This method sends a trigger signal to the CoppeliaSim scene, 
            % which performs a simulation step when the stepping mode is used.
            obj.sim_.step();
        end
    end
end