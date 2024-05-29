classdef DQ_CoppeliaSimInterface < handle

    properties (Access = private)
        handles_map_;
        client_created_;
        client_;
        sim_;
        %client_ = RemoteAPIClient();
        %sim_ = client_.require('sim');
    end
    properties
        
    
    end

    methods (Access = private)

        function create_client(obj, host, rpcPort, cntPort, verbose_)
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
        function connect(obj, host, rpcPort, cntPort, verbose_)

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

                
               otherwise
                  error('Wrong arguments');
            end
            obj.create_client(host, rpcPort, cntPort, verbose_);
            obj.client_created_ = true;

        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end