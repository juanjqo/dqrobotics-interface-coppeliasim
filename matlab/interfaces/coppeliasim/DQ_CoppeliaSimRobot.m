classdef DQ_CoppeliaSimRobot < handle    
    properties (Access = private)
        robot_name_ string
        coppeliasim_interface_sptr_ DQ_CoppeliaSimInterface
    end
    
    methods (Access = protected)
        function obj = DQ_CoppeliaSimRobot(robot_name, coppeliasim_interface_sptr)
            obj.robot_name_ = robot_name;
            obj.coppeliasim_interface_sptr_ = coppeliasim_interface_sptr;
        end
        
        function rtn = get_interface_sptr_(obj)
            rtn = obj.coppeliasim_interface_sptr_;
        end
    end
end

