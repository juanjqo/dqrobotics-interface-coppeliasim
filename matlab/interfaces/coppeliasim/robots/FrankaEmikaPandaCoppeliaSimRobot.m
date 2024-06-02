classdef FrankaEmikaPandaCoppeliaSimRobot < DQ_SerialCoppeliaSimRobot

    methods
        function obj = FrankaEmikaPandaCoppeliaSimRobot(robot_name, coppeliasim_interface_sptr)
            obj = obj@DQ_SerialCoppeliaSimRobot(robot_name, coppeliasim_interface_sptr);
        end
        
        function kin = kinematics(obj)
            kin = FrankaEmikaPandaRobot.kinematics();
            kin.set_reference_frame(obj.get_interface_sptr_().get_object_pose(obj.base_frame_name_));
            kin.set_base_frame(obj.get_interface_sptr_().get_object_pose(obj.base_frame_name_));
        end
    end
end

