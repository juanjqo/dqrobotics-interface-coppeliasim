classdef URXCoppeliaSimRobot < DQ_SerialCoppeliaSimRobot
    properties (Access = private)
        model_ URXCoppeliaSimRobot_MODEL
    end

    methods (Access = private)
        function raw_dh_matrix = get_dh_matrix_(obj, model)
            arguments
                obj
                model URXCoppeliaSimRobot_MODEL
            end
            switch (model)
                case URXCoppeliaSimRobot_MODEL.UR5
                    raw_dh_matrix = [-pi/2, -pi/2, 0, -pi/2, 0, 0;
                        0.089159-0.02315, 0, 0, 0.10915, 0.09465, 0.0823;
                        0, -0.425, -0.39225, 0, 0, 0;
                        pi/2,0,0,pi/2,-pi/2,0;
                        repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,6)];
            
                otherwise
                   raw_dh_matrix = Zero(0,0);
    
            end
        end

    end

    methods
        function obj = URXCoppeliaSimRobot(robot_name, coppeliasim_interface_sptr, model)
            arguments
                robot_name string
                coppeliasim_interface_sptr DQ_CoppeliaSimInterface
                model URXCoppeliaSimRobot_MODEL
            end
            obj = obj@DQ_SerialCoppeliaSimRobot(robot_name, coppeliasim_interface_sptr);
            obj.model_ = model;
        end
        
        function kin = kinematics(obj)
            kin =  DQ_SerialManipulatorDH(obj.get_dh_matrix_(obj.model_));
            obj.base_frame_name_
            kin.set_reference_frame(obj.get_interface_sptr_().get_object_pose(obj.base_frame_name_));
            kin.set_base_frame(obj.get_interface_sptr_().get_object_pose(obj.base_frame_name_));
        end
    end
end
