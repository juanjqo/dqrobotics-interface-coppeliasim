clear all
close all
clc


vi = DQ_CoppeliaSimInterface();
vi.connect();
%vi.close_scene()
vi.load_from_model_browser("/robots/non-mobile/UR5.ttm", "/UR5");

robot = URXCoppeliaSimRobot("/UR5", vi,"UR5");
robot_model = robot.kinematics();

vi.enable_dynamics(true);
robot.set_operation_modes("DYNAMIC", "POSITION");
vi.start_simulation()

for i=1:50
    q = robot.get_configuration_space_positions();
    x = robot_model.fkm(q);
    vi.set_object_pose("/ReferenceFrame", x)
    u = [1.5         0         0   -1.5708         0    1.5708];
    robot.set_control_inputs(u);
end



vi.stop_simulation();
x.translation()

