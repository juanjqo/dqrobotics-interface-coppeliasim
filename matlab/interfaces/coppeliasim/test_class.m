clear all
close all
clc


vi = DQ_CoppeliaSimInterface();
vi.connect();
vi.set_stepping_mode(false);


phi = pi/4;
r = cos(phi/2) + DQ.k*sin(phi/2);

vi.set_object_pose("/Sphere", DQ(1))

jointnames = vi.get_jointnames_from_base_objectname("/Franka");

vi.set_joint_modes(jointnames, "KINEMATIC");
vi.set_joint_control_modes(jointnames, "POSITION");
vi.enable_dynamics(false);

q = [0.9 0 0 -1.5 0 0 0]';

vi.start_simulation();
for i=1:100
    vi.set_joint_positions(jointnames, q);
    %vi.set_joint_position(jointnames{4}, 1.5)
    qr = vi.get_joint_positions(jointnames)'
end
v = vi.get_joint_position(jointnames{4})
map = vi.get_map();


vi.stop_simulation();

