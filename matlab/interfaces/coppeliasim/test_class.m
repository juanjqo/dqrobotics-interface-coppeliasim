clear all
close all
clc


vi = DQ_CoppeliaSimInterface();
vi.connect();
vi.set_stepping_mode(true);


phi = pi/4;
r = cos(phi/2) + DQ.k*sin(phi/2);


jointnames = vi.get_jointnames_from_base_objectname("/Franka");

vi.set_joint_modes(jointnames, "DYNAMIC");
vi.set_joint_control_modes(jointnames, "TORQUE");
vi.enable_dynamics(true);

q = [0.1 0 0 0 0 0 0]';

vi.start_simulation();
for i=1:100
    vi.set_joint_torques(jointnames, q);
    %vi.set_joint_position(jointnames{4}, 1.5)
    % q_dot = vi.get_joint_torques(jointnames)'
    vi.get_joint_torque(jointnames{1})
    vi.trigger_next_simulation_step();
end
v = vi.get_joint_position(jointnames{4})
map = vi.get_map();


vi.stop_simulation();

