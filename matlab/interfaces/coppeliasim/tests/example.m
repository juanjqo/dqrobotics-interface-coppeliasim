clear all;
close all;
clc;

include_namespace_dq

vi = DQ_CoppeliaSimInterface();
vi.disconnect_all();
vi.connect();
vi.set_synchronous(true);
vi.start_simulation();


jointnames = vi.get_jointnames_from_base_objectname("/LBR4p");

%---------------------Robot definition--------------------------%
LBR4p_DH_theta = [0, 0, 0, 0, 0, 0, 0];
LBR4p_DH_d = [0.200, 0, 0.4, 0, 0.39, 0, 0];
LBR4p_DH_a = [0, 0, 0, 0, 0, 0, 0];
LBR4p_DH_alpha = [pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, 0];
LBR4p_DH_type = double(repmat(DQ_JointType.REVOLUTE,1,7));
LBR4p_DH_matrix = [LBR4p_DH_theta;
                    LBR4p_DH_d;
                    LBR4p_DH_a;
                    LBR4p_DH_alpha
                    LBR4p_DH_type];

robot = DQ_SerialManipulatorDH(LBR4p_DH_matrix);
robot.set_effector(1+0.5*DQ.E*DQ.k*0.07);

n = robot.get_dim_configuration_space();
qmin = [-pi -pi -pi/2 -pi -pi/2 -pi -pi/2]';
qmax = [pi pi pi/2 pi pi/2 pi pi/2]';

try           
    xbase = vi.get_object_pose(jointnames{1});
    robot.set_reference_frame(xbase);
    robot.set_base_frame(xbase);
    vi.set_object_pose('base', xbase);
    %----------------------------------------------------------------
    %---------------------Controller definition----------------------%
    qp_solver = DQ_QuadprogSolver();
    controller = DQ_ClassicQPController(robot, qp_solver);
    controller.set_gain(0.2);
    controller.set_damping(0.05);
    controller.set_control_objective(ControlObjective.DistanceToPlane)
    
    xplane = vi.get_object_pose('target_plane');
    plane_d = Adsharp(xplane,-k_);
    controller.set_target_primitive(plane_d);
    controller.set_stability_threshold(0.00001);
    desired_distance_to_target_plane = 0;
    %----------------------------------------------------------------
    %  Plane constraints
    safe_distance = 0.1;
    wall_plane  = Adsharp(vi.get_object_pose('wall_plane'),  k_); 
    floor_plane = Adsharp(vi.get_object_pose('floor_plane'), k_);
    back_plane  = Adsharp(vi.get_object_pose('back_plane'),  k_);
    planes = {wall_plane, floor_plane, back_plane};
    %----------------------------------------------------------------

    j=1;
    while ~controller.system_reached_stable_region()
        %---------------------------------------------------------------%
        % This command performs a simulation step in CoppeliaSim. This step
        % is required when you are working in synchronous mode.
        vi.trigger_next_simulation_step();
        
        % Waits until the simulation step is finished.
        vi.wait_for_simulation_step_to_end();
        %---------------------------------------------------------------%

        q =  vi.get_joint_positions(jointnames);
        x_pose = robot.fkm(q);
        p = translation(x_pose); 
        J_pose = robot.pose_jacobian(q); 
        J_trans = robot.translation_jacobian(J_pose,x_pose);
        
        for i=1:size(planes,2)
            Jdists(i,:) = robot.point_to_plane_distance_jacobian(J_trans, p, planes{i});
            dists(i)    = DQ_Geometry.point_to_plane_distance(p, planes{i}) -safe_distance;
        end

        A = [-Jdists; -eye(n); eye(n)];
        b = [dists'; (q-qmin); -(q-qmax)];
        controller.set_inequality_constraint(A,b);
        u = controller.compute_setpoint_control_signal(q, ...
                                        desired_distance_to_target_plane);

        %---------------------------------------------------------------%
        % Set the target joint velocities in CoppeliaSim. This command is
        % required when you are working in synchronous mode and the robot 
        % joints are set in velocity control mode.
        vi.set_joint_target_velocities(jointnames, u);
        %---------------------------------------------------------------%

        %---------------------------------------------------------------%
        %----Set the effector frame object on CoppeliaSim to check if
        %----the kinematic model is matching. This step is optional.
        vi.set_object_pose('effector', robot.fkm(q))
        %---------------------------------------------------------------%
        
        norm(controller.get_last_error_signal())


        u_log(:,j) = u;
        q_dot(:,j) = vi.get_joint_velocities(jointnames);
        j = j+1;
    end
    % Before closing the connection to CoppeliaSim, 
    % make sure that the last command sent out had time to arrive.
    vi.wait_for_simulation_step_to_end();
    vi.stop_simulation();
    vi.disconnect();
    show_joint_velocities(u_log, q_dot);

catch ME
    vi.stop_simulation();
    vi.disconnect();
    rethrow(ME)
end