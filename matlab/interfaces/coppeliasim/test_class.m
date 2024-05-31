clear all
close all
clc


vi = DQ_CoppeliaSimInterface();
vi.connect();
vi.set_stepping_mode(true);
vi.start_simulation();

phi = pi/4;
r = cos(phi/2) + DQ.k*sin(phi/2);

vi.set_object_pose("/Sphere", DQ(1))


vi.stop_simulation();

