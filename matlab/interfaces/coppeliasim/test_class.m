clear all
close all
clc


vi = DQ_CoppeliaSimInterface();
vi.connect();
vi.set_stepping_mode(true);
vi.start_simulation();

objectname = "Franka";
handles = vi.get_object_handles({"/Franka", "/Jaco"})

vi.stop_simulation();

