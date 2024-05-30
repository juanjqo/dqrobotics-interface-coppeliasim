clear all
close all
clc


vi = DQ_CoppeliaSimInterface();
vi.connect();
vi.set_stepping_mode(true);
vi.start_simulation();

vi.set_object_translation("/Sphere", DQ([1,1,1]));

vi.stop_simulation();

