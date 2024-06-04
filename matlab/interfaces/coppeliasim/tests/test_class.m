clear all
close all
clc


vi = DQ_CoppeliaSimInterface();
vi.connect();

vi.start_simulation()


com = vi.get_center_of_mass("/Sphere", "BODY_FRAME");


vi.stop_simulation();


