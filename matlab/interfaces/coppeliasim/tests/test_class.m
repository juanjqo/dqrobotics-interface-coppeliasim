clear all
close all
clc


vi = DQ_CoppeliaSimInterface();
vi.connect();
vi.close_scene();

vi.load_from_model_browser("/robots/non-mobile/FrankaEmikaPanda.ttm", "/Franka");

vi.start_simulation()


vi.stop_simulation();


