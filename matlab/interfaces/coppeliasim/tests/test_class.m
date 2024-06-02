clear all
close all
clc


vi = DQ_CoppeliaSimInterface();
vi.connect();
%vi.close_scene()
%vi.load_scene("/Users/juanjqo/git/space_robot/scenes/space_robot.ttt")

vi.load_from_model_browser("/robots/non-mobile/FrankaEmikaPanda.ttm", "/Franka");
vi.stop_simulation();

