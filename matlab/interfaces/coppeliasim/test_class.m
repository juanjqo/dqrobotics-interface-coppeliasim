clear all
close all
clc


vi = DQ_CoppeliaSimInterface();
vi.connect();
%vi.close_scene()
%vi.load_scene("/Users/juanjqo/git/space_robot/scenes/space_robot.ttt")

str = "Franka";
newstr = vi.remove_first_slash_from_string(str)
vi.stop_simulation();

