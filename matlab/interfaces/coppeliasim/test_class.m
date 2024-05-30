clear all
close all
clc


vi = DQ_CoppeliaSimInterface();
vi.connect();
vi.set_stepping_mode(true);
vi.start_simulation();
for i=1:5
    t = vi.get_simulation_time()
    vi.trigger_next_simulation_step();
end

vi.stop_simulation();

