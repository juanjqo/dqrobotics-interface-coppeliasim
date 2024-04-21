#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h"


int main()
{
    try
    {
        DQ_CoppeliaSimInterface vi;
        vi.connect();
        vi.set_stepping_mode(true);
        vi.start_simulation();
        double t = 0.0;

        while (t < 4.0)
        {
            std::cout<<"status: "<<vi.is_simulation_running()<<std::endl;
            t = vi.get_simulation_time();
            std::cout<<"Simulation time: "<<t<<std::endl;
            vi.trigger_next_simulation_step();

        }
        vi.stop_simulation();
        std::cout<<"status: "<<vi.is_simulation_running()<<std::endl;


    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Caught a runtime error: " << e.what() << std::endl;
    }

    return 0;
}
