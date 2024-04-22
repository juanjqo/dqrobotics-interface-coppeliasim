#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h"
#include<chrono>
#include<thread>

int main()
{
    try
    {
        DQ_CoppeliaSimInterface vi;
        vi.connect();
        vi.set_stepping_mode(true);
        vi.start_simulation();

        std::vector<std::string> jointnames = {"/Franka/joint", "/Franka/link2_resp/joint"};
        std::cout<<"Handle: "<<vi.get_object_handle("/Franka/joint")<<std::endl;
        std::vector<int> handles = vi.get_object_handles(jointnames);
        std::cout<<"Handle: "<<vi.get_object_handle("/Franka/joint")<<std::endl;

        DQ position = vi.get_object_translation("/ReferenceFrame[1]", "/ReferenceFrame[0]");
        std::cout<<"Position: "<<position<<std::endl;


        auto m = vi.get_map();
        for (const auto& p : m)
        {
            std::cout << '[' << p.first << "] = " << p.second << '\n';
        }
        double t = 0.0;



        while (t < 1.0)
        {
            //std::cout<<"status: "<<vi.is_simulation_running()<<" "
            //          <<vi.get_simulation_state()<<std::endl;
            t = vi.get_simulation_time();
            std::cout<<"Simulation time: "<<t<<std::endl;
            vi.trigger_next_simulation_step();

        }
        vi.stop_simulation();
        vi.get_simulation_state();
        //std::cout<<"status: "<<vi.is_simulation_running()<<" "
        //          <<vi.get_simulation_state()<<std::endl;

        vi.set_object_translation("/ReferenceFrame[0]", DQ(0, 0,0,0));
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Caught a runtime error: " << e.what() << std::endl;
    }

    return 0;
}
