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

        std::vector<std::string> jointnames =
            vi.get_jointnames_from_base_objectname(std::string("/Franka"));

        for (auto &p:  jointnames)
        {
            std::cout<<p<<std::endl;
        }

        double t = 0.0;

        VectorXd q = vi.get_joint_positions(jointnames);
        std::cout<<"q: "<<q.transpose()<<std::endl;



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
        vi.set_object_pose("/ReferenceFrame[0]", vi.get_object_pose("/Franka/connection"));

        vi.show_map();
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Caught a runtime error: " << e.what() << std::endl;
    }

    return 0;
}
