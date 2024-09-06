#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h"
#include<chrono>
#include<thread>

int main()
{
    try
    {
        DQ_CoppeliaSimInterface vi;
        vi.connect("localhost", 23000);
        vi.set_stepping_mode(true);
        vi.set_engine(DQ_CoppeliaSimInterface::ENGINE::MUJOCO);

        vi.set_mujoco_global_impratio(1);
        vi.set_mujoco_global_wind({0,0,0});
        vi.set_mujoco_global_overridesolimp({1,2,3,4,5});
        vi.set_gravity(DQ(0));
        vi.set_mujoco_joint_stiffness("Revolute_joint", 0.45);
        vi.set_mujoco_joint_armature("Revolute_joint", 0.1);

        std::cout<<" Engine: "<<vi.get_engine()<<std::endl;
        vi.set_simulation_time_step(0.05);
        vi.set_physics_time_step(0.005);
        std::cout<<"Simulation time step: "<<vi.get_simulation_time_step()
                  <<" Physics time step: "<<vi.get_physics_time_step()<<std::endl;

        std::cout<<"gravity: "<<vi.get_gravity()<<std::endl;
        vi.start_simulation();

        for (int i=0;i<1000;i++)
            vi.trigger_next_simulation_step();

        vi.stop_simulation();

    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Caught a runtime error: " << e.what() << std::endl;
    }

    return 0;
}
