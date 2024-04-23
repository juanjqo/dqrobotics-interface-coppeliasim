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
        vi.set_dynamic_engine(DQ_CoppeliaSimInterface::MUJOCO);
        std::cout<<"Simulation time step: "<<vi.get_simulation_time_step()
                 <<" Physics time step: "<<vi.get_physics_time_step()<<std::endl;
        vi.set_simulation_time_step(0.05);
        vi.set_physics_time_step(0.005);
        std::cout<<"Simulation time step: "<<vi.get_simulation_time_step()
                  <<" Physics time step: "<<vi.get_physics_time_step()<<std::endl;

        std::cout<<"gravity: "<<vi.get_gravity()<<std::endl;
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

        VectorXd u = VectorXd::Zero(7);
        u << 0.1, 0, 0, 0, 0, 0, 0;

        vi.set_joint_modes(jointnames, DQ_CoppeliaSimInterface::DYNAMIC);
        vi.set_joint_control_modes(jointnames, DQ_CoppeliaSimInterface::VELOCITY);
        vi.enable_dynamics(true);

        //for(int i=0; i<100;i++)
            //vi.set_joint_positions(jointnames, u);
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        //vi.pause_simulation();


        while (t < 5.0)
        {
            //std::cout<<"status: "<<vi.is_simulation_running()<<" "
            //          <<vi.get_simulation_state()<<std::endl;
            t = vi.get_simulation_time();
            std::cout<<"Simulation time: "<<t<<std::endl;
            vi.set_joint_target_velocities(jointnames, u);
            std::cout<<"joint vel: "<<vi.get_joint_velocities(jointnames).transpose()<<std::endl;
            vi.trigger_next_simulation_step();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

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
