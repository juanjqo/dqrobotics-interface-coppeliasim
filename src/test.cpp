#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.h>
#include<chrono>
#include<thread>

int main()
{
    try
    {
        DQ_CoppeliaSimInterfaceZMQExperimental vi;
        vi.connect("localhost", 23000);
        vi.close_scene();
        vi.set_stepping_mode(true);
        vi.set_engine(DQ_CoppeliaSimInterfaceZMQExperimental::ENGINE::MUJOCO);

        //vi.set_mujoco_global_impratio(1);
        //vi.set_mujoco_global_wind({0,0,0});
        //vi.set_mujoco_global_overridesolimp({1,2,3,4,5});
        //vi.set_gravity(DQ(0));
        //vi.set_mujoco_joint_stiffness("Revolute_joint", 0.45);
        //vi.set_mujoco_joint_armature("Revolute_joint", 0.1);
        std::string objectname = "/MyBox";
        auto rtn = vi.add_primitive(DQ_CoppeliaSimInterfaceZMQExperimental::PRIMITIVE::CUBOID, objectname, {0.3,0.3,0.3});
        vi.set_engine(DQ_CoppeliaSimInterfaceZMQExperimental::ENGINE::MUJOCO);
        vi.set_object_as_static(objectname, false);
        vi.set_object_as_respondable(objectname, true);
        vi.set_object_translation(objectname, 0.3*k_);
        //vi.set_mujoco_body_friction("/box", {0,0,0});

        vi.set_mujoco_body_friction(objectname, {0.4, 0.005, 0.0001});

        std::cout<<" Engine: "<<vi.get_engine()<<std::endl;
        //vi.set_simulation_time_step(0.05);
        //vi.set_physics_time_step(0.005);
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
