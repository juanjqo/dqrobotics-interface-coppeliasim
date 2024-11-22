#include "RemoteAPIClient.h"
#include <iostream>


int main()
{
    RemoteAPIClient client;
    auto sim = client.getObject().sim();
    try
    {
        sim.startSimulation();
        sim.setEngineFloatParam(sim.mujoco_joint_damping, sim.getObject("/Revolute_joint"), 0.3);
        sim.stopSimulation();

    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Caught a runtime error: " << e.what() << std::endl;
        sim.stopSimulation();
    }
    return 0;
}
