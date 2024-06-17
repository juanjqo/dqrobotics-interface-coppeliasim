#include "RemoteAPIClient.h"
#include <iostream>


int main()
{
    RemoteAPIClient client;
    auto sim = client.getObject().sim();
    try
    {
        sim.startSimulation();
        auto handle = sim.getObject("/Franka");
        std::vector<int64_t> scripthandles = sim.getObjectsInTree(handle,
                                                                sim.appobj_script_type,
                                                                0);
        std::cout<<"Number of scripts: "<<scripthandles.size()<<std::endl;
        sim.stopSimulation();

    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Caught a runtime error: " << e.what() << std::endl;
        sim.stopSimulation();
    }

    return 0;
}
