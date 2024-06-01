#include "RemoteAPIClient.h"
#include <iostream>


int main()
{
    try
    {
        RemoteAPIClient client;
        auto sim = client.getObject().sim();
        sim.startSimulation();
        auto handle = sim.getObject("/Franka");
        auto shapehandles = sim.getObjectsInTree(handle, sim.object_shape_type, 0);
        for (auto& v: shapehandles)
            std::cout<<v<<std::endl;
        sim.stopSimulation();

    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Caught a runtime error: " << e.what() << std::endl;
    }

    return 0;
}
