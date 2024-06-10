#include "RemoteAPIClient.h"
#include <iostream>


int main()
{
    try
    {
        RemoteAPIClient client;
        auto sim = client.getObject().sim();
        sim.startSimulation();
        auto handle = sim.getObject("/Franka/connection");
        auto scriptHandle = sim.addScript(sim.scripttype_childscript);
        sim.associateScriptWithObject(scriptHandle, handle);
        sim.executeScriptString("--Hello there", scriptHandle);
        sim.stopSimulation();

    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Caught a runtime error: " << e.what() << std::endl;
    }

    return 0;
}
