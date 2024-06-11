#include "RemoteAPIClient.h"
#include <iostream>


int main()
{
    RemoteAPIClient client;
    auto sim = client.getObject().sim();
    try
    {
        sim.startSimulation();
        auto handle = sim.getObject("/Franka/connection");
        auto scriptHandle = sim.addScript(sim.scripttype_childscript);
        sim.associateScriptWithObject(scriptHandle, handle);
        sim.initScript(scriptHandle);
        sim.executeScriptString("print('Hello there')",
                                sim.getScript(sim.scripttype_sandboxscript));
        sim.stopSimulation();

    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Caught a runtime error: " << e.what() << std::endl;
        sim.stopSimulation();
    }

    return 0;
}
