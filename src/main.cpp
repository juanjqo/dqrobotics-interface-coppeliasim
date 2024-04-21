#include "RemoteAPIClient.h"
#include <iostream>
#include <iomanip>

json myFunc(const json& input)
{
    std::cout << pretty_print(input) << "\n\n";
    return 21;
}
std::function<json(const json &)> myCallback = myFunc;

int main()
{
    try
    {
        RemoteAPIClient client;
        auto sim = client.getObject().sim();

        //sim.setStepping(true);

        //sim.startSimulation();
        //double t = 0.0;
        /*
            do
        {
            t = sim.getSimulationTime();
            printf("Simulation time: %.2f [s]\n", t);
            sim.step();
        } while (t < 3.0);
*/
        sim.stopSimulation();


    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "Caught a runtime error: " << e.what() << std::endl;
    }

    return 0;
}
