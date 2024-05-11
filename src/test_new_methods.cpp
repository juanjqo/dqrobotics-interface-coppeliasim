#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h"
#include<chrono>
#include<thread>
#include <dqrobotics/solvers/DQ_PROXQPSolver.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/DQ.h>

using namespace DQ_robotics;
using namespace Eigen;

int main()
{
    DQ_CoppeliaSimInterface vi;
    vi.connect("localhost", 23000);
    vi.start_simulation();

    auto com  = vi.get_center_of_mass("/link7_resp",
                                     DQ_CoppeliaSimInterface::BODY_FRAME);
    std::cout<<"com: "<<com<<std::endl;


    vi.stop_simulation();

}
