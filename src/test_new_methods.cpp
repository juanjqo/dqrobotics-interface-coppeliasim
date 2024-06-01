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
    auto jointnames = vi.get_jointnames_from_base_objectname("/Franka/joint");
    auto torques = vi.get_joint_torques(jointnames);
    std::cout<<torques<<std::endl;



    vi.stop_simulation();

}
