#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h"

#include <dqrobotics/solvers/DQ_PROXQPSolver.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimModels.h>

using namespace DQ_robotics;
using namespace Eigen;

int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimInterface>();
    vi->connect("localhost", 23000);


    auto csmodels = DQ_CoppeliaSimModels(vi);
    csmodels.load_reference_frames({"x", "xd", "x_e", "x_m"});
    csmodels.load_panda("/Franka");

    vi->start_simulation();



    vi->stop_simulation();

}
