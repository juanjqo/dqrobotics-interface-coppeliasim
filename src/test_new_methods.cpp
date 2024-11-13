#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h"

//#include <dqrobotics/solvers/DQ_PROXQPSolver.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimModels.h>

using namespace DQ_robotics;
using namespace Eigen;

int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimInterface>();
    vi->connect("localhost", 23000);
    vi->close_scene();


    vi->plot_reference_frame("/x", DQ(1));
    vi->plot_plane("/plane", k_, 0.5*k_);
    vi->plot_line("/line", i_, -0.3*j_);
    vi->plot_cylinder("/cyl", i_, 0.1*k_);



    vi->start_simulation();





    vi->stop_simulation();

}
