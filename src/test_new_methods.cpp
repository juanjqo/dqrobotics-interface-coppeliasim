#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimZmqInterface.h"

//#include <dqrobotics/solvers/DQ_PROXQPSolver.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimZmqModels.h>

using namespace DQ_robotics;
using namespace Eigen;

int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimZmqInterface>();
    vi->connect("192.168.50.16", 23000, 800);
    vi->close_scene();


    vi->plot_reference_frame("/x", DQ(1));
    vi->plot_plane("/plane", k_, 0.5*k_);
    vi->plot_line("/line", i_, -0.3*j_);
    vi->plot_cylinder("/cyl", i_, 0.1*k_);



    vi->start_simulation();


    std::cout<<vi->get_object_pose("/x")<<std::endl;

    vi->stop_simulation();

}
