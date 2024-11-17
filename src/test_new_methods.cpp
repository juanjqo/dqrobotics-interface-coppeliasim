#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimZmqInterface.h"

//#include <dqrobotics/solvers/DQ_PROXQPSolver.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/DQ.h>

using namespace DQ_robotics;
using namespace Eigen;

int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimZmqInterface>();
    vi->connect("192.168.50.16", 23000, 800);
    auto vi_exp = DQ_CoppeliaSimZmqInterface::experimental(vi);

    vi->close_scene();










    vi_exp.plot_reference_frame("/x2", 1+0.5*E_*0.3*k_, 1.5, {0.01, 0.1});
    vi_exp.plot_plane("/plane", k_, 0.5*k_);
    vi_exp.plot_line("/line", i_, -0.3*j_);
    vi_exp.plot_cylinder("/cyl", i_, 0.1*k_);


    vi->start_simulation();
    std::cout<<vi->get_object_pose("/x2")<<std::endl;


    vi->stop_simulation();

}
