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
    vi->close_scene();



    auto csmodels = DQ_CoppeliaSimModels(vi);
    csmodels.load_reference_frames({"/x", "xd", "x_e", "x_m"});
    //csmodels.load_panda("/Franka");

    csmodels.load_primitive(DQ_CoppeliaSimInterface::SPHEROID,
                            "/cone",
                            DQ(1),
                            {0.1,0.1,0.1},{1,0,0},0.5,false, true);

    csmodels.load_primitive(DQ_CoppeliaSimInterface::SPHEROID,
                            "/cone2",
                            1+0.5*E_*0.5*k_,
                            {0.1,0.1,0.1},{0,1,0},0.5,false, true);

    //vi->set_object_parent("/cone", "/Franka/connection", false);
    auto dist = vi->compute_distance("/cone", "/cone2", 1);

    //vi->draw_trajectory(DQ(0,1,1,0.5));//
    //vi->draw_trajectory(DQ(0,1,1,0.5));
    std::cout<<dist<<std::endl;


    vi->draw_trajectory("/cone");


    vi->start_simulation();





    vi->stop_simulation();

}
