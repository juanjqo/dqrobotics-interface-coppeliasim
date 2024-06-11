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


    vi->set_dynamic_engine(DQ_CoppeliaSimInterface::ENGINE::MUJOCO);
    auto csmodels = DQ_CoppeliaSimModels(vi);
    csmodels.load_reference_frames({"/x", "xd", "x_e", "x_m"});
    csmodels.load_panda("/Franka");

    csmodels.load_primitive(DQ_CoppeliaSimInterface::PRIMITIVE::SPHEROID,
                            "/cone",
                            DQ(1),
                            {0.1,0.1,0.1},{1,0,0,0.5},false, true);

    csmodels.load_primitive(DQ_CoppeliaSimInterface::PRIMITIVE::SPHEROID,
                            "/cone2",
                            1+0.5*E_*0.5*k_,
                            {0.1,0.1,0.1},{0,1,0,0.5},false, true);


    //vi->set_object_parent("/cone", "/Franka/connection", false);
    //auto dist = vi->compute_distance("/cone", "/cone2", 1);

    //vi->draw_trajectory(DQ(0,1,1,0.5));//
    //vi->draw_trajectory(DQ(0,1,1,0.5));
    //std::cout<<dist<<std::endl;
    //vi->set_object_pose("/xd", 1+0.5*E_*1*k_);

    DQ xd = 1+0.5*E_*1*k_;
    DQ r = xd.P();
    DQ plane = r*k_*r.conj() + E_*0.5;
    vi->add_plane("/plane",
                  plane.P(),
                  xd.translation(), {2,2}, {1,0,0,0.5}, true, 0.5);

    vi->add_line("line", plane.P(), xd.translation(), {0.01,1.5}, {1,0,0,0.5}, true, 1);


    //vi->draw_trajectory("/cone");


    vi->start_simulation();





    vi->stop_simulation();

}
