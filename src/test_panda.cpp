#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h"

#include <dqrobotics/solvers/DQ_PROXQPSolver.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimModels.h>

using namespace DQ_robotics;
using namespace Eigen;

int main()
{

    auto vi1 = std::make_shared<DQ_CoppeliaSimInterface>();

    try {
        vi1->connect("localhost", 23000, 200);//
        vi1->close_scene();
        vi1->load_from_model_browser("/robots/non-mobile/FrankaEmikaPanda.ttm",
                                     "/Franka", true, false);
        //vi1->remove_child_script_from_object("/Franka");
        vi1->draw_trajectory("/Franka/connection", 4, {1,0,0}, 100);
        vi1->plot_reference_frame("/x", DQ(1));
        vi1->show_map();
        vi1->remove_object("/Franka", true);
        std::cout<<" "<<std::endl;
        vi1->show_map();

        vi1->start_simulation();





        vi1->stop_simulation();
    } catch (const std::runtime_error& e)
    {
        std::cerr<<e.what()<<std::endl;

    }


}
