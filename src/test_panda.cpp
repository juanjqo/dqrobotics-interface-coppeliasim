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
        //vi1->load_from_model_browser("/robots/non-mobile/FrankaEmikaPanda.ttm","/Franka", true, false);
        //vi1->remove_child_script_from_object("/Franka");
        //vi1->draw_trajectory("/Franka/connection", 4, {1,0,0}, 100);
        vi1->plot_reference_frame("/x", DQ(1));

        vi1->plot_plane("/plane", k_, k_,{0.2,0.2}, {1,0,0,0.5}, true);
        vi1->plot_line("/line", k_, k_);
        vi1->plot_sphere("/mysphere", 1*k_);
        vi1->plot_cylinder("/cylinder", k_,0.8*k_, {0.5,1.5}, {1,0,0,0.5}, true, 1);

        //vi1->remove_object("/Sphere", true);
        vi1->remove_plotted_object("/x");
        vi1->remove_plotted_object("/plane");
        vi1->remove_plotted_object("/line");
        vi1->remove_plotted_object("/mysphere");
        vi1->remove_plotted_object("/cylinder");
        //auto size_ = vi1->get_bounding_box_size("/line");
        //std::cout<<std::format("x: {}, y: {}, z: {}", size_.at(0),size_.at(1),size_.at(2))<<std::endl;
        vi1->show_map();
        vi1->show_created_handle_map();

        vi1->start_simulation();





        vi1->stop_simulation();
    } catch (const std::runtime_error& e)
    {
        std::cerr<<e.what()<<std::endl;

    }


}
