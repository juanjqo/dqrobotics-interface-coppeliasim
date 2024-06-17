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
    auto vi2 = std::make_shared<DQ_CoppeliaSimInterface>();

    try {
        vi1->connect("localhost", 23000, 100);//
        vi2->connect("localhost", 23000, 100);

        vi1->start_simulation();

        vi2->plot_reference_frame("/x2", DQ(1));//
        std::cout<<vi1->get_object_pose("/x2")<<std::endl;;



        vi1->stop_simulation();
    } catch (const std::runtime_error& e)
    {
        std::cerr<<e.what()<<std::endl;

    }


}
