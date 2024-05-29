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
    //vi.start_simulation();

    vi.close_scene();
    vi.load_scene("/Users/juanjqo/git/space_robot/scenes/space_robot.ttt");
    vi.save_scene("/Users/juanjqo/git/space_robot/scenes/space_robot2.ttt");


    vi.stop_simulation();

}
