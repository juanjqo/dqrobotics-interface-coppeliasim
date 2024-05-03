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
    vi.set_stepping_mode(true);
    vi.set_dynamic_engine(DQ_CoppeliaSimInterface::BULLET);
    //vi.enable_dynamics(false);
    vi.set_gravity(DQ(0));
    vi.start_simulation();

    double T = 0.01;
    double lambda = 1;

    std::string robotname = std::string("/Sphere");

    DQ xd = vi.get_object_pose("/xdesired");
    DQ x  = vi.get_object_pose(robotname);


    DQ xerror = x.conj()*xd;

    //while (vec6(log(xerror)).norm() > 0.001)
    for (int i=0; i<200; i++)
    {
        x  = vi.get_object_pose(robotname);
        xd = vi.get_object_pose("/xdesired");
        xerror = x.conj()*xd;
        auto J = 0.5*haminus8(x);
        VectorXd uvec = pinv(J)*(-lambda*C8()*haminus8(xd.conj())*Q8(xerror)*log(xerror).vec6());
        DQ u = DQ(uvec);
        //x = exp(T/2*u)*x;
        //vi.set_object_pose("/coffee_drone",x);
        std::cout<<"u: "<<u.vec6().transpose()<<std::endl;

        vi.set_object_twist(robotname,u);
        vi.trigger_next_simulation_step();
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));


        std::cout<<"Error: "<<vec6(log(xerror)).norm()<<"  i: "<<i<<std::endl;
    }

    vi.stop_simulation();

}
