#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimZmqInterface.h>
#include <dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimZmqRobot.h>

using namespace DQ_robotics;
using namespace Eigen;

VectorXd compute_control_signal(const MatrixXd& J,
                                const VectorXd& q,
                                const double& damping,
                                const double& gain,
                                const VectorXd& task_error);

int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimZmqInterface>();
    vi->connect();

    //To enable experimental methods
    auto vi_exp = std::make_shared<DQ_CoppeliaSimZmqInterface::experimental>(vi);

    // Load the models only if they are not already on the scene.
    vi_exp->load_from_model_browser("/robots/non-mobile/UR5.ttm", "/UR5");
    vi_exp->load_from_model_browser("/other/reference frame.ttm", "/Current_pose");
    vi_exp->load_from_model_browser("/other/reference frame.ttm", "/Desired_pose");
    vi->start_simulation();

    auto robot = URXCoppeliaSimZmqRobot("/UR5", vi, URXCoppeliaSimZmqRobot::MODEL::UR5);
    auto robot_model = robot.kinematics();
    robot.set_robot_as_visualization_tool();

    auto q = robot.get_configuration_space_positions();
    double gain = 10;
    double T = 0.001;
    double damping = 0.01;

    auto xd = robot_model.fkm(((VectorXd(6) <<  0.5, 0, 1.5, 0, 0, 0).finished()));
    vi->set_object_pose("/Desired_pose", xd);

    for (int i=0; i<300; i++)
    {
        auto x = robot_model.fkm(q);
        vi->set_object_pose("/Current_pose", x);
        auto J =  robot_model.pose_jacobian(q);
        auto Jt = robot_model.translation_jacobian(J, x);
        auto task_error = (x.translation()-xd.translation()).vec4();
        auto u = compute_control_signal(Jt, q, damping, gain, task_error);
        q = q + T*u;
        robot.set_control_inputs(q);
        std::cout<<"error: "<<task_error.norm()<<std::endl;
    }
    vi->stop_simulation();
}

VectorXd compute_control_signal(const MatrixXd& J,
                                const VectorXd& q,
                                const double& damping,
                                const double& gain,
                                const VectorXd& task_error)
{
    VectorXd u = (J.transpose()*J + damping*damping*MatrixXd::Identity(q.size(), q.size())).inverse()*
                 J.transpose()*(-gain*task_error);
    return u;
}
