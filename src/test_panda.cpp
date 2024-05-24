#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>
#include <dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimRobot.h>

using namespace DQ_robotics;
using namespace Eigen;



#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <thread>

using namespace Eigen;

/**
 * @brief This function computes the inertia tensor mean
 * @param inertia_list Matrix that contains several readings of the inertia tensor of all robot links.
 * @returns The mean of several readings of the inertia tensor of the same robot link.
 */
MatrixXd get_mean_inertia_tensor(MatrixXd &inertia_list)
{
    int trials = inertia_list.cols()/3;
    int joints = inertia_list.rows()/3;
    MatrixXd inertia_mean = MatrixXd::Zero(joints*3, 3);
    VectorXd aux = VectorXd::Zero(trials);
    for (int row=0;row<joints*3;row++)
    {
        for (int j=0;j<3;j++)
        {
            for(int i=0;i<trials;i++)
            {
                aux(i) = inertia_list(row, i*3+j);
            }
            inertia_mean(row,j) = aux.mean();
        }
    }
    return inertia_mean;
}


/**
 * @brief This function computes the center of mass mean
 * @param com_list Matrix that contains several readings of the center of mass of of all robot links.
 * @returns The mean of several readings of the center of mass.
 */
MatrixXd get_mean_com(MatrixXd &com_list)
{
    int trials = com_list.cols();
    int joints = com_list.rows()/3;
    MatrixXd com_mean = MatrixXd::Zero(joints*3, 1);
    VectorXd aux = VectorXd::Zero(trials);
    for (int row=0;row<joints*3;row++)
    {
        for(int i=0;i<trials;i++)
        {
            aux(i) = com_list(row, i);
        }
        com_mean(row,0) = aux.mean();
    }
    return com_mean;
}

/**
 * @brief This function returns a rotation matrix given a rotation unit quaternion.
 * @param r unit quaternion.
 * @returns The rotation matrix.
 */
MatrixXd get_rotation_matrix(DQ r){
    Matrix<double, 3, 3> R;
    VectorXd vecr = r.vec4();
    double w = vecr(0);
    double a = vecr(1);
    double b = vecr(2);
    double c = vecr(3);
    R << 1-2*(b*b +c*c), 2*(a*b-w*c), 2*(a*c+w*b),
        2*(a*b+w*c), 1-2*(a*a+c*c),  2*(b*c-w*a),
        2*(a*c-w*b), 2*(b*c+w*a),   1-2*(a*a+b*b);

    return R;
}



int main(void)
{
    DQ_CoppeliaSimInterface vi;
    vi.connect();
    vi.set_synchronous(true);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::vector<std::string> linknames ={"Franka/link2_resp","Franka/link3_resp","Franka/link4_resp",
                                          "Franka/link5_resp","Franka/link6_resp","Franka/link7_resp",
                                          "Franka/link8_resp"};


    //std::vector<std::string> jointnames ={"Franka_joint1", "Franka_joint2","Franka_joint3",
    //                                       "Franka_joint4", "Franka_joint5", "Franka_joint6",
    //                                       "Franka_joint7"};


    std::vector<std::string> jointnames =
        vi.get_jointnames_from_base_objectname(std::string("Franka"));

    vi.set_joint_modes(jointnames, DQ_CoppeliaSimInterface::DYNAMIC);
    vi.set_joint_control_modes(jointnames, DQ_CoppeliaSimInterface::POSITION);

    //------------------- Robot definition--------------------------
    //---------- Franka Emika Panda serial manipulator
    DQ_SerialManipulatorMDH franka = FrankaEmikaPandaRobot::kinematics();

    //Update the base of the robot from CoppeliaSim
    DQ new_base_robot = (franka.get_base_frame())*vi.get_object_pose("Franka")*(1+0.5*E_*(-0.07*k_));
    franka.set_reference_frame(new_base_robot);
    //--------------------------------------------------------------
    // We extract the inertial parameters in four different robot configurations. After, we compute the mean of the readings.

    VectorXd targetPos1 = VectorXd::Zero(7);
    targetPos1 << 0,0,0,-M_PI_2, 0, M_PI_2,0;

    VectorXd targetPos2 = VectorXd::Zero(7);
    targetPos2 << 90*M_PI/180,90*M_PI/180,135*M_PI/180,-45*M_PI/180,90*M_PI/180,180*M_PI/180,0;

    VectorXd targetPos3 = VectorXd::Zero(7);
    targetPos3 << -90*M_PI/180,90*M_PI/180,135*M_PI/180,-45*M_PI/180,90*M_PI/180,180*M_PI/180,0;

    VectorXd targetPos4 = VectorXd::Zero(7);
    targetPos4 << 0,0,0,-90*M_PI/180,0,90*M_PI/180,0;

    Matrix<double, 7,4> list_positions;
    list_positions.col(0) = targetPos1;
    list_positions.col(1) = targetPos2;
    list_positions.col(2) = targetPos3;
    list_positions.col(3) = targetPos4;

    MatrixXd inertia_list = MatrixXd::Zero(linknames.size()*3, list_positions.cols()*3);
    MatrixXd com_list = MatrixXd::Zero(linknames.size()*3, list_positions.cols());

    std::cout<<"Number of positions: "<<list_positions.cols()<<std::endl;

    VectorXd mass_list = VectorXd::Zero(7);

    for (int k=0;k<4;k++)
    {
        std::cout<<"Setting position: "<<k+1<<std::endl;
        for (int i=0;i<500;i++)
        {
            vi.set_joint_target_positions(jointnames,  list_positions.col(k));
            vi.trigger_next_simulation_step();
            VectorXd q = vi.get_joint_positions(jointnames);
            //vi.set_object_pose("ReferenceFrame", franka.fkm(q));
        }

        for (int j=0;j<linknames.size();j++)
        {
            std::string link = linknames[j];
            double mass = vi.get_mass(link);
            mass_list(j) = mass;

            VectorXd q = vi.get_joint_positions(jointnames);
            DQ xi = franka.fkm(q,j);
            //vi.set_object_pose("ReferenceFrame", xi);
            MatrixXd R = get_rotation_matrix(xi.P());

            //compute the Inertia tensor of each link expressed in the absolute frame.
            MatrixXd I_absolute_frame = vi.get_inertia_matrix(link, DQ_CoppeliaSimInterface::ABSOLUTE_FRAME);

            //compute the Inertia tensor of each link expressed in the Denavit Hartenberg frames.
            MatrixXd I_DH_frame = R.transpose()*I_absolute_frame*R;
            inertia_list.block(j*3, k*3, 3,3) = I_DH_frame;

            //compute the center of mass of each link expressed in the absolute frame.
            DQ pcom0 = vi.get_center_of_mass(link, DQ_CoppeliaSimInterface::ABSOLUTE_FRAME);
            DQ xcom0 = 1+ E_*0.5*pcom0;

            //We compute the constant rigid transformation of the center of mass with respect to the DH frames.
            VectorXd vecxc = vec3(translation(xi.conj()*xcom0));
            com_list.block(j*3, k, 3,1) = vecxc;
            vi.trigger_next_simulation_step();
        }
    }
    std::cout<<" "<<std::endl;
    std::cout<<"std::vector<Matrix<double, 3,3>> inertia_tensors("<<linknames.size()<<");"<<std::endl;
    for (int j=0;j<linknames.size();j++)
    {
        std::string link = linknames[j];
        auto inertias = get_mean_inertia_tensor(inertia_list);
        auto inertia = inertias.block(j*3, 0, 3,3);
        //std::cout<<"Inertia_matrix (mean) in DH frame:\n"<<
        std::cout<<"inertia_tensors["<<j<<"] =  (MatrixXd(3,3) <<"<<inertia(0,0)<<", "<<inertia(0,1)<<", "<<inertia(0,2)<<","<<std::endl;
        std::cout<<"                                      "<<inertia(1,0)<<", "<<inertia(1,1)<<", "<<inertia(1,2)<<","<<std::endl;
        std::cout<<"                                      "<<inertia(2,0)<<", "<<inertia(2,1)<<", "<<inertia(2,2)<<").finished();"<<std::endl;
        std::cout<<" "<<std::endl;

    }
    std::cout<<" "<<std::endl;
    std::cout<<"std::vector<Vector3d> center_of_masses("<<linknames.size()<<");"<<std::endl;
    for (int j=0;j<linknames.size();j++)
    {
        std::string link = linknames[j];
        //std::cout<<" "<<std::endl;
        auto coms = get_mean_com(com_list);
        auto com = coms.block(j*3, 0, 3,1);
        std::cout<<"center_of_masses["<<j<<"] = (Vector3d() <<"<<com(0,0)<<", "
                  <<com(1,0)<<", "<<com(2,0)<<").finished();"<<std::endl;
    }
    std::cout<<" "<<std::endl;
    auto mass = mass_list.transpose();
    std::cout<<"std::vector<double> masses = {"<<mass(0)<<","<<
        mass(1)<<","<<mass(2)<<","<<mass(3)<<","<<
        mass(4)<<","<< mass(5)<<","<< mass(6)<<"};"<<std::endl;

    std::cout<<" "<<std::endl;
    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();

    vi.disconnect();


    return 0;
}
