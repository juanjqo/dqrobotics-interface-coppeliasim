#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h"
#include <gtest/gtest.h>


using namespace DQ_robotics;
using namespace Eigen;

namespace My{
    class InterfaceUnitTests : public testing::Test {

    protected:
        enum class ROBOT_MODE{KINEMATIC, POSITION, VELOCITY, TORQUE};
        std::unique_ptr<DQ_CoppeliaSimInterface> vi_;
        VectorXd q_panda_home_ = (VectorXd(7)<<0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4).finished();
    InterfaceUnitTests() {
        vi_ = std::make_unique<DQ_CoppeliaSimInterface>();
        vi_->connect("localhost", 23000, 1000);
    }

    ~InterfaceUnitTests() override {
        vi_->stop_simulation();
        delay();
        vi_->close_scene();
    }

    std::vector<std::string> load_panda(const ROBOT_MODE& robot_mode){
        vi_->load_from_model_browser("/robots/non-mobile/FrankaEmikaPanda.ttm","/Franka", true, true);
        auto jointnames = vi_->get_jointnames_from_parent_object("/Franka");
        switch (robot_mode){

        case ROBOT_MODE::KINEMATIC:
            vi_->set_joint_modes(jointnames, DQ_CoppeliaSimInterface::JOINT_MODE::KINEMATIC);
            vi_->enable_dynamics(false);
            break;
        case ROBOT_MODE::POSITION:
            vi_->set_joint_modes(jointnames, DQ_CoppeliaSimInterface::JOINT_MODE::DYNAMIC);
            vi_->set_joint_control_modes(jointnames, DQ_CoppeliaSimInterface::JOINT_CONTROL_MODE::POSITION);
            vi_->enable_dynamics(true);
            break;
        case ROBOT_MODE::VELOCITY:
            vi_->set_joint_modes(jointnames, DQ_CoppeliaSimInterface::JOINT_MODE::DYNAMIC);
            vi_->set_joint_control_modes(jointnames, DQ_CoppeliaSimInterface::JOINT_CONTROL_MODE::VELOCITY);
            vi_->enable_dynamics(true);
            break;
        case ROBOT_MODE::TORQUE:
            vi_->set_joint_modes(jointnames, DQ_CoppeliaSimInterface::JOINT_MODE::DYNAMIC);
            vi_->set_joint_control_modes(jointnames, DQ_CoppeliaSimInterface::JOINT_CONTROL_MODE::TORQUE);
            vi_->enable_dynamics(true);
            break;
        }
        return jointnames;
    }

    void delay(const int& time_ms = 100){
       std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
    }

    void SetUp() override {
        // Code here will be called immediately after the constructor (right
        // before each test).
        vi_->close_scene();
    }

    void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
        vi_->stop_simulation();
    }

    }; // class InterfaceUnitTests



    //----------------------TESTS HERE----------------------------------------
    TEST_F(InterfaceUnitTests, start_stop_simulation){
        vi_->start_simulation();
        delay();
        EXPECT_NE(vi_->get_simulation_state(),8);
        EXPECT_NE(vi_->get_simulation_state(),0);
        vi_->stop_simulation();
        delay();
        EXPECT_EQ(vi_->get_simulation_state(),0)<<"Error in start_simulation() or stop_simulation()";
    };

    TEST_F(InterfaceUnitTests, stepping_mode){
        vi_->set_stepping_mode(true);
        double T{vi_->get_simulation_time_step()};
        double t{0};
        double elapsed_time{0};
        vi_->start_simulation();
        for (auto i=0; i<5;i++)
        {
            t = i*T;
            elapsed_time = vi_->get_simulation_time();
            vi_->trigger_next_simulation_step();
        }
        vi_->set_stepping_mode(false);
        EXPECT_EQ(t, elapsed_time)<<"There is something wrong with the stepping mode!";

    };

    TEST_F(InterfaceUnitTests, get_object_pose) {
        DQ p = -0.1*k_;
        DQ x = 1 + 0.5*E_*p;
        VectorXd vx = x.vec8();
        VectorXd vxr = vi_->get_object_pose("/Floor").vec8();
        for (auto i=0;i<vxr.size();i++)
            EXPECT_NEAR(vx(i), vxr(i), 1e-6)<<"Error in get_object_pose()";
    }

    TEST_F(InterfaceUnitTests, plot_reference_frame) {
        DQ r = cos(M_PI/2) + k_*sin(M_PI/2);
        DQ p = 0.5*i_ + 0.4*j_ + 0.9*k_;
        DQ x = r + 0.5*E_*p*r;
        vi_->plot_reference_frame("/x", x);
        EXPECT_EQ(vi_->get_object_pose("/x"), x)<<"Error in plot_reference_frame()";
    }

    TEST_F(InterfaceUnitTests, plot_plane) {
        DQ p = 0.5*i_ + 0.4*j_ + 0.9*k_;
        vi_->plot_plane("/plane", k_, p);
        EXPECT_EQ(vi_->get_object_translation("/plane"), p)<<"Error in plot_plane()";
    }

    TEST_F(InterfaceUnitTests, plot_line) {
        DQ p = 0.5*i_ + 0.4*j_ + 0.9*k_;
        vi_->plot_line("/line", k_, p);
        EXPECT_EQ(vi_->get_object_translation("/line"), p)<<"Error in plot_line()";
    }

    TEST_F(InterfaceUnitTests, plot_cylinder) {
        DQ p = 0.5*i_ + 0.4*j_ + 0.9*k_;
        vi_->plot_line("/cylinder", i_, p);
        EXPECT_EQ(vi_->get_object_translation("/cylinder"), p)<<"Error in plot_cylinder()";
    }


    TEST_F(InterfaceUnitTests, plot_sphere) {
        DQ p = 0.5*i_ + 0.4*j_ + 0.9*k_;
        vi_->plot_sphere("/sphere", p);
        EXPECT_EQ(vi_->get_object_translation("/sphere"), p)<<"Error in plot_sphere()";
    }

    TEST_F(InterfaceUnitTests, object_exist) {
        vi_->plot_reference_frame("/x", DQ(1));
        EXPECT_EQ(vi_->object_exist_on_scene("/x"), true)<<"Error in object_exist()";
    }

    TEST_F(InterfaceUnitTests, remove_object) {
        vi_->plot_reference_frame("/x", DQ(1));
        vi_->remove_object("/x");
        EXPECT_EQ(vi_->object_exist_on_scene("/x"), false)<<"Error in remove_object()";
    }

    TEST_F(InterfaceUnitTests, joint_positions){
        auto jointnames = load_panda(ROBOT_MODE::KINEMATIC);
        vi_->set_joint_positions(jointnames, q_panda_home_);
        delay();
        auto qr = vi_->get_joint_positions(jointnames);
        for (auto i=0;i<q_panda_home_.size();i++)
            EXPECT_EQ(q_panda_home_(i), qr(i));
        vi_->enable_dynamics(true);
    }

    TEST_F(InterfaceUnitTests, target_joint_positions){
        auto jointnames = load_panda(ROBOT_MODE::POSITION);
        vi_->start_simulation();
        vi_->set_joint_target_positions(jointnames, q_panda_home_);
        delay(800);
        auto qr = vi_->get_joint_positions(jointnames);
        for (auto i=0;i<q_panda_home_.size();i++)
            EXPECT_NEAR(q_panda_home_(i), qr(i), 1e-3);
    }


    //-----------------------------------------------------------------------

} // namespace My

int main(int argc, char **argv) {
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}
