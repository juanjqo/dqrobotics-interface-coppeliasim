#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h"
#include <gtest/gtest.h>


using namespace DQ_robotics;
using namespace Eigen;

namespace My{
    class InterfaceUnitTests : public testing::Test {

    protected:
        std::unique_ptr<DQ_CoppeliaSimInterface> vi_;
    InterfaceUnitTests() {
        vi_ = std::make_unique<DQ_CoppeliaSimInterface>();
        vi_->connect("localhost", 23000, 1000);
    }

    ~InterfaceUnitTests() override {
        vi_->stop_simulation();
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
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        EXPECT_NE(vi_->get_simulation_state(),8);
        EXPECT_NE(vi_->get_simulation_state(),0);
        vi_->stop_simulation();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
        DQ r = cos(M_PI/2) + k_*sin(M_PI/2);
        DQ p = 0.5*i_ + 0.4*j_ + 0.9*k_;
        DQ x = r + 0.5*E_*p*r;
        vi_->plot_reference_frame("/x", x);
        EXPECT_EQ(vi_->get_object_pose("/x"), x)<<"Error in get_object_pose()";
    }

    //-----------------------------------------------------------------------

} // namespace My

int main(int argc, char **argv) {
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}
