#include "dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h"
#include <gtest/gtest.h>


using namespace DQ_robotics;
using namespace Eigen;

namespace My{
    class InterfaceUnitTests : public testing::Test {

    protected:
    InterfaceUnitTests() {
    }

    ~InterfaceUnitTests() override {
    }

    void SetUp() override {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    }; // class InterfaceUnitTests
    //----------------------TESTS HERE----------------------------------------
    TEST_F(InterfaceUnitTests, start_simulation){
        DQ_CoppeliaSimInterface vi;
        vi.connect("localhost", 23000, 1000);
        vi.close_scene();
        vi.start_simulation();
        EXPECT_NE(vi.get_simulation_state(),8);
        EXPECT_NE(vi.get_simulation_state(),0);
        vi.stop_simulation();
    };

    TEST_F(InterfaceUnitTests, get_object_pose) {
        DQ_CoppeliaSimInterface vi;
        vi.connect("localhost", 23000, 1000);
        vi.close_scene();
        DQ r = cos(M_PI/2) + k_*sin(M_PI/2);
        DQ p = 0.5*i_ + 0.4*j_ + 0.9*k_;
        DQ x = r + 0.5*E_*p*r;
        vi.plot_reference_frame("/x", x);
        vi.stop_simulation();

        EXPECT_EQ(vi.get_object_pose("/x"), x);
    }

    //-----------------------------------------------------------------------

} // namespace My

int main(int argc, char **argv) {
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}
