#include <gtest/gtest.h>
#include "swerve_kinematics.h"
using namespace m2::simulation;

TEST(Swerve, TranslationAndReverse) {
    const auto forward = module_target(.2,0,0,.23,.34,0);
    EXPECT_NEAR(forward.angle,0,1e-12); EXPECT_NEAR(forward.speed,.2,1e-12);
    const auto reverse = module_target(-.2,0,0,.23,.34,0);
    EXPECT_NEAR(reverse.angle,0,1e-12); EXPECT_NEAR(reverse.speed,-.2,1e-12);
    const auto left = module_target(0,.2,0,.23,.34,0);
    EXPECT_NEAR(left.angle,pi/2,1e-12); EXPECT_NEAR(left.speed,.2,1e-12);
}

TEST(Swerve, ReconstructCombinedTwistAtEveryWheel) {
    for (double x : {-.23,.23}) for (double y : {-.34,.34})
        for (double heading : {-7.0,-pi,0.0,pi,7.0}) {
            const auto t = module_target(.2,-.1,.4,x,y,heading);
            EXPECT_NEAR(t.speed*std::cos(t.angle),.2-.4*y,1e-12);
            EXPECT_NEAR(t.speed*std::sin(t.angle),-.1+.4*x,1e-12);
            EXPECT_LE(std::abs(t.angle-heading),pi/2);
        }
}

TEST(Swerve, StopKeepsModuleAngle) {
    const auto t = module_target(0,0,0,.23,.34,2.3);
    EXPECT_DOUBLE_EQ(t.angle,2.3); EXPECT_DOUBLE_EQ(t.speed,0);
}
