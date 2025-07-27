#include <gtest/gtest.h>
#include "StationaryVehicle.h"

TEST(StationaryVehicle, NoMovement) {
    Eigen::Vector3d pos(1.0, 2.0, 3.0);
    Eigen::Vector3d orient(0.1, 0.2, 0.3);
    StationaryVehicle vehicle(pos, orient);
    EXPECT_EQ(vehicle.getPosition(), pos);
    EXPECT_EQ(vehicle.getVelocity(), Eigen::Vector3d::Zero());
    EXPECT_EQ(vehicle.getOrientation(), orient);
    EXPECT_EQ(vehicle.getAngularVelocity(), Eigen::Vector3d::Zero());
}
