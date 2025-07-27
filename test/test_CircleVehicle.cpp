#include <gtest/gtest.h>
#include "CircleVehicle.h"

TEST(CircleVehicle, PositionAndVelocity) {
    double radius = 10.0;
    double speed = 2.0;
    double depth = -5.0;
    Eigen::Vector3d center(1.0, 2.0, 0.0);
    CircleVehicle vehicle(radius, speed, depth, center);
    vehicle.step(0.5);
    Eigen::Vector3d pos = vehicle.getPosition();
    Eigen::Vector3d vel = vehicle.getVelocity();
    EXPECT_NEAR(pos(2), depth, 1e-6);
    EXPECT_NEAR((pos.head<2>() - center.head<2>()).norm(), radius, 1e-6);
    EXPECT_NEAR(vel.norm(), speed, 1e-6);
}
