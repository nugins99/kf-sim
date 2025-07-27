#include <gtest/gtest.h>
#include "IMUSensor6D.h"
#include "VehicleTruthModel.h"

class DummyTruth : public VehicleTruthModel {
public:
    Eigen::Vector3d getPosition() const override { return Eigen::Vector3d(0,0,0); }
    Eigen::Vector3d getVelocity() const override { return Eigen::Vector3d(1,2,3); }
    Eigen::Vector3d getOrientation() const override { return Eigen::Vector3d(0,0,0); }
    Eigen::Vector3d getAngularVelocity() const override { return Eigen::Vector3d(0.1,0.2,0.3); }
    void step(double) override {}
};

TEST(IMUSensor6D, BiasEffect) {
    DummyTruth truth;
    Eigen::Vector3d accelBias(1.0, 2.0, 3.0);
    Eigen::Vector3d gyroBias(0.1, 0.2, 0.3);
    IMUSensor6D imu(0.0, 0.0, accelBias, gyroBias);
    auto meas = imu.measure(&truth, 0.1);
    EXPECT_NEAR(meas(0), accelBias(0), 1e-6);
    EXPECT_NEAR(meas(1), accelBias(1), 1e-6);
    EXPECT_NEAR(meas(2), accelBias(2), 1e-6);
    EXPECT_NEAR(meas(3), gyroBias(0) + truth.getAngularVelocity()(0), 1e-6);
    EXPECT_NEAR(meas(4), gyroBias(1) + truth.getAngularVelocity()(1), 1e-6);
    EXPECT_NEAR(meas(5), gyroBias(2) + truth.getAngularVelocity()(2), 1e-6);
}
