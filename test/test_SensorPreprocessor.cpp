#include <gtest/gtest.h>
#include "SensorPreprocessor.h"
#include "StationaryVehicle.h"

TEST(SensorPreprocessor, MeasurementMatchesTruth) {
    StationaryVehicle truth(Eigen::Vector3d(1,2,3), Eigen::Vector3d(0,0,0));
    SensorPreprocessor pre(0.0, 0.0, 0.0);
    pre.initialize(Eigen::Vector3d(0,0,0));
    auto meas = pre.getMeasurement(&truth, 0.1, 0, Eigen::Matrix<double,6,1>::Zero());
    EXPECT_EQ(meas.head<3>(), truth.getPosition());
}
