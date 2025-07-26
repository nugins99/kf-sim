#pragma once
#include <Eigen/Dense>
#include <random>
#include "VehicleTruthModel.h"

/**
 * @brief Simulated Range/Bearing/Depth sensor for 6DOF vehicle, outputs noisy position (x, y, z).
 */
class PositionDepthSensor
{
   public:
    PositionDepthSensor(double pos_noise_std);

    /**
     * @brief Simulate a measurement from the sensor using the truth model.
     * @param truth Pointer to the truth model.
     * @return 3D vector [x, y, z] with noise.
     */
    Eigen::Vector3d measure(const VehicleTruthModel* truth);

   private:
    std::random_device rd_;
    std::default_random_engine gen_;
    std::normal_distribution<double> pos_noise_;
    Eigen::Vector3d drift_ = Eigen::Vector3d::Zero();  ///< Drift vector for position measurements
    Eigen::Vector3d drift_rate_ = Eigen::Vector3d::Zero();  ///< Drift rate for position measurements
};
