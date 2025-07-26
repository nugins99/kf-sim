#pragma once
#include <Eigen/Dense>
#include <random>

#include "VehicleTruthModel.h"

/**
 * @brief Simulated IMU sensor class for 6D acceleration and angular velocity measurements.
 */
class IMUSensor6D
{
   public:
    IMUSensor6D(double accel_noise_std, double gyro_noise_std);

    /**
     * @brief Simulate a measurement from the IMU using the truth model.
     * @param truth Pointer to the truth model.
     * @param dt Time step (seconds).
     * @note: ax, ay, az are calculated from the change in velocity over time.
     * @note: wx, wy, wz are the angular velocities directly from the truth model.
     * @return 6D vector [ax, ay, az, wx, wy, wz] with noise.
     */
    Eigen::Matrix<double, 6, 1> measure(const VehicleTruthModel* truth, double dt);

   private:
    std::random_device rd_;
    std::default_random_engine gen_;
    std::normal_distribution<double> accel_noise_;
    std::normal_distribution<double> gyro_noise_;
    Eigen::Vector3d prev_vel_;
    bool prev_vel_set_;
};
