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
    /**
     * @brief Constructor
     * @param accel_noise_std Standard deviation of simulated accelerometer noise (ax, ay, az).
     *        Higher values increase the randomness in the acceleration measurements.
     * @param gyro_noise_std Standard deviation of simulated gyroscope noise (wx, wy, wz).
     *        Higher values increase the randomness in the angular velocity measurements.
     *
     * These parameters control the amount of Gaussian noise added to the true acceleration and angular velocity
     * values reported by the sensor. This simulates real-world sensor imperfections and uncertainty.
     */
    IMUSensor6D(double accel_noise_std, double gyro_noise_std);

    /**
     * @brief Constructor with bias
     * @param accel_noise_std Standard deviation of simulated accelerometer noise (ax, ay, az).
     * @param gyro_noise_std Standard deviation of simulated gyroscope noise (wx, wy, wz).
     * @param accel_bias 3D bias vector added to acceleration measurements.
     * @param gyro_bias 3D bias vector added to angular velocity measurements.
     */
    IMUSensor6D(double accel_noise_std, double gyro_noise_std,
               const Eigen::Vector3d& accel_bias,
               const Eigen::Vector3d& gyro_bias);

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
    Eigen::Vector3d m_accelBias = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_gyroBias = Eigen::Vector3d::Zero();
};
