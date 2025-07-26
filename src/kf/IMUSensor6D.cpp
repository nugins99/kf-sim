#include "IMUSensor6D.h"
#include <cmath>

IMUSensor6D::IMUSensor6D(double accel_noise_std, double gyro_noise_std)
    : gen_(rd_()),
      accel_noise_(0.0, accel_noise_std),
      gyro_noise_(0.0, gyro_noise_std),
      prev_vel_set_(false)
{}

Eigen::Matrix<double, 6, 1> IMUSensor6D::measure(const VehicleTruthModel* truth, double dt)
{
    Eigen::Vector3d vel = truth->getVelocity();
    Eigen::Vector3d angvel = truth->getAngularVelocity();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    if (prev_vel_set_)
    {
        accel = (vel - prev_vel_) / dt;
    }
    prev_vel_ = vel;
    prev_vel_set_ = true;
    // Angular velocity is direct from truth, plus noise
    Eigen::Vector3d gyro = angvel;
    // Add noise
    for (int i = 0; i < 3; ++i)
    {
        accel(i) += accel_noise_(gen_);
        gyro(i) += gyro_noise_(gen_);
    }
    Eigen::Matrix<double, 6, 1> out;
    out << accel, gyro;
    return out;
}