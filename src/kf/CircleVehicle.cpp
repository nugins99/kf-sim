#include "CircleVehicle.h"
#include <cmath>

CircleVehicle::CircleVehicle(double radius, double speed, double depth, const Eigen::Vector3d& center)
    : r_(radius), v_(speed), theta_(0.0), depth_(depth), center_(center) {}

void CircleVehicle::step(double dt)
{
    theta_ += v_ * dt / r_;
    if (theta_ > 2 * M_PI) theta_ -= 2 * M_PI;
}

Eigen::Vector3d CircleVehicle::getPosition() const
{
    return center_ + Eigen::Vector3d(r_ * std::cos(theta_), r_ * std::sin(theta_), depth_);
}

Eigen::Vector3d CircleVehicle::getVelocity() const
{
    return Eigen::Vector3d(-v_ * std::sin(theta_), v_ * std::cos(theta_), 0.0);
}

Eigen::Vector3d CircleVehicle::getOrientation() const
{
    return Eigen::Vector3d(0.0, 0.0, theta_);  // roll, pitch, yaw
}

Eigen::Vector3d CircleVehicle::getAngularVelocity() const
{
    return Eigen::Vector3d(0.0, 0.0, v_ / r_);
}
