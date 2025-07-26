#include "CircleVehicle.h"

#include <cmath>

CircleVehicle::CircleVehicle(double radius, double speed, double depth,
                             const Eigen::Vector3d& center)
    : m_radius(radius), m_speed(speed), m_theta(0.0), m_depth(depth), m_center(center)
{}

void CircleVehicle::step(double dt)
{
    m_theta += m_speed * dt / m_radius;
    if (m_theta > 2 * M_PI) m_theta -= 2 * M_PI;
}

Eigen::Vector3d CircleVehicle::getPosition() const
{
    return m_center +
           Eigen::Vector3d(m_radius * std::cos(m_theta), m_radius * std::sin(m_theta), m_depth);
}

Eigen::Vector3d CircleVehicle::getVelocity() const
{
    return Eigen::Vector3d(-m_speed * std::sin(m_theta), m_speed * std::cos(m_theta), 0.0);
}

Eigen::Vector3d CircleVehicle::getOrientation() const
{
    return Eigen::Vector3d(0.0, 0.0, m_theta);  // roll, pitch, yaw
}

Eigen::Vector3d CircleVehicle::getAngularVelocity() const
{
    return Eigen::Vector3d(0.0, 0.0, m_speed / m_radius);
}
