#include "CircleVehicle.h"

#include <cmath>

CircleVehicle::CircleVehicle(double radius, double speed, double depth,
                             const Eigen::Vector3d& center,
                             double currentSpeed, double currentDirection)
    : m_radius(radius), m_speed(speed), m_theta(0.0), m_depth(depth), m_center(center),
      m_currentSpeed(currentSpeed), m_currentDirection(currentDirection),
      m_currentDrift(Eigen::Vector3d::Zero())
{}

void CircleVehicle::step(double dt)
{
    m_theta += m_speed * dt / m_radius;
    if (m_theta > 2 * M_PI) m_theta -= 2 * M_PI;
    // Accumulate drift due to ocean current
    Eigen::Vector3d currentVel(m_currentSpeed * std::cos(m_currentDirection),
                               m_currentSpeed * std::sin(m_currentDirection), 0.0);
    m_currentDrift += currentVel * dt;
}

Eigen::Vector3d CircleVehicle::getPosition() const
{
    Eigen::Vector3d circlePos = m_center +
        Eigen::Vector3d(m_radius * std::cos(m_theta), m_radius * std::sin(m_theta), m_depth);
    // Add accumulated drift due to ocean current
    return circlePos + m_currentDrift;
}

Eigen::Vector3d CircleVehicle::getVelocity() const
{
    // Vehicle velocity plus ocean current
    Eigen::Vector3d circleVel(-m_speed * std::sin(m_theta), m_speed * std::cos(m_theta), 0.0);
    Eigen::Vector3d currentVel(m_currentSpeed * std::cos(m_currentDirection),
                               m_currentSpeed * std::sin(m_currentDirection), 0.0);
    return circleVel + currentVel;
}

Eigen::Vector3d CircleVehicle::getOrientation() const
{
    return Eigen::Vector3d(0.0, 0.0, m_theta);  // roll, pitch, yaw
}

Eigen::Vector3d CircleVehicle::getAngularVelocity() const
{
    return Eigen::Vector3d(0.0, 0.0, m_speed / m_radius);
}
