#pragma once
#include <Eigen/Dense>

#include "VehicleTruthModel.h"

/**
 * @brief Vehicle moving in a horizontal circle at constant speed and yaw.
 */
class CircleVehicle : public VehicleTruthModel
{
   public:
    CircleVehicle(double radius, double speed, double depth,
                  const Eigen::Vector3d& center = Eigen::Vector3d::Zero(),
                  double currentSpeed = 0.1, double currentDirection = 0.0);
    void step(double dt) override;
    Eigen::Vector3d getPosition() const override;
    Eigen::Vector3d getVelocity() const override;
    Eigen::Vector3d getOrientation() const override;
    Eigen::Vector3d getAngularVelocity() const override;

   private:
    double m_radius;
    double m_speed;
    double m_theta;
    double m_depth;
    Eigen::Vector3d m_center;
    double m_currentSpeed;      // Ocean current speed (ft/sec)
    double m_currentDirection;  // Ocean current direction (radians, 0=east)
    Eigen::Vector3d m_currentDrift; // Accumulated drift due to ocean current
};
