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
                  const Eigen::Vector3d& center = Eigen::Vector3d::Zero());
    void step(double dt) override;
    Eigen::Vector3d getPosition() const override;
    Eigen::Vector3d getVelocity() const override;
    Eigen::Vector3d getOrientation() const override;
    Eigen::Vector3d getAngularVelocity() const override;

   private:
    double r_, v_, theta_, depth_;
    Eigen::Vector3d center_;
};
