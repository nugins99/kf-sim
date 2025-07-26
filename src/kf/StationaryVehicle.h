#pragma once
#include <Eigen/Dense>
#include "VehicleTruthModel.h"

/**
 * @brief Stationary vehicle scenario (no movement).
 */
class StationaryVehicle : public VehicleTruthModel
{
   public:
    StationaryVehicle(const Eigen::Vector3d& pos = Eigen::Vector3d::Zero(),
                  const Eigen::Vector3d& orient = Eigen::Vector3d::Zero());
    void step(double dt) override;
    Eigen::Vector3d getPosition() const override;
    Eigen::Vector3d getVelocity() const override;
    Eigen::Vector3d getOrientation() const override;
    Eigen::Vector3d getAngularVelocity() const override;

   private:
    Eigen::Vector3d pos_;
    Eigen::Vector3d orient_;
};
