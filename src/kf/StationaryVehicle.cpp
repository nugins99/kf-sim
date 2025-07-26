#include "StationaryVehicle.h"

StationaryVehicle::StationaryVehicle(const Eigen::Vector3d& pos, const Eigen::Vector3d& orient)
    : pos_(pos), orient_(orient)
{}

void StationaryVehicle::step(double /*dt*/) {}

Eigen::Vector3d StationaryVehicle::getPosition() const { return pos_; }
Eigen::Vector3d StationaryVehicle::getVelocity() const { return Eigen::Vector3d::Zero(); }
Eigen::Vector3d StationaryVehicle::getOrientation() const { return orient_; }
Eigen::Vector3d StationaryVehicle::getAngularVelocity() const { return Eigen::Vector3d::Zero(); }
