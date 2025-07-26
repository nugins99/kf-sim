#include "StationaryVehicle.h"

StationaryVehicle::StationaryVehicle(const Eigen::Vector3d& pos, const Eigen::Vector3d& orient)
    : m_pos(pos), m_orient(orient)
{}

void StationaryVehicle::step(double /*dt*/) {}

Eigen::Vector3d StationaryVehicle::getPosition() const { return m_pos; }
Eigen::Vector3d StationaryVehicle::getVelocity() const { return Eigen::Vector3d::Zero(); }
Eigen::Vector3d StationaryVehicle::getOrientation() const { return m_orient; }
Eigen::Vector3d StationaryVehicle::getAngularVelocity() const { return Eigen::Vector3d::Zero(); }
