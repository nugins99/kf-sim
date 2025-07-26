#include "PositionDepthSensor.h"

#include <boost/property_tree/ptree.hpp>

PositionDepthSensor::PositionDepthSensor(double pos_noise_std)
    : gen_(rd_()), pos_noise_(0.0, pos_noise_std)
{
    // drift_rate_ = Eigen::Vector3d{1e-3, 1e-3, 1e-6};  // Example drift rate in each direction
}

PositionDepthSensor::PositionDepthSensor(double pos_noise_std, const Eigen::Vector3d& drift_rate)
    : gen_(rd_()), pos_noise_(0.0, pos_noise_std), drift_rate_(drift_rate)
{}

PositionDepthSensor::PositionDepthSensor(const boost::property_tree::ptree& pt)
    : gen_(rd_()),
      pos_noise_(0.0, pt.get<double>("sensor.pos_noise", 1.0)),
      drift_rate_(Eigen::Vector3d(pt.get<double>("sensor.drift_rate_x", 0.0),
                                  pt.get<double>("sensor.drift_rate_y", 0.0),
                                  pt.get<double>("sensor.drift_rate_z", 0.0)))
{}

Eigen::Vector3d PositionDepthSensor::measure(const VehicleTruthModel* truth)
{
    Eigen::Vector3d position = truth->getPosition();
    for (int i = 0; i < 3; ++i)
    {
        position(i) += pos_noise_(gen_);
    }
    drift_ += drift_rate_;     // Update drift based on drift rate
    return position + drift_;  // Add drift to the position measurement
}
