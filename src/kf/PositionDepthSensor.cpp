#include "PositionDepthSensor.h"

#include <boost/property_tree/ptree.hpp>

PositionDepthSensor::PositionDepthSensor(double pos_noise_std)
    : m_gen(m_rd()), m_posNoise(0.0, pos_noise_std)
{
    // m_driftRate = Eigen::Vector3d{1e-3, 1e-3, 1e-6};  // Example drift rate in each direction
}

PositionDepthSensor::PositionDepthSensor(double pos_noise_std, const Eigen::Vector3d& drift_rate)
    : m_gen(m_rd()), m_posNoise(0.0, pos_noise_std), m_driftRate(drift_rate)
{}

PositionDepthSensor::PositionDepthSensor(const boost::property_tree::ptree& pt)
    : m_gen(m_rd()),
      m_posNoise(0.0, pt.get<double>("sensor.pos_noise", 1.0)),
      m_driftRate(Eigen::Vector3d(pt.get<double>("sensor.drift_rate_x", 0.0),
                                  pt.get<double>("sensor.drift_rate_y", 0.0),
                                  pt.get<double>("sensor.drift_rate_z", 0.0)))
{}

Eigen::Vector3d PositionDepthSensor::measure(const VehicleTruthModel* truth)
{
    Eigen::Vector3d position = truth->getPosition();
    for (int i = 0; i < 3; ++i)
    {
        position(i) += m_posNoise(m_gen);
    }
    m_drift += m_driftRate;     // Update drift based on drift rate
    return position + m_drift;  // Add drift to the position measurement
}
