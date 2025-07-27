#include "SensorPreprocessor.h"

#include <boost/property_tree/ptree.hpp>
#include <random>

SensorPreprocessor::SensorPreprocessor(double imuAccelNoise, double imuGyroNoise,
                                       double posNoise)
    : m_imu(imuAccelNoise, imuGyroNoise),
      m_position(posNoise),
      m_integratedVel(Eigen::Vector3d::Zero()),
      m_rng(std::random_device{}()),
      m_dropoutDist(0.0, 1.0)
{}

SensorPreprocessor::SensorPreprocessor(const boost::property_tree::ptree& pt)
    : m_imu(pt.get<double>("sensor.imu_accel_noise", 0.01),
            pt.get<double>("sensor.imu_gyro_noise", 0.001),
            Eigen::Vector3d(pt.get<double>("sensor.imu_accel_bias_x", 0.0),
                            pt.get<double>("sensor.imu_accel_bias_y", 0.0),
                            pt.get<double>("sensor.imu_accel_bias_z", 0.0)),
            Eigen::Vector3d(pt.get<double>("sensor.imu_gyro_bias_x", 0.0),
                            pt.get<double>("sensor.imu_gyro_bias_y", 0.0),
                            pt.get<double>("sensor.imu_gyro_bias_z", 0.0))),
      m_position(pt),
      m_integratedVel(Eigen::Vector3d::Zero()),
      m_rng(std::random_device{}()),
      m_dropoutDist(0.0, 1.0)
{}

void SensorPreprocessor::initialize(const Eigen::Vector3d& initialVelocity)
{
    m_integratedVel = initialVelocity;
}

StateVec SensorPreprocessor::getMeasurement(VehicleTruthModel* truth, double dt, int step,
                                            const StateVec& estimate)
{
    // Simulate 5% dropout
    if (m_dropoutDist(m_rng) < 0.3)
    {
        // Return invalid measurement (all NaNs)
        return StateVec::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    // Simulate IMU measurement
    Eigen::Matrix<double, 6, 1> imu_meas = m_imu.measure(truth, dt);
    Eigen::Vector3d accel = imu_meas.head<3>();
    if (step == 0)
    {
        m_integratedVel = truth->getVelocity();
    }
    else
    {
        updateState(estimate);
    }
    m_integratedVel += accel * dt;

    // Simulate position measurement every 10 steps
    Eigen::Vector3d posMeas(0, 0, 0);
    bool use_pos = (step % 10 == 0);
    if (use_pos)
    {
        posMeas = m_position.measure(truth);
    }
    else
    {
        posMeas = estimate.head<3>();
    }

    StateVec measurement;
    measurement << posMeas(0), posMeas(1), posMeas(2), m_integratedVel(0), m_integratedVel(1),
        m_integratedVel(2);
    return measurement;
}

void SensorPreprocessor::updateState(const StateVec& state)
{
    m_integratedVel = state.segment<3>(3);
}

const Eigen::Vector3d& SensorPreprocessor::getIntegratedVel() const { return m_integratedVel; }
