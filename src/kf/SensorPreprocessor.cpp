#include "SensorPreprocessor.h"

#include <boost/property_tree/ptree.hpp>

SensorPreprocessor::SensorPreprocessor(double imu_accel_noise, double imu_gyro_noise,
                                       double pos_noise)
    : m_imu(imu_accel_noise, imu_gyro_noise),
      m_position(pos_noise),
      m_integratedVel(Eigen::Vector3d::Zero())
{}

SensorPreprocessor::SensorPreprocessor(const boost::property_tree::ptree& pt)
    : m_imu(pt.get<double>("sensor.imu_accel_noise", 0.01),
            pt.get<double>("sensor.imu_gyro_noise", 0.001)),
      m_position(pt),
      m_integratedVel(Eigen::Vector3d::Zero())
{}

void SensorPreprocessor::initialize(const Eigen::Vector3d& initial_vel)
{
    m_integratedVel = initial_vel;
}

SensorPreprocessor::StateVec SensorPreprocessor::getMeasurement(VehicleTruthModel* truth, double dt,
                                                                int step, const StateVec& kf_est)
{
    // Simulate IMU measurement
    Eigen::Matrix<double, 6, 1> imu_meas = m_imu.measure(truth, dt);
    Eigen::Vector3d accel = imu_meas.head<3>();
    if (step == 0)
    {
        m_integratedVel = truth->getVelocity();
    }
    m_integratedVel += accel * dt;

    // Simulate position measurement every 10 steps
    Eigen::Vector3d pos_meas(0, 0, 0);
    bool use_pos = (step % 10 == 0);
    if (use_pos)
    {
        pos_meas = m_position.measure(truth);
    }
    else
    {
        pos_meas = kf_est.head<3>();
    }

    StateVec measurement;
    measurement << pos_meas(0), pos_meas(1), pos_meas(2), m_integratedVel(0), m_integratedVel(1),
        m_integratedVel(2);
    return measurement;
}

void SensorPreprocessor::updateIntegratedVel(const StateVec& kf_state)
{
    m_integratedVel = kf_state.segment<3>(3);
}

const Eigen::Vector3d& SensorPreprocessor::getIntegratedVel() const { return m_integratedVel; }
