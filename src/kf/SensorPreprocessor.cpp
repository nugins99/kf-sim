#include "SensorPreprocessor.h"

SensorPreprocessor::SensorPreprocessor(double imu_accel_noise, double imu_gyro_noise, double pos_noise)
    : imu_(imu_accel_noise, imu_gyro_noise), position_(pos_noise), integrated_vel_(Eigen::Vector3d::Zero()) {}

void SensorPreprocessor::initialize(const Eigen::Vector3d& initial_vel) {
    integrated_vel_ = initial_vel;
}

SensorPreprocessor::StateVec SensorPreprocessor::getMeasurement(VehicleTruthModel* truth, double dt, int step, const StateVec& kf_est) {
    // Simulate IMU measurement
    Eigen::Matrix<double, 6, 1> imu_meas = imu_.measure(truth, dt);
    Eigen::Vector3d accel = imu_meas.head<3>();
    if (step == 0) {
        integrated_vel_ = truth->getVelocity();
    }
    integrated_vel_ += accel * dt;

    // Simulate position measurement every 10 steps
    Eigen::Vector3d pos_meas(0, 0, 0);
    bool use_pos = (step % 10 == 0);
    if (use_pos) {
        pos_meas = position_.measure(truth);
    } else {
        pos_meas = kf_est.head<3>();
    }

    StateVec measurement;
    measurement << pos_meas(0), pos_meas(1), pos_meas(2),
        integrated_vel_(0), integrated_vel_(1), integrated_vel_(2);
    return measurement;
}

void SensorPreprocessor::updateIntegratedVel(const StateVec& kf_state) {
    integrated_vel_ = kf_state.segment<3>(3);
}

const Eigen::Vector3d& SensorPreprocessor::getIntegratedVel() const {
    return integrated_vel_;
}
