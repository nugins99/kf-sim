#pragma once
#include <Eigen/Dense>
#include <boost/property_tree/ptree.hpp>

#include "IMUSensor6D.h"
#include "PositionDepthSensor.h"
#include "VehicleTruthModel.h"
#include <Types.h>  // Include the Types header for DOF, StateVec, StateMat

/**
 * @class SensorPreprocessor
 * @brief Preprocesses sensor data for state estimation in a Kalman Filter.
 *
 * Handles simulation and preprocessing of IMU and position sensor data,
 * integrates velocity estimates, and provides measurement vectors for filtering.
 */
class SensorPreprocessor
{
   public:

    /**
     * @brief Constructor
     * @param imu_accel_noise Standard deviation of IMU accelerometer noise.
     * @param imu_gyro_noise Standard deviation of IMU gyroscope noise.
     * @param pos_noise Standard deviation of position sensor noise.
     */
    SensorPreprocessor(double imu_accel_noise, double imu_gyro_noise, double pos_noise);

    /**
     * @brief Constructor from property tree
     * @param pt boost::property_tree::ptree object for config
     */
    SensorPreprocessor(const boost::property_tree::ptree& pt);

    /**
     * @brief Initializes the preprocessor with an initial velocity.
     * @param initial_vel Initial velocity vector (3D).
     */
    void initialize(const Eigen::Vector3d& initial_vel);

    /**
     * @brief Generates a noisy measurement vector based on the truth model and filter estimate.
     * @param truth Pointer to the vehicle truth model.
     * @param dt Time step size.
     * @param step Current simulation step.
     * @param kf_est Current Kalman Filter state estimate.
     * @return StateVec Measurement vector.
     */
    StateVec getMeasurement(VehicleTruthModel* truth, double dt, int step, const StateVec& kf_est);

    /**
     * @brief Updates the internally integrated velocity using the current filter state.
     * @param kf_state Current Kalman Filter state vector.
     */
    void updateState(const StateVec& kf_state);

    /**
     * @brief Returns the current integrated velocity vector.
     * @return Reference to the integrated velocity.
     */
    const Eigen::Vector3d& getIntegratedVel() const;

   private:
    IMUSensor6D m_imu;                ///< IMU sensor simulation object
    PositionDepthSensor m_position;   ///< Position and depth sensor simulation object
    Eigen::Vector3d m_integratedVel;  ///< Integrated velocity vector
};
