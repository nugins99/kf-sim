#pragma once
#include <Types.h>  // Include the Types header for DOF, StateVec, StateMat

#include <Eigen/Dense>
#include <boost/property_tree/ptree.hpp>
#include <random>  // Include for std::mt19937 and std::uniform_real_distribution

#include "IMUSensor6D.h"
#include "PositionDepthSensor.h"
#include "SensorPreprocessorInterface.h"
#include "VehicleTruthModel.h"

/**
 * @class SensorPreprocessor
 * @brief Preprocesses sensor data for state estimation in a Kalman Filter.
 *
 * Handles simulation and preprocessing of IMU and position sensor data,
 * integrates velocity estimates, and provides measurement vectors for filtering.
 */
class SensorPreprocessor : public SensorPreprocessorInterface
{
   public:
    /**
     * @brief Constructor
     * @param imuAccelNoise Standard deviation of IMU accelerometer noise.
     * @param imuGyroNoise Standard deviation of IMU gyroscope noise.
     * @param posNoise Standard deviation of position sensor noise.
     */
    SensorPreprocessor(double imuAccelNoise, double imuGyroNoise, double posNoise);

    /**
     * @brief Constructor from property tree
     * @param pt boost::property_tree::ptree object for config
     */
    SensorPreprocessor(const boost::property_tree::ptree& pt);

    /**
     * @brief Initializes the preprocessor with an initial velocity.
     * @param initialVelocity Initial velocity vector (3D).
     */
    void initialize(const Eigen::Vector3d& initialVelocity);

    /**
     * @brief Generates a noisy measurement vector based on the truth model and filter estimate.
     * @param truth Pointer to the vehicle truth model.
     * @param dt Time step size.
     * @param step Current simulation step.
     * @param estimate Current Kalman Filter state estimate.
     * @return StateVec Measurement vector.
     */
    StateVec getMeasurement(VehicleTruthModel* truth, double dt, int step, const StateVec& estimate);

    /**
     * @brief Updates the internally integrated velocity using the current filter state.
     * @param state Current Kalman Filter state vector.
     */
    void updateState(const StateVec& state);

    /**
     * @brief Returns the current integrated velocity vector.
     * @return Reference to the integrated velocity.
     */
    const Eigen::Vector3d& getIntegratedVel() const;

   private:
    IMUSensor6D m_imu;                ///< IMU sensor simulation object
    PositionDepthSensor m_position;   ///< Position and depth sensor simulation object
    Eigen::Vector3d m_integratedVel;  ///< Integrated velocity vector
    std::mt19937 m_rng;               ///< Random number generator for dropout
    std::uniform_real_distribution<> m_dropoutDist; ///< Dropout distribution
};
