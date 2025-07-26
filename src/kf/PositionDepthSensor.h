#pragma once
#include <Eigen/Dense>
#include <boost/property_tree/ptree.hpp>
#include <random>

#include "VehicleTruthModel.h"

/**
 * @brief Simulated Range/Bearing/Depth sensor for 6DOF vehicle, outputs noisy position (x, y, z).
 */
class PositionDepthSensor
{
   public:
    /**
     * @brief Constructor
     * @param pos_noise_std Standard deviation of position noise
     */
    PositionDepthSensor(double pos_noise_std);

    /**
     * @brief Constructor with drift rate
     * @param pos_noise_std Standard deviation of position noise
     * @param drift_rate Drift rate vector for position measurements
     */
    PositionDepthSensor(double pos_noise_std, const Eigen::Vector3d& drift_rate);

    /**
     * @brief Constructor from property tree
     * @param pt boost::property_tree::ptree object for config
     */
    PositionDepthSensor(const boost::property_tree::ptree& pt);

    /**
     * @brief Simulate a measurement from the sensor using the truth model.
     * @param truth Pointer to the truth model.
     * @return 3D vector [x, y, z] with noise.
     */
    Eigen::Vector3d measure(const VehicleTruthModel* truth);

   private:
    std::random_device rd_;
    std::default_random_engine gen_;
    std::normal_distribution<double> pos_noise_;
    Eigen::Vector3d drift_ = Eigen::Vector3d::Zero();  ///< Drift vector for position measurements
    Eigen::Vector3d drift_rate_ =
        Eigen::Vector3d::Zero();  ///< Drift rate for position measurements
};
