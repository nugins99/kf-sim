#pragma once
#include <boost/property_tree/ptree.hpp>
#include <memory>

#include "ExtendedKalmanFilter.h"
#include "KalmanFilter.h"
#include "KalmanFilterInterface.h"
#include "Types.h"

/**
 * @brief Factory function to create a Kalman filter implementation based on config.
 * @tparam DOF Number of state dimensions.
 * @param pt Property tree with configuration.
 * @param dt Time step.
 * @param Q Process noise covariance.
 * @param R Measurement noise covariance.
 * @return Unique pointer to KalmanFilterInterface<DOF>.
 */
template <int DOF>
std::unique_ptr<KalmanFilterInterface<DOF>> createKalmanFilterFromConfig(
    const boost::property_tree::ptree& pt, double dt, const StateMat& Q,
    const StateMat& R)
{
    std::string type = pt.get<std::string>("filter.type", "standard");
    if (type == "ekf")
    {
        auto f = [](const StateVec& x, [[maybe_unused]] double dt)
        { return x; };  // Identity for demo
        auto h = [](const StateVec& x) { return x; };
        auto F_jac = []([[maybe_unused]] const StateVec& x, [[maybe_unused]] double dt)
        { return StateMat::Identity(); };
        auto H_jac = []([[maybe_unused]] const StateVec& x, [[maybe_unused]] double dt)
        { return StateMat::Identity(); };
        return std::make_unique<ExtendedKalmanFilter<DOF>>(dt, Q, R, f, h, F_jac, H_jac);
    }
    else
    {
        return std::make_unique<KalmanFilter<DOF>>(dt, Q, R);
    }
}
