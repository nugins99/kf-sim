#pragma once
#include <Eigen/Dense>
#include "Types.h"  // Include Types.h for StateVec and StateMat definitions
/**
 * @brief Interface for Kalman Filter classes.
 *
 * This interface defines the required methods for any Kalman Filter implementation, including
 * standard, extended, and unscented variants. It provides a consistent API for initializing,
 * predicting, updating, and retrieving state and covariance estimates.
 *
 * @tparam DOF Number of state dimensions.
 */
template <int DOF>
class KalmanFilterInterface
{
   public:
    virtual ~KalmanFilterInterface() = default;
    /**
     * @brief Initialize filter state and covariance.
     * @param initial_state Initial state vector.
     * @param initial_covariance Initial state covariance matrix.
     */
    virtual void init(const StateVec& initial_state, const StateMat& initial_covariance) = 0;
    /**
     * @brief Predict the next state and covariance using the process model.
     */
    virtual void predict() = 0;
    /**
     * @brief Update the state and covariance with a new measurement.
     * @param measurement Measurement vector.
     */
    virtual void update(const StateVec& measurement) = 0;
    /**
     * @brief Get the current state estimate.
     * @return State vector.
     */
    virtual StateVec getState() const = 0;
    /**
     * @brief Get the current state covariance.
     * @return State covariance matrix.
     */
    virtual StateMat getCovariance() const = 0;
};
