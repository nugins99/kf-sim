#pragma once
#include <Eigen/Dense>

/**
 * @brief Templated Kalman Filter class for configurable DOF (dimensions of state).
 *
 * @tparam DOF Number of state dimensions (e.g., 4 for 2D position and velocity).
 */
template <int DOF>
class KalmanFilter
{
   public:
    /**
     * @brief State vector type (DOF x 1).
     */
    using StateVec = Eigen::Matrix<double, DOF, 1>;
    /**
     * @brief State matrix type (DOF x DOF).
     */
    using StateMat = Eigen::Matrix<double, DOF, DOF>;

    /**
     * @brief Constructor for KalmanFilter.
     * @param dt Time step (seconds).
     * @param process_noise Process noise covariance matrix (DOF x DOF).
     * @param measurement_noise Measurement noise covariance matrix (DOF x DOF).
     */
    KalmanFilter(double dt, const StateMat& process_noise, const StateMat& measurement_noise)
        : dt_(dt), Q_(process_noise), R_(measurement_noise)
    {
        F_ = StateMat::Identity();
        // For position-velocity model, set F(i, i+DOF/2) = dt
        for (int i = 0; i < DOF / 2; ++i)
        {
            F_(i, i + DOF / 2) = dt_;
        }
        H_ = StateMat::Identity();
    }

    /**
     * @brief Initialize filter state and covariance.
     * @param initial_state Initial state vector (DOF x 1).
     * @param initial_covariance Initial state covariance matrix (DOF x DOF).
     */
    void init(const StateVec& initial_state, const StateMat& initial_covariance)
    {
        x_ = initial_state;
        P_ = initial_covariance;
    }

    /**
     * @brief Predict the next state and covariance using the process model.
     */
    void predict()
    {
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    /**
     * @brief Update the state and covariance with a new measurement.
     * @param measurement Measurement vector (DOF x 1).
     */
    void update(const StateVec& measurement)
    {
        StateVec y = measurement - H_ * x_;
        StateMat S = H_ * P_ * H_.transpose() + R_;
        StateMat K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + K * y;
        P_ = (StateMat::Identity() - K * H_) * P_;
    }

    /**
     * @brief Get the current state estimate.
     * @return State vector (DOF x 1).
     */
    StateVec getState() const { return x_; }
    /**
     * @brief Get the current state covariance.
     * @return State covariance matrix (DOF x DOF).
     */
    StateMat getCovariance() const { return P_; }

   private:
    double dt_;   ///< Time step
    StateVec x_;  ///< State vector
    StateMat P_;  ///< State covariance
    StateMat Q_;  ///< Process noise covariance
    StateMat R_;  ///< Measurement noise covariance
    StateMat F_;  ///< State transition matrix
    StateMat H_;  ///< Measurement matrix
};