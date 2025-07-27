#pragma once
#include <Eigen/Dense>
#include <functional>

#include "KalmanFilterInterface.h"
#include "Types.h"

/**
 * @brief Extended Kalman Filter (EKF) class for nonlinear models.
 *
 * The EKF generalizes the standard Kalman Filter to handle nonlinear state transition and
 * measurement models. Instead of assuming linear dynamics, the EKF uses user-provided nonlinear
 * functions for the process and measurement models, and linearizes them at each step using their
 * Jacobians. This allows the filter to estimate states for systems where the evolution and/or
 * observation equations are nonlinear.
 *
 * Differences from a standard Kalman Filter:
 * - Standard Kalman Filter assumes linear process and measurement models (x' = F x, z = H x).
 * - EKF allows arbitrary nonlinear models (x' = f(x, dt), z = h(x)), provided by the user.
 * - EKF requires the user to provide Jacobian functions for both the process and measurement
 * models.
 * - EKF linearizes the models at each step using the current state estimate.
 * - EKF is more flexible but can be less accurate if the system is highly nonlinear or the
 * linearization is poor.
 *
 * @tparam DOF Number of state dimensions.
 */
template <int DOF>
class ExtendedKalmanFilter : public KalmanFilterInterface<DOF>
{
   public:
    /**
     * @brief Nonlinear state transition function type.
     * @param x Current state vector.
     * @param dt Time step.
     * @return Predicted next state vector.
     */
    using StateFunc = std::function<StateVec(const StateVec&, double)>;
    /**
     * @brief Nonlinear measurement function type.
     * @param x Current state vector.
     * @return Predicted measurement vector.
     */
    using MeasFunc = std::function<StateVec(const StateVec&)>;
    /**
     * @brief Jacobian function type.
     * @param x Current state vector.
     * @param dt Time step.
     * @return Jacobian matrix (DOF x DOF).
     */
    using JacobianFunc = std::function<StateMat(const StateVec&, double)>;

    /**
     * @brief EKF constructor.
     * @param dt Time step (seconds).
     * @param process_noise Process noise covariance matrix.
     * @param measurement_noise Measurement noise covariance matrix.
     * @param f Nonlinear state transition function.
     * @param h Nonlinear measurement function.
     * @param F_jac Jacobian of state transition function.
     * @param H_jac Jacobian of measurement function.
     */
    ExtendedKalmanFilter(double dt, const StateMat& process_noise,
                         const StateMat& measurement_noise, StateFunc f, MeasFunc h,
                         JacobianFunc F_jac, JacobianFunc H_jac)
        : m_dt(dt),
          m_Q(process_noise),
          m_R(measurement_noise),
          m_f(f),
          m_h(h),
          m_F_jac(F_jac),
          m_H_jac(H_jac)
    {}

    /**
     * @brief Initialize filter state and covariance.
     * @param initial_state Initial state vector.
     * @param initial_covariance Initial state covariance matrix.
     */
    void init(const StateVec& initial_state, const StateMat& initial_covariance) override
    {
        m_x = initial_state;
        m_P = initial_covariance;
    }

    /**
     * @brief Predict the next state and covariance using the nonlinear process model.
     */
    void predict() override
    {
        m_x = m_f(m_x, m_dt);
        StateMat F = m_F_jac(m_x, m_dt);
        m_P = F * m_P * F.transpose() + m_Q;
    }

    /**
     * @brief Update the state and covariance with a new measurement using the nonlinear measurement
     * model.
     * @param measurement Measurement vector.
     */
    void update(const StateVec& measurement) override
    {
        StateVec y = measurement - m_h(m_x);
        StateMat H = m_H_jac(m_x, m_dt);
        StateMat S = H * m_P * H.transpose() + m_R;
        StateMat K = m_P * H.transpose() * S.inverse();
        m_x = m_x + K * y;
        m_P = (StateMat::Identity() - K * H) * m_P;
    }

    /**
     * @brief Get the current state estimate.
     * @return State vector.
     */
    StateVec getState() const override { return m_x; }
    /**
     * @brief Get the current state covariance.
     * @return State covariance matrix.
     */
    StateMat getCovariance() const override { return m_P; }

   private:
    double m_dt;           ///< Time step
    StateVec m_x;          ///< State vector
    StateMat m_P;          ///< State covariance
    StateMat m_Q;          ///< Process noise covariance
    StateMat m_R;          ///< Measurement noise covariance
    StateFunc m_f;         ///< Nonlinear state transition function
    MeasFunc m_h;          ///< Nonlinear measurement function
    JacobianFunc m_F_jac;  ///< Jacobian of state transition
    JacobianFunc m_H_jac;  ///< Jacobian of measurement
};
