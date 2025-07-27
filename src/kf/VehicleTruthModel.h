#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <Types.h>

/**
 * @brief Interface for vehicle truth motion models (6DOF).
 */
class VehicleTruthModel
{
   public:
    virtual ~VehicleTruthModel() = default;
    /**
     * @brief Advance the model by one time step.
     * @param dt Time step (seconds).
     */
    virtual void step(double dt) = 0;

    /**
     * @brief Get the current true state as a 6D vector [x, y, z, vx, vy, vz].
     * @return State vector.
     */
    StateVec next(double dt)
    {
        step(dt);
        return getState();
    }

    /**
     * @brief Get the current true state as a 6D vector [x, y, z, vx, vy, vz].
     * @return State vector.
     */
    virtual StateVec getState() const
    {
        // Create a state vector from the current position and velocity
        StateVec state;
        state << getPosition()(0), getPosition()(1), getPosition()(2),
                 getVelocity()(0), getVelocity()(1), getVelocity()(2);
        return state;
    }

    /**
     * @brief Get the current true position [x, y, z].
     */
    virtual Eigen::Vector3d getPosition() const = 0;
    /**
     * @brief Get the current true velocity [vx, vy, vz].
     */
    virtual Eigen::Vector3d getVelocity() const = 0;
    /**
     * @brief Get the current true orientation [roll, pitch, yaw].
     */
    virtual Eigen::Vector3d getOrientation() const = 0;
    /**
     * @brief Get the current true angular velocity [wx, wy, wz].
     */
    virtual Eigen::Vector3d getAngularVelocity() const = 0;
};
