#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <memory>

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

    std::pair<Eigen::Vector3d, Eigen::Vector3d> next(double dt)
    {
        step(dt);
        return {getPosition(), getVelocity()};
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
