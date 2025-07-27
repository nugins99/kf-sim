#pragma once
#include <Eigen/Dense>
#include <boost/property_tree/ptree.hpp>
#include "VehicleTruthModel.h"
#include <Types.h> // For DOF, StateVec, StateMat

/**
 * @class SensorPreprocessorInterface
 * @brief Abstract interface for sensor preprocessing in state estimation.
 */
class SensorPreprocessorInterface {
public:
    virtual ~SensorPreprocessorInterface() = default;

    /**
     * @brief Initializes the preprocessor with an initial velocity.
     * @param initialVel Initial velocity vector (3D).
     */
    virtual void initialize(const Eigen::Vector3d& initiallVelocity) = 0;

    /**
     * @brief Generates a noisy measurement vector based on the truth model and filter estimate.
     * @param truth Pointer to the vehicle truth model.
     * @param dt Time step size.
     * @param step Current simulation step.
     * @param estimate Current Kalman Filter state estimate.
     * @return StateVec Measurement vector.
     */
    virtual StateVec getMeasurement(VehicleTruthModel* truth, double dt, int step, const StateVec& estimate) = 0;

    /**
     * @brief Updates the internally integrated velocity using the current filter state.
     * @param state Current Kalman Filter state vector.
     */
    virtual void updateState(const StateVec& state) = 0;

    /**
     * @brief Returns the current integrated velocity vector.
     * @return Reference to the integrated velocity.
     */
    virtual const Eigen::Vector3d& getIntegratedVel() const = 0;
};
