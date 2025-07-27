#pragma once
#include <Eigen/Dense>
#include <string>
#include "Types.h"  // Include Types.h for StateVec and StateMat definitions

/**
 * @brief Abstract interface for simulation output reporting.
 */
class OutputInterface
{
   public:

    virtual ~OutputInterface() = default;
    /**
     * @brief Report a single simulation step.
     * @param step Simulation step index.
     * @param truth_state True state vector (position + velocity).
     * @param kf_state Estimated state vector (position + velocity).
     * @param kf_cov State covariance matrix.
     */
    virtual void report(int step, const StateVec& truth_state, const StateVec& kf_state, const StateMat& kf_cov) = 0;
};
