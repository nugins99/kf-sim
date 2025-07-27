#pragma once
#include <iomanip>
#include <ios>
#include <iostream>

#include "OutputInterface.h"

/**
 * @brief Progress reporter that outputs simulation progress to std::cerr.
 */
class ProgressReporter : public OutputInterface
{
   public:
    /**
     * @brief Constructor
     * @param steps Total number of simulation steps for progress reporting.
     */
    explicit ProgressReporter(int steps) : m_totalSteps(steps) {}

    void report(int step, const StateVec& truth_state, const StateVec& kf_state,
                [[maybe_unused]] const StateMat& kf_cov) override
    {
        // Simple progress output: print step and position error
        double position_error = (truth_state.head<3>() - kf_state.head<3>()).norm();
        auto progress = (static_cast<float>(step) / m_totalSteps) * 100;
        std::cout << std::fixed << std::setprecision(1) << std::setw(4) << progress << "% "
                  << " Position Error: " << std::setw(6) << position_error << std::endl;
    }

   private:
    int m_totalSteps;  ///< Total number of simulation steps for progress reporting
};
