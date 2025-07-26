#pragma once
#include <iomanip>
#include <iostream>

#include "OutputInterface.h"

/**
 * @brief Progress reporter that outputs simulation progress to std::cerr.
 */
class ProgressReporter : public OutputInterface
{
   public:
    ProgressReporter(int interval = 1000) : interval_(interval) {}
    void report(int step, const Eigen::Vector3d& tpos, [[maybe_unused]] const Eigen::Vector3d& tvel,
                [[maybe_unused]] double tspeed, [[maybe_unused]] double theading,
                const Eigen::Matrix<double, 6, 1>& est, [[maybe_unused]] double espeed,
                [[maybe_unused]] double eheading, const Eigen::Vector3d& measurement,
                double position_error) override
    {
        if (step % interval_ == 0)
        {
            std::cerr << std::setprecision(3) << std::fixed;
            std::cerr << "# Step " << step << ": "
                      << "True Position: " << tpos.transpose() << ", "
                      << "Estimated Position: " << est.head<3>().transpose() << ", "
                      << "Position Error: " << position_error << ", "
                      << "Integrated Velocity: " << measurement.transpose() << std::endl;
        }
    }

   private:
    int interval_;
};
