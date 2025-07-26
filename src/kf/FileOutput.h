#pragma once
#include <fstream>
#include <string>

#include "OutputInterface.h"

/**
 * @brief Concrete output implementation for writing simulation results to a file.
 */
class FileOutput : public OutputInterface
{
   public:
    explicit FileOutput(const std::string& filename) : ofs_(filename)
    {
        ofs_ << "# step true_x true_y true_z true_vx true_vy true_vz true_speed true_heading "
                "est_x est_y est_z est_vx est_vy est_vz est_speed est_heading range bearing depth "
                "position_error"
             << std::endl;
    }
    ~FileOutput() override { ofs_.close(); }

    void report(int step, const Eigen::Vector3d& tpos, const Eigen::Vector3d& tvel, double tspeed,
                double theading, const Eigen::Matrix<double, 6, 1>& est, double espeed,
                double eheading, const Eigen::Vector3d& measurement, double position_error) override
    {
        ofs_ << step << " " << tpos(0) << " " << tpos(1) << " " << tpos(2) << " " << tvel(0) << " "
             << tvel(1) << " " << tvel(2) << " " << tspeed << " " << theading << " " << est(0)
             << " " << est(1) << " " << est(2) << " " << est(3) << " " << est(4) << " " << est(5)
             << " " << espeed << " " << eheading << " " << measurement(0) << " " << measurement(1)
             << " " << measurement(2) << " " << position_error << std::endl;
    }

   private:
    std::ofstream ofs_;
};
