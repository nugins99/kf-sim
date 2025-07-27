#pragma once
#include <fstream>
#include <string>
#include <cmath>

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
                "est_x est_y est_z est_vx est_vy est_vz est_speed est_heading position_error velocity_error std_dev_x std_dev_y std_dev_z std_dev_vx std_dev_vy std_dev_vz"
             << std::endl;
    }
    ~FileOutput() override { ofs_.close(); }

    /**
     * @brief Output file column mapping (for gnuplot and analysis):
     *   1: step
     *   2: true_x
     *   3: true_y
     *   4: true_z
     *   5: true_vx
     *   6: true_vy
     *   7: true_vz
     *   8: true_speed
     *   9: true_heading
     *  10: est_x
     *  11: est_y
     *  12: est_z
     *  13: est_vx
     *  14: est_vy
     *  15: est_vz
     *  16: est_speed
     *  17: est_heading
     *  18: position_error (||true_pos - est_pos||)
     *  19: velocity_error (||true_vel - est_vel||)
     *  20-25: std_devs (sqrt of diagonal of covariance matrix)
     */
    void report(int step, const StateVec& truth_state, const StateVec& kf_state, const StateMat& kf_cov) override
    {
        // Extract position and velocity
        Eigen::Vector3d tpos = truth_state.head<3>();
        Eigen::Vector3d tvel = truth_state.tail<3>();
        Eigen::Vector3d epos = kf_state.head<3>();
        Eigen::Vector3d evel = kf_state.tail<3>();
        double tspeed = tvel.head<2>().norm();
        double theading = std::atan2(tvel(1), tvel(0)) * 180.0 / M_PI;
        double espeed = evel.head<2>().norm();
        double eheading = std::atan2(evel(1), evel(0)) * 180.0 / M_PI;
        double position_error = (tpos - epos).norm();
        double velocity_error = (tvel - evel).norm();
        ofs_ << step;
        ofs_ << " " << tpos(0) << " " << tpos(1) << " " << tpos(2);
        ofs_ << " " << tvel(0) << " " << tvel(1) << " " << tvel(2);
        ofs_ << " " << tspeed << " " << theading << " ";
        ofs_ << epos(0) << " " << epos(1) << " " << epos(2) << " ";
        ofs_ << evel(0) << " " << evel(1) << " " << evel(2) << " ";
        ofs_ << espeed << " " << eheading << " ";
        ofs_ << position_error << " " << velocity_error << " ";
        for (int i = 0; i < 6; ++i) ofs_ << std::sqrt(kf_cov(i, i)) << (i < 5 ? " " : "");
        ofs_ << std::endl;
    }

   private:
    std::ofstream ofs_;
};
