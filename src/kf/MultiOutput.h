#pragma once
#include "OutputInterface.h"
#include <vector>
#include <memory>

/**
 * @brief OutputInterface implementation that distributes output to multiple registered outputs.
 */
class MultiOutput : public OutputInterface {
public:
    void addOutput(std::shared_ptr<OutputInterface> output) {
        outputs_.push_back(output);
    }
    void report(int step,
                const Eigen::Vector3d& tpos,
                const Eigen::Vector3d& tvel,
                double tspeed,
                double theading,
                const Eigen::Matrix<double, 6, 1>& est,
                double espeed,
                double eheading,
                const Eigen::Vector3d& measurement,
                double position_error) override {
        for (auto& out : outputs_) {
            out->report(step, tpos, tvel, tspeed, theading, est, espeed, eheading, measurement, position_error);
        }
    }
private:
    std::vector<std::shared_ptr<OutputInterface>> outputs_;
};
