#pragma once
#include <memory>
#include <vector>

#include "OutputInterface.h"

/**
 * @brief OutputInterface implementation that distributes output to multiple registered outputs.
 */
class MultiOutput : public OutputInterface
{
   public:
    void addOutput(std::shared_ptr<OutputInterface> output) { outputs_.push_back(output); }
    void report(int step, const StateVec& truth_state, const StateVec& kf_state, const StateMat& kf_cov) override
    {
        for (auto& out : outputs_)
            out->report(step, truth_state, kf_state, kf_cov);
    }
   private:
    std::vector<std::shared_ptr<OutputInterface>> outputs_;
};
