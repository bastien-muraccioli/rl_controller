#include "PolicySimulatorHandling.h"
#include <mc_rtc/logging.h>
#include <string>
#include <vector>

PolicySimulatorHandling::PolicySimulatorHandling()
{
  // Default constructor - no simulator handling
}

PolicySimulatorHandling::PolicySimulatorHandling(const std::string& simulator_name):
    simulatorName(simulator_name)
{
    if(simulator_name == "Maniskill")
    {
        maniskillToMcRtcIdx_ = {0, 3, 7, 11, 15, 1, 4, 8, 12, 16, 2, 5, 9, 13, 17, 6, 10, 14, 18};
        mcRtcToManiskillIdx_ = {0, 5, 10, 1, 6, 11, 15, 2, 7, 12, 16, 3, 8, 13, 17, 4, 9, 14, 18};
    }
    else {
        mc_rtc::log::error_and_throw("Unsupported simulator: {}", simulator_name);
    }
}

PolicySimulatorHandling::~PolicySimulatorHandling()
{
}

Eigen::VectorXd PolicySimulatorHandling::reorderJointsToSimulator(const Eigen::VectorXd & obs, size_t dofNumber)
{  
  if(obs.size() != dofNumber) {
    mc_rtc::log::error("Observation reordering expects dofNumber joints, got {}", obs.size());
    return obs;
  }
  
  Eigen::VectorXd reordered = Eigen::VectorXd::Zero(dofNumber);
  if (simulatorName == "Maniskill")
  {
    for(int i = 0; i < dofNumber; i++) {
        if(i >= mcRtcToManiskillIdx_.size()) {
        mc_rtc::log::error("Trying to access mcRtcToManiskillIdx_[{}] but size is {}", i, mcRtcToManiskillIdx_.size());
        reordered[i] = 0.0;
        continue;
        }
        
        int srcIdx = mcRtcToManiskillIdx_[i];
        if(srcIdx >= obs.size()) {
        mc_rtc::log::error("Index {} out of bounds for obs size {}", srcIdx, obs.size());
        reordered[i] = 0.0;
        } else {
        reordered[i] = obs(srcIdx);
        }
    }
  }
  return reordered;
}

Eigen::VectorXd PolicySimulatorHandling::reorderJointsFromSimulator(const Eigen::VectorXd & action, size_t dofNumber)
{  
  if(action.size() != dofNumber) {
    mc_rtc::log::error("Action reordering expects dofNumber joints, got {}", action.size());
    return action;
  }
  
  Eigen::VectorXd reordered = Eigen::VectorXd::Zero(dofNumber);
  if (simulatorName == "Maniskill")
  {
    for(int i = 0; i < dofNumber; i++) {
        if(i >= maniskillToMcRtcIdx_.size()) {
        mc_rtc::log::error("Trying to access maniskillToMcRtcIdx_[{}] but size is {}", i, maniskillToMcRtcIdx_.size());
        reordered[i] = 0.0;
        continue;
        }
        
        int srcIdx = maniskillToMcRtcIdx_[i];
        if(srcIdx >= action.size()) {
        mc_rtc::log::error("Action reorder index {} out of bounds for action size {}", srcIdx, action.size());
        reordered[i] = 0.0;
        } else {
        reordered[i] = action(srcIdx);
        }
    }
  }
  return reordered;
}

std::vector<int> PolicySimulatorHandling::getSimulatorIndices(std::vector<int> mcRtcIndices) const
{
    if(simulatorName == "Maniskill")
    {
        std::vector<int> maniskillIndices;
        for(int idx : mcRtcIndices)
        {
            if(idx < mcRtcToManiskillIdx_.size())
            {
                maniskillIndices.push_back(maniskillToMcRtcIdx_[idx]);
            }
            else
            {
                mc_rtc::log::error("Index {} out of bounds for mcRtcToManiskillIdx_ size {}", idx, mcRtcToManiskillIdx_.size());
            }
        }
        return maniskillIndices;
    }
    return {};
}