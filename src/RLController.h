#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorqueTask.h>
#include "RLPolicyInterface.h"
#include <memory>
#include <Eigen/Dense>
#include "api.h"

struct RLController : public mc_control::fsm::Controller
{
  RLController(mc_rbdyn::RobotModulePtr rm, double dt, 
               const mc_rtc::Configuration & config);
  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
  
  std::unique_ptr<RLPolicyInterface> rlPolicy_;
  std::shared_ptr<mc_tasks::TorqueTask> torqueTask_;
  
  std::vector<std::string> allJoints_;      // All 19 joints in mc_rtc order
  std::vector<std::string> legJoints_;      // 10 leg joints only
  
  std::vector<int> maniskillToMcRtcIdx_;    // Action reordering: Maniskill -> mc_rtc
  std::vector<int> mcRtcToManiskillIdx_;    // Observation reordering: mc_rtc -> Maniskill
  
  // Past action for observation (size 19)
  Eigen::VectorXd pastAction_;
  
  Eigen::VectorXd kp_;  // Position gains (19 joints)
  Eigen::VectorXd kd_;  // Velocity gains (19 joints)
  
  Eigen::VectorXd getCurrentObservation();
  void applyAction(const Eigen::VectorXd & action);
  
  Eigen::VectorXd reorderObservationToManiskill(const Eigen::VectorXd & obs);
  Eigen::VectorXd reorderActionFromManiskill(const Eigen::VectorXd & action);
  
  Eigen::VectorXd computeImpedanceTorques(const Eigen::VectorXd & desiredPos, 
                                         const Eigen::VectorXd & currentPos,
                                         const Eigen::VectorXd & currentVel);

  void supported_robots(std::vector<std::string> & out) const override 
  { 
    out = {"h1"}; 
  }

private:
  void initializeAllJoints();
  void initializeImpedanceGains();
}; 