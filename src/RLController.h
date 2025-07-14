#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorqueTask.h>
#include <mc_tasks/PostureTask.h>
#include "api.h"

struct RLController_DLLAPI RLController : public mc_control::fsm::Controller
{
  RLController(mc_rbdyn::RobotModulePtr rm, double dt, 
               const mc_rtc::Configuration & config);
 
  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
  
  std::unique_ptr<mc_solver::ContactConstraint> contactConstraintTest;
  // std::unique_ptr<RLPolicyInterface> rlPolicy_;
  std::shared_ptr<mc_tasks::TorqueTask> torqueTask;
  std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask;

  std::vector<std::vector<double>> desiredPosture;
  std::vector<std::string> jointNames;
  std::map<std::string, std::vector<double>> postureTarget;
  // std::map<std::string, std::vector<double>> torqueTarget;

  std::map<std::string, double> kp;
  std::map<std::string, double> kd;
  size_t dofNumber = 0; // Number of degrees of freedom in the robot

  std::map<std::string, std::vector<double>> convertPosToTorque(std::map<std::string, std::vector<double>> & posTarget);

private:
}; 