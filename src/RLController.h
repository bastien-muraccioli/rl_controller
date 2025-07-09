#pragma once

#include <mc_control/fsm/Controller.h>
// #include <mc_tasks/TorqueTask.h>
#include <mc_tasks/CompliantPostureTask.h>

#include "api.h"

struct RLController_DLLAPI RLController : public mc_control::fsm::Controller
{
  RLController(mc_rbdyn::RobotModulePtr rm, double dt, 
               const mc_rtc::Configuration & config);
 
  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
  
  std::shared_ptr<mc_tasks::CompliantPostureTask> compPostureTask;
  std::unique_ptr<mc_solver::ContactConstraint> contactConstraintTest;

private:
}; 