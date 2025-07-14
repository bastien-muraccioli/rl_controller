#include "RLController_Initial.h"
#include "../RLController.h"
#include <mc_rtc/logging.h>

void RLController_Initial::configure(const mc_rtc::Configuration & config)
{
  // No configuration needed for initial state
}

void RLController_Initial::start(mc_control::fsm::Controller & ctl)
{
  auto & ctl_ = static_cast<RLController &>(ctl);
  postureTarget = ctl_.postureTarget;
  mc_rtc::log::info("RLController_Initial state started");
}

bool RLController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController &>(ctl_);
  if(ctl.robot().encoderVelocities().empty())
  {
    mc_rtc::log::warning("[RLController_Initial] No encoder velocities available, skipping residual computation");
    return false;
  }
  // output("OK");
  // ctl.torqueTarget = ctl.convertPosToTorque(ctl.postureTarget);
  if(isTorqueTask)
  {
    auto target = ctl.convertPosToTorque(postureTarget);
    ctl.torqueTask->target(target);
  }
  else
  {
    if(ctl.FSMPostureTask->eval().norm() < 0.001)
    {
      ctl.FSMPostureTask->weight(0.0);
      ctl.torqueTask->weight(1000.0);
      auto target = ctl.convertPosToTorque(postureTarget);
      mc_rtc::log::info("RLController_Initial: Giving a torque target to TorqueTask");
      ctl.torqueTask->target(target);
      mc_rtc::log::info("RLController_Initial: Switching to TorqueTask");
      ctl.solver().addTask(ctl.torqueTask);
      isTorqueTask = true;
    }
  }
  
  return false;
}

void RLController_Initial::teardown(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("RLController_Initial state ending");
}

EXPORT_SINGLE_STATE("RLController_Initial", RLController_Initial)
