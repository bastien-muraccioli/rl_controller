#include "Posture_TorqueTask_Torque.h"
#include <mc_rtc/logging.h>

void Posture_TorqueTask_Torque::configure(const mc_rtc::Configuration & config)
{
}

void Posture_TorqueTask_Torque::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController&>(ctl_);
  mc_rtc::log::info("Posture_TorqueTask_Torque state started");

  ctl.datastore().get<std::string>("ControlMode") = "Torque";
  ctl.static_pos = true;
  ctl.useQP = false;
  ctl.taskType = 0;

  mc_rtc::log::success("Posture_TorqueTask_Torque state initialization completed");

  mc_rtc::log::info("Following default Posture with Torque control");
}

bool Posture_TorqueTask_Torque::run(mc_control::fsm::Controller & ctl_)
{
  return false;
}

void Posture_TorqueTask_Torque::teardown(mc_control::fsm::Controller & ctl_)
{
}

EXPORT_SINGLE_STATE("Posture_TorqueTask_Torque", Posture_TorqueTask_Torque)
