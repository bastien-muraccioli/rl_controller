#include "Posture_FDTask_Pos.h"
#include <mc_rtc/logging.h>

void Posture_FDTask_Pos::configure(const mc_rtc::Configuration & config)
{
}

void Posture_FDTask_Pos::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController&>(ctl_);
  mc_rtc::log::info("Posture_FDTask_Pos state started");

  ctl.datastore().get<std::string>("ControlMode") = "Position";
  ctl.static_pos = true;
  ctl.useQP = false;
  ctl.taskType = 1;

  mc_rtc::log::success("Posture_FDTask_Pos state initialization completed");

  mc_rtc::log::info("Following default Posture with Position control");
}

bool Posture_FDTask_Pos::run(mc_control::fsm::Controller & ctl_)
{
  return false;
}

void Posture_FDTask_Pos::teardown(mc_control::fsm::Controller & ctl_)
{
}

EXPORT_SINGLE_STATE("Posture_FDTask_Pos", Posture_FDTask_Pos)
