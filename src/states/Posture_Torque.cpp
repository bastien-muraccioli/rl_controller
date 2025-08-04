#include "Posture_Torque.h"
#include <mc_rtc/logging.h>

void Posture_Torque::configure(const mc_rtc::Configuration & config)
{
}

void Posture_Torque::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController&>(ctl_);
  mc_rtc::log::info("Posture_Torque state started");

  ctl.datastore().get<std::string>("ControlMode") = "Torque";
  ctl.static_pos = true;

  mc_rtc::log::success("Posture_Torque state initialization completed");

  mc_rtc::log::info("Following default Posture with Torque control");
}

bool Posture_Torque::run(mc_control::fsm::Controller & ctl_)
{ 
  return false;
}

void Posture_Torque::teardown(mc_control::fsm::Controller & ctl_)
{
}

EXPORT_SINGLE_STATE("Posture_Torque", Posture_Torque)
