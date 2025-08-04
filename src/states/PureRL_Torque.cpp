#include "PureRL_Torque.h"

void PureRL_Torque::configure(const mc_rtc::Configuration & config)
{
}

void PureRL_Torque::start(mc_control::fsm::Controller & ctl)
{
  RL_utils::start_rl_state(ctl, "PureRL_Torque");
  auto & ctl_rl = static_cast<RLController&>(ctl);
  
  ctl_rl.datastore().get<std::string>("ControlMode") = "Torque";
  ctl_rl.useQP = false;

  mc_rtc::log::info("using Pure RL (no QP) and Torque control");
}

bool PureRL_Torque::run(mc_control::fsm::Controller & ctl)
{
  return RL_utils::run_rl_state(ctl, "PureRL_Torque");
  // output("OK");
  // return false;
}

void PureRL_Torque::teardown(mc_control::fsm::Controller & ctl)
{
  RL_utils::teardown_rl_state(ctl, "PureRL_Torque");
}

EXPORT_SINGLE_STATE("PureRL_Torque", PureRL_Torque)
