#include "PureRL_Torque.h"

void PureRL_Torque::configure(const mc_rtc::Configuration & config)
{
}

void PureRL_Torque::start(mc_control::fsm::Controller & ctl)
{
  utils::start_rl_state(ctl, "PureRL_Torque");
  auto & ctl_rl = static_cast<RLController&>(ctl);

  ctl_rl.datastore().get<std::string>("ControlMode") = "Torque";
  ctl_rl.useQP = false;
  ctl_rl.taskType = 1;

  mc_rtc::log::info("using Pure RL (no QP) and Torque control");
}

bool PureRL_Torque::run(mc_control::fsm::Controller & ctl)
{
  utils::run_rl_state(ctl, "PureRL_Torque");
  return false;
}

void PureRL_Torque::teardown(mc_control::fsm::Controller & ctl)
{
  utils::teardown_rl_state(ctl, "PureRL_Torque");
}

EXPORT_SINGLE_STATE("PureRL_Torque", PureRL_Torque)
