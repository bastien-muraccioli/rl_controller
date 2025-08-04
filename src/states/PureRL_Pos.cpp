#include "PureRL_Pos.h"

void PureRL_Pos::configure(const mc_rtc::Configuration & config)
{
}

void PureRL_Pos::start(mc_control::fsm::Controller & ctl)
{
  RL_utils::start_rl_state(ctl, "PureRL_Pos");
  auto & ctl_rl = static_cast<RLController&>(ctl);

  ctl_rl.datastore().get<std::string>("ControlMode") = "Position";
  ctl_rl.useQP = false;

  mc_rtc::log::info("using Pure RL (no QP) and Position control");
}

bool PureRL_Pos::run(mc_control::fsm::Controller & ctl)
{
  return RL_utils::run_rl_state(ctl, "PureRL_Pos");
  // output("OK");
  // return false;
}

void PureRL_Pos::teardown(mc_control::fsm::Controller & ctl)
{
  RL_utils::teardown_rl_state(ctl, "PureRL_Pos");
}

EXPORT_SINGLE_STATE("PureRL_Pos", PureRL_Pos)
