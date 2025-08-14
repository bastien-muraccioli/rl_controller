#include "PureRL_Pos.h"

void PureRL_Pos::configure(const mc_rtc::Configuration & config)
{
}

void PureRL_Pos::start(mc_control::fsm::Controller & ctl)
{
  utils::start_rl_state(ctl, "PureRL_FDTask_Pos");
  auto & ctl_rl = static_cast<RLController&>(ctl);

  ctl_rl.datastore().get<std::string>("ControlMode") = "Position";
  ctl_rl.useQP = false;
  ctl_rl.taskType = 42;

  mc_rtc::log::info("using Pure RL (no QP) and Position control");
}

bool PureRL_Pos::run(mc_control::fsm::Controller & ctl)
{
  utils::run_rl_state(ctl, "PureRL_FDTask_Pos");
  return false;
}

void PureRL_Pos::teardown(mc_control::fsm::Controller & ctl)
{
  utils::teardown_rl_state(ctl, "PureRL_Pos");
}

EXPORT_SINGLE_STATE("PureRL_Pos", PureRL_Pos)
