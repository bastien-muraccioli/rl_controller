#include "RL_QP_Pos.h"

void RL_QP_Pos::configure(const mc_rtc::Configuration & config)
{
}

void RL_QP_Pos::start(mc_control::fsm::Controller & ctl)
{
  RL_utils::start_rl_state(ctl, "RL_QP_Pos");
  auto & ctl_rl = static_cast<RLController&>(ctl);
  
  ctl_rl.datastore().get<std::string>("ControlMode") = "Position";
  ctl_rl.useQP = true;

  mc_rtc::log::info("using RL with QP and Position control");
}

bool RL_QP_Pos::run(mc_control::fsm::Controller & ctl)
{
  return RL_utils::run_rl_state(ctl, "RL_QP_Pos");
  // output("OK");
  // return true;
}

void RL_QP_Pos::teardown(mc_control::fsm::Controller & ctl)
{
  RL_utils::teardown_rl_state(ctl, "RL_QP_Pos");
}

EXPORT_SINGLE_STATE("RL_QP_Pos", RL_QP_Pos)
