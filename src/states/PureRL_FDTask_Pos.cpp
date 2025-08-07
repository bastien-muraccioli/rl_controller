#include "PureRL_FDTask_Pos.h"

void PureRL_FDTask_Pos::configure(const mc_rtc::Configuration & config)
{
}

void PureRL_FDTask_Pos::start(mc_control::fsm::Controller & ctl)
{
  utils::start_rl_state(ctl, "PureRL_FDTask_Pos");
  auto & ctl_rl = static_cast<RLController&>(ctl);

  ctl_rl.datastore().get<std::string>("ControlMode") = "Position";
  ctl_rl.useQP = false;
  ctl_rl.taskType = 1;

  ctl.FDTask->stiffness(0.0);
  ctl.FDTask->damping(0.0);
  ctl.FDTaskComputation();
  ctl.FDTask->refAccel(ctl.refAccel);
  ctl.solver().addTask(ctl.FDTask);

  mc_rtc::log::info("using Pure RL (no QP) and Position control");
}

bool PureRL_FDTask_Pos::run(mc_control::fsm::Controller & ctl)
{
  utils::run_rl_state(ctl, "PureRL_FDTask_Pos");
  ctl.FDTask->refAccel(ctl.refAccel);
  return false;
}

void PureRL_FDTask_Pos::teardown(mc_control::fsm::Controller & ctl)
{
  auto & ctl = static_cast<RLController &>(ctl_);
  ctl.solver().removeTask(ctl.FDTask);
  utils::teardown_rl_state(ctl, "PureRL_FDTask_Pos");
}

EXPORT_SINGLE_STATE("PureRL_FDTask_Pos", PureRL_FDTask_Pos)
