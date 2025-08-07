#include "PureRL_FDTask_Torque.h"

void PureRL_FDTask_Torque::configure(const mc_rtc::Configuration & config)
{
}

void PureRL_FDTask_Torque::start(mc_control::fsm::Controller & ctl)
{
  utils::start_rl_state(ctl, "PureRL_FDTask_Torque");
  auto & ctl_rl = static_cast<RLController&>(ctl);

  ctl_rl.datastore().get<std::string>("ControlMode") = "Torque";
  ctl_rl.useQP = false;
  ctl_rl.taskType = 1;

  ctl.FDTask->stiffness(0.0);
  ctl.FDTask->damping(0.0);
  ctl.FDTaskComputation();
  ctl.FDTask->refAccel(ctl.refAccel);
  ctl.solver().addTask(ctl.FDTask);
  mc_rtc::log::info("using Pure RL (no QP) and Torque control");
}

bool PureRL_FDTask_Torque::run(mc_control::fsm::Controller & ctl)
{
  utils::run_rl_state(ctl, "PureRL_FDTask_Torque");
  ctl.FDTask->refAccel(ctl.refAccel);
  return false;
}

void PureRL_FDTask_Torque::teardown(mc_control::fsm::Controller & ctl)
{
  auto & ctl = static_cast<RLController &>(ctl_);
  ctl.solver().removeTask(ctl.FDTask);
  RL_utils::teardown_rl_state(ctl, "PureRL_FDTask_Torque");
}

EXPORT_SINGLE_STATE("PureRL_FDTask_Torque", PureRL_FDTask_Torque)
