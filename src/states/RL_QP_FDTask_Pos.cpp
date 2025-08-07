#include "RL_QP_FDTask_Pos.h"

void RL_QP_FDTask_Pos::configure(const mc_rtc::Configuration & config)
{
}

void RL_QP_FDTask_Pos::start(mc_control::fsm::Controller & ctl)
{
  utils::start_rl_state(ctl, "RL_QP_FDTask_Pos");
  auto & ctl_rl = static_cast<RLController&>(ctl);
  
  ctl_rl.datastore().get<std::string>("ControlMode") = "Position";
  ctl_rl.useQP = true;
  ctl_rl.taskType = 1;

  ctl_rl.FDTask->stiffness(0.0);
  ctl_rl.FDTask->damping(0.0);
  ctl_rl.TasksSimulation(ctl_rl.q_zero_vector);
  ctl_rl.FDTask->refAccel(ctl_rl.refAccel);
  ctl_rl.solver().addTask(ctl_rl.FDTask);

  mc_rtc::log::info("using RL with QP and Position control");
}

bool RL_QP_FDTask_Pos::run(mc_control::fsm::Controller & ctl)
{
  auto & ctl_rl = static_cast<RLController&>(ctl);
  utils::run_rl_state(ctl, "RL_QP_FDTask_Pos");
  ctl_rl.FDTask->refAccel(ctl_rl.refAccel);
  return false;
}

void RL_QP_FDTask_Pos::teardown(mc_control::fsm::Controller & ctl)
{
  auto & ctl_rl = static_cast<RLController &>(ctl);
  ctl_rl.solver().removeTask(ctl_rl.FDTask);
  utils::teardown_rl_state(ctl, "RL_QP_FDTask_Pos");
}

EXPORT_SINGLE_STATE("RL_QP_FDTask_Pos", RL_QP_FDTask_Pos)
