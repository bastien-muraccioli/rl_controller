#include "RL_QP_FDTask_Torque.h"

void RL_QP_FDTask_Torque::configure(const mc_rtc::Configuration & config)
{
}

void RL_QP_FDTask_Torque::start(mc_control::fsm::Controller & ctl)
{
  utils::start_rl_state(ctl, "RL_QP_FDTask_Torque");
  auto & ctl_rl = static_cast<RLController&>(ctl);
  
  ctl_rl.datastore().get<std::string>("ControlMode") = "Torque";
  ctl_rl.useQP = true;
  ctl_rl.taskType = 1;

  ctl_rl.FDTask->stiffness(0.0);
  ctl_rl.FDTask->damping(0.0);
  ctl_rl.TasksSimulation(ctl_rl.q_zero_vector);
  ctl_rl.FDTask->refAccel(ctl_rl.refAccel);
  ctl_rl.solver().addTask(ctl_rl.FDTask);

  mc_rtc::log::info("using RL with QP and Torque control");
}

bool RL_QP_FDTask_Torque::run(mc_control::fsm::Controller & ctl)
{ 
  auto & ctl_rl = static_cast<RLController&>(ctl);
  utils::run_rl_state(ctl, "RL_QP_FDTask_Torque");
  ctl_rl.FDTask->refAccel(ctl_rl.refAccel);
  return false;
}

void RL_QP_FDTask_Torque::teardown(mc_control::fsm::Controller & ctl)
{
  auto & ctl_rl = static_cast<RLController &>(ctl);
  ctl_rl.solver().removeTask(ctl_rl.FDTask);
  utils::teardown_rl_state(ctl, "RL_QP_FDTask_Torque");
}

EXPORT_SINGLE_STATE("RL_QP_FDTask_Torque", RL_QP_FDTask_Torque)
