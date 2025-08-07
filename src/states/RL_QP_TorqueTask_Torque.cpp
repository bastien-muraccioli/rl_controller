#include "RL_QP_TorqueTask_Torque.h"

void RL_QP_TorqueTask_Torque::configure(const mc_rtc::Configuration & config)
{
}

void RL_QP_TorqueTask_Torque::start(mc_control::fsm::Controller & ctl)
{
  utils::start_rl_state(ctl, "RL_QP_TorqueTask_Torque");
  auto & ctl_rl = static_cast<RLController&>(ctl);
  
  ctl_rl.datastore().get<std::string>("ControlMode") = "Torque";
  ctl_rl.useQP = true;
  ctl_rl.taskType = 0;

  ctl_rl.TasksSimulation(ctl_rl.q_zero_vector);
  ctl_rl.torqueTask->target(ctl_rl.torque_target);
  ctl_rl.solver().addTask(ctl_rl.torqueTask);

  mc_rtc::log::info("using RL with QP and Torque control");
}

bool RL_QP_TorqueTask_Torque::run(mc_control::fsm::Controller & ctl)
{
  auto & ctl_rl = static_cast<RLController&>(ctl);
  utils::run_rl_state(ctl, "RL_QP_TorqueTask_Torque");
  ctl_rl.torqueTask->target(ctl_rl.torque_target);
  return false;
}

void RL_QP_TorqueTask_Torque::teardown(mc_control::fsm::Controller & ctl)
{
  auto & ctl_rl = static_cast<RLController &>(ctl);
  ctl_rl.solver().removeTask(ctl_rl.torqueTask);
  utils::teardown_rl_state(ctl, "RL_QP_TorqueTask_Torque");
}

EXPORT_SINGLE_STATE("RL_QP_TorqueTask_Torque", RL_QP_TorqueTask_Torque)
