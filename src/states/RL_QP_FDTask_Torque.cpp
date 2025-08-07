#include "RL_QP_FDTask_Torque.h"

void RL_QP_FDTask_Torque::configure(const mc_rtc::Configuration & config)
{
}

void RL_QP_FDTask_Torque::start(mc_control::fsm::Controller & ctl)
{
  RL_utils::start_rl_state(ctl, "RL_QP_FDTask_Torque");
  auto & ctl_rl = static_cast<RLController&>(ctl);
  
  ctl_rl.datastore().get<std::string>("ControlMode") = "Torque";
  ctl_rl.useQP = true;
  ctl_rl.taskType = 1;

  mc_rtc::log::info("using RL with QP and Torque control");
}

bool RL_QP_FDTask_Torque::run(mc_control::fsm::Controller & ctl)
{ 
  return RL_utils::run_rl_state(ctl, "RL_QP_FDTask_Torque");
  // output("OK");
  // return false;
}

void RL_QP_FDTask_Torque::teardown(mc_control::fsm::Controller & ctl)
{
  RL_utils::teardown_rl_state(ctl, "RL_QP_FDTask_Torque");
}

EXPORT_SINGLE_STATE("RL_QP_FDTask_Torque", RL_QP_FDTask_Torque)
