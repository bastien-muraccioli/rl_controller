#include "PureRL_TorqueTask_Torque.h"

void PureRL_TorqueTask_Torque::configure(const mc_rtc::Configuration & config)
{
}

void PureRL_TorqueTask_Torque::start(mc_control::fsm::Controller & ctl)
{
  RL_utils::start_rl_state(ctl, "PureRL_TorqueTask_Torque");
  auto & ctl_rl = static_cast<RLController&>(ctl);
  
  ctl_rl.datastore().get<std::string>("ControlMode") = "Torque";
  ctl_rl.useQP = false;
  ctl_rl.taskType = 0;

  mc_rtc::log::info("using Pure RL (no QP) and Torque control");
}

bool PureRL_TorqueTask_Torque::run(mc_control::fsm::Controller & ctl)
{
  return RL_utils::run_rl_state(ctl, "PureRL_TorqueTask_Torque");
  // output("OK");
  // return false;
}

void PureRL_TorqueTask_Torque::teardown(mc_control::fsm::Controller & ctl)
{
  RL_utils::teardown_rl_state(ctl, "PureRL_TorqueTask_Torque");
}

EXPORT_SINGLE_STATE("PureRL_TorqueTask_Torque", PureRL_TorqueTask_Torque)
