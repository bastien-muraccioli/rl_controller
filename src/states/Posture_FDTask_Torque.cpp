#include "Posture_FDTask_Torque.h"
#include <mc_rtc/logging.h>

void Posture_FDTask_Torque::configure(const mc_rtc::Configuration & config)
{
}

void Posture_FDTask_Torque::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController&>(ctl_);
  mc_rtc::log::info("Posture_FDTask_Torque state started");

  ctl.datastore().get<std::string>("ControlMode") = "Torque";
  ctl.useQP = true;
  ctl.taskType = 1;

  ctl.FDTask->stiffness(0.0);
  ctl.FDTask->damping(0.0);
  ctl.TasksSimulation(ctl.q_zero_vector, true);
  ctl.FDTask->refAccel(ctl.refAccel);
  ctl.solver().addTask(ctl.FDTask);

  mc_rtc::log::success("Posture_Torque state initialization completed");

  mc_rtc::log::info("Following default Posture with Torque control");
}

bool Posture_FDTask_Torque::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController&>(ctl_);

  ctl.TasksSimulation(ctl.q_zero_vector, true);
  ctl.FDTask->refAccel(ctl.refAccel);

  return false;
}

void Posture_FDTask_Torque::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController&>(ctl_);
  ctl.solver().removeTask(ctl.FDTask);
}

EXPORT_SINGLE_STATE("Posture_FDTask_Torque", Posture_FDTask_Torque)
