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

  // std::vector<double> kp, kd;
  // try {
  //     bool success = ctl.datastore().call<bool>("h1::GetPDGains", kp, kd);
  //     if(success) {
  //         // Process gains...
  //         const std::vector<double> const_kp = {1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 200.0, 200.0, 100.0, 100.0, 200.0, 200.0, 100.0, 100.0, 200.0};
  //         const std::vector<double> const_kd = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 6.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  //         bool set_success = ctl.datastore().call<bool>("h1::SetPDGains", const_kp, const_kd);
  //         if(set_success) {
  //             mc_rtc::log::info("Successfully retrieved and set PD gains: kp size={}, kd size={}", kp.size(), kd.size());
  //             for (auto el : const_kp)
  //             {
  //               mc_rtc::log::info(el);
  //             }
  //         } else {
  //             mc_rtc::log::warning("Failed to set PD gains to MuJoCo");
  //         }
  //     } else {
  //         mc_rtc::log::warning("Failed to retrieve PD gains from MuJoCo");
  //     }
  // } catch(const std::exception& e) {
  //     mc_rtc::log::warning("Failed to access PD gains: {}", e.what());
  // }

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
