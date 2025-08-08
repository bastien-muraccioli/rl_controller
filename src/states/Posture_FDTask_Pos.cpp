#include "Posture_FDTask_Pos.h"
#include <mc_rtc/logging.h>

void Posture_FDTask_Pos::configure(const mc_rtc::Configuration & config)
{
}

void Posture_FDTask_Pos::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController&>(ctl_);
  mc_rtc::log::info("Posture_FDTask_Pos state started");

  ctl.datastore().get<std::string>("ControlMode") = "Position";
  ctl.useQP = true;
  ctl.taskType = 1;

  ctl.FDTask->stiffness(0.0);
  ctl.FDTask->damping(0.0);
  ctl.TasksSimulation(ctl.q_zero_vector, true);
  ctl.FDTask->refAccel(ctl.refAccel);
  ctl.solver().addTask(ctl.FDTask);

  mc_rtc::log::success("Posture_FDTask_Pos state initialization completed");

  mc_rtc::log::info("Following default Posture with Position control");
}

bool Posture_FDTask_Pos::run(mc_control::fsm::Controller & ctl_)
{ 
  auto & ctl = static_cast<RLController&>(ctl_);
  // ctl.robot().forwardKinematics();
  // ctl.robot().forwardVelocity();
  // ctl.robot().forwardAcceleration();

  // auto q = ctl.robot().encoderValues();
  // ctl.currentPos = Eigen::VectorXd::Map(q.data(), q.size());
  // auto vel = ctl.robot().encoderVelocities();
  // ctl.currentVel = Eigen::VectorXd::Map(vel.data(), vel.size());

  // Eigen::MatrixXd Kp_inv = ctl.kp_vector.cwiseInverse().asDiagonal();

  // ctl.q_cmd = ctl.currentPos + Kp_inv*(ctl.high_kp_vector*(ctl.q_zero_vector - ctl.currentPos) - ctl.currentVel.cwiseProduct(ctl.high_kd_vector - ctl.kd_vector)); //
  ctl.TasksSimulation(ctl.q_zero_vector, true);
  ctl.FDTask->refAccel(ctl.refAccel);
  return false;
}

void Posture_FDTask_Pos::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController&>(ctl_);
  ctl.solver().removeTask(ctl.FDTask);
}

EXPORT_SINGLE_STATE("Posture_FDTask_Pos", Posture_FDTask_Pos)
