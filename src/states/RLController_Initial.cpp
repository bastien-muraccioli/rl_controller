#include "RLController_Initial.h"
#include "../RLController.h"
#include <mc_rtc/logging.h>

void RLController_Initial::configure(const mc_rtc::Configuration & config)
{
  // No configuration needed for initial state
}

void RLController_Initial::start(mc_control::fsm::Controller & ctl)
{
  auto & ctl_ = static_cast<RLController &>(ctl);
  postureTarget = ctl_.postureTarget;
  mc_rtc::log::info("RLController_Initial state started");
}

bool RLController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController &>(ctl_);
  auto & real_robot = ctl.realRobot(ctl.robots()[0].name());
  ctl.baseAngVel = real_robot.bodyVelW("pelvis").angular();
  ctl.baseAngVelB = real_robot.bodyVelB("pelvis").angular();
  Eigen::Matrix3d baseRot = real_robot.bodyPosW("pelvis").rotation();
  ctl.rpy = mc_rbdyn::rpyFromMat(baseRot);

  if(ctl.robot().encoderVelocities().empty())
  {
    mc_rtc::log::warning("[RLController_Initial] No encoder velocities available");
    return false;
  }
  // output("OK");
  // ctl.torqueTarget = ctl.convertPosToTorque(ctl.postureTarget);
  if(isTorqueTask)
  {
    // auto target = ctl.convertPosToTorque(postureTarget);
    // ctl.torqueTask->target(target);
    torqueTaskSimulation(ctl);
  }
  else
  {
    if(ctl.FSMPostureTask->eval().norm() < 0.01)
    {
      ctl.solver().removeTask(ctl.FSMPostureTask);
      isTorqueTask = true;
      torqueTaskSimulation(ctl);
      ctl.solver().addTask(ctl.similiTorqueTask);
    }
  }

  if(counter >= 0.025)
  {
    counter = 0.0;
    if(line_counter > ctl.line_number) line_counter = 0; // Reset the line counter if it exceeds the number of lines in the file
    ctl.input_vec = ctl.input_vec_array[line_counter];
    ctl.output_vec = ctl.output_vec_array[line_counter];
    line_counter++;
  }

  counter += ctl.timeStep;

  return false;
}

void RLController_Initial::teardown(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("RLController_Initial state ending");
}

void RLController_Initial::torqueTaskSimulation(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController &>(ctl_);
  auto & robot = ctl.robots()[0];
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());

  auto q = realRobot.encoderValues();
  ctl.currentPos = Eigen::VectorXd::Map(q.data(), q.size());
  // ctl.currentPos_w_floatingBase = Eigen::VectorXd::Map(q.data(), q.size());
  // ctl.currentPos = ctl.currentPos_w_floatingBase.head(ctl.dofNumber); // Exclude the floating base part
  auto vel = realRobot.encoderVelocities();
  ctl.currentVel = Eigen::VectorXd::Map(vel.data(), vel.size());
  // ctl.currentVel_w_floatingBase = Eigen::VectorXd::Map(vel.data(), vel.size());
  // ctl.currentVel = ctl.currentVel_w_floatingBase.head(ctl.dofNumber); // Exclude the floating base part

  mc_rtc::log::info("[RLController] Current Position: {}", ctl.currentPos);
  mc_rtc::log::info("[RLController] Current Velocity: {}", ctl.currentVel);

  ctl.tau_d = ctl.kp_vector.cwiseProduct(ctl.refPos - ctl.currentPos) + ctl.kd_vector.cwiseProduct(-ctl.currentVel);
  mc_rtc::log::info("[RLController] tau_d: {}", ctl.tau_d);

  // ctl.tau_d_w_floatingBase = Eigen::VectorXd::Zero(ctl.dofNumber_with_floatingBase); // full vector zeroed
  // ctl.tau_d_w_floatingBase.head(ctl.dofNumber) = ctl.tau_d; // copy only joint torques


  // Simulate the torque task by converting the torque target to an acceleration target
  rbd::ForwardDynamics fd(realRobot.mb());
  fd.computeH(realRobot.mb(), realRobot.mbc());
  fd.computeC(realRobot.mb(), realRobot.mbc());
  Eigen::MatrixXd M_w_floatingBase = fd.H();
  Eigen::VectorXd Cg_w_floatingBase = fd.C();
  Eigen::MatrixXd M = M_w_floatingBase.bottomRightCorner(ctl.dofNumber, ctl.dofNumber);
  Eigen::VectorXd Cg = Cg_w_floatingBase.tail(ctl.dofNumber);
  ctl.refAccel = M.completeOrthogonalDecomposition().solve(ctl.tau_d - Cg);
  // For the TVM backend
  // ctl.refAccel = Eigen::VectorXd::Zero(ctl.dofNumber_with_floatingBase);
  // ctl.refAccel.head(ctl.dofNumber) = ctl.refAccel_w_floatingBase.head(ctl.dofNumber);
  // Keep the full vector for Task
  // ctl.refAccel = ctl.refAccel_w_floatingBase; 


  mc_rtc::log::info("[RLController] refAccel: {}", ctl.refAccel);
  ctl.similiTorqueTask->refAccel(ctl.refAccel);
}

EXPORT_SINGLE_STATE("RLController_Initial", RLController_Initial)
