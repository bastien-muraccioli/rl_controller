#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/PostureTask.h>
#include <RBDyn/Coriolis.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <vector>
#include "api.h"

// Fixed vector sizes
constexpr int INPUT_SIZE = 35;
constexpr int OUTPUT_SIZE = 19;

struct RLController_DLLAPI RLController : public mc_control::fsm::Controller
{
  RLController(mc_rbdyn::RobotModulePtr rm, double dt, 
               const mc_rtc::Configuration & config);
 
  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
  
  std::unique_ptr<mc_solver::ContactConstraint> contactConstraintTest;
  std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask;
  std::shared_ptr<mc_tasks::PostureTask> similiTorqueTask;

  Eigen::VectorXd refAccel;
  // Eigen::VectorXd refAccel_w_floatingBase;
  Eigen::VectorXd refPos;
  Eigen::VectorXd tau_d;
  // Eigen::VectorXd tau_d_w_floatingBase;
  Eigen::VectorXd currentPos;
  // Eigen::VectorXd currentPos_w_floatingBase;
  Eigen::VectorXd currentVel;
  // Eigen::VectorXd currentVel_w_floatingBase;

  std::vector<std::vector<double>> desiredPosture;
  std::vector<std::string> jointNames;
  std::map<std::string, std::vector<double>> postureTarget;

  std::map<std::string, double> kp;
  std::map<std::string, double> kd;
  Eigen::VectorXd kp_vector;
  Eigen::VectorXd kd_vector;

  size_t dofNumber_with_floatingBase = 0; // Number of degrees of freedom in the robot
  size_t dofNumber = 0; // Number of degrees of freedom in the robot without floating base

  // For position control
  Eigen::VectorXd ddot_qp; // Desired acceleration in the QP solver
  Eigen::VectorXd ddot_qp_w_floatingBase; // Desired acceleration in the QP solver with floating base
  Eigen::VectorXd q_cmd; // The commended position send to the internal PD of the robot
  Eigen::VectorXd q_cmd_w_floatingBase; // The commended position send to the internal PD of the robot with floating base
  Eigen::VectorXd tau_cmd_after_pd; // The commended position after PD control

  Eigen::Vector3d baseAngVel; // Base angular velocity
  Eigen::Vector3d baseAngVelB; // Base angular velocity
  Eigen::Vector3d rpy; // Roll, pitch, yaw angles of the base

  Eigen::VectorXd parseVector(const std::string& str, size_t size);

  Eigen::VectorXd input_vec;
  Eigen::VectorXd output_vec;

  std::vector<Eigen::VectorXd> input_vec_array;
  std::vector<Eigen::VectorXd> output_vec_array;

  int line_number = 0;

private:
}; 