#include "RLController.h"
#include <Eigen/src/Core/Matrix.h>
#include <RBDyn/MultiBodyConfig.h>
#include <cmath>
#include <cstddef>
#include <mc_rtc/logging.h>

RLController::RLController(mc_rbdyn::RobotModulePtr rm, double dt, 
                           const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{

  // // Add constraints
  // contactConstraintTest =std::make_unique<mc_solver::ContactConstraint>(timeStep, mc_solver::ContactConstraint::ContactType::Acceleration);
  // solver().addConstraintSet(contactConstraintTest);

  // selfCollisionConstraint->setCollisionsDampers(solver(), {1.8, 70.0});
  
  // dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
  //     new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.9, true));

  // solver().addConstraintSet(dynamicsConstraint);
  dofNumber_with_floatingBase = robot().mb().nrDof();
  dofNumber = robot().mb().nrDof() - 6; // Remove the floating base part (6 DoF)
  // refAccel = Eigen::VectorXd::Zero(dofNumber_with_floatingBase); // Task
  refAccel = Eigen::VectorXd::Zero(dofNumber); // TVM
  refAccel_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase);
  refPos = Eigen::VectorXd::Zero(dofNumber);
  tau_d = Eigen::VectorXd::Zero(dofNumber);
  tau_d_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase);
  kp_vector = Eigen::VectorXd::Zero(dofNumber);
  kd_vector = Eigen::VectorXd::Zero(dofNumber);
  currentPos = Eigen::VectorXd::Zero(dofNumber);
  currentVel = Eigen::VectorXd::Zero(dofNumber);
  currentPos_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase);
  currentVel_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase);


  similiTorqueTask = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(), 0.0, 1000.0);
  similiTorqueTask->weight(100000.0);
  similiTorqueTask->stiffness(0.0);
  similiTorqueTask->damping(0.0);
  similiTorqueTask->refAccel(refAccel);
  // solver().addTask(similiTorqueTask);

  // Get the gains from the configuration or set default values
  kp = config("kp");
  kd = config("kd");

  // Get the default posture target from the robot's posture task
  FSMPostureTask = getPostureTask(robot().name());
  auto posture = FSMPostureTask->posture();
  size_t i = 0;
  std::vector<std::string> joint_names;
  joint_names.reserve(robot().mb().joints().size());
  for (const auto &j : robot().mb().joints()) {
      const std::string &joint_name = j.name();
      if(j.type() == rbd::Joint::Type::Rev)
      {
        jointNames.emplace_back(joint_name);  
        if (const auto &t = posture[robot().jointIndexByName(joint_name)]; !t.empty()) {
            kp_vector[i] = kp.at(joint_name);
            kd_vector[i] = kd.at(joint_name);
            refPos[i] = t[0];
            mc_rtc::log::info("[RLController] Joint {}: refPos {}, kp {}, kd {}", joint_name, refPos[i], kp_vector[i], kd_vector[i]);
            i++;
        }
      }
  }
  

  

  mc_rtc::log::info("[RLController] Posture target initialized with {} joints", dofNumber);

  // add to logger
  logger().addLogEntry("RLController_refAccel", [this]() { return refAccel; });
  logger().addLogEntry("RLController_refAccel_w_floatingBase", [this]()
  { return refAccel_w_floatingBase; });
  logger().addLogEntry("RLController_refPos", [this]() { return refPos; });
  logger().addLogEntry("RLController_tau_d", [this]() { return tau_d; });
  logger().addLogEntry("RLController_tau_d_w_floatingBase", [this]()
  { return tau_d_w_floatingBase; });
  logger().addLogEntry("RLController_kp", [this]() { return kp_vector; });
  logger().addLogEntry("RLController_kd", [this]() { return kd_vector; });
  logger().addLogEntry("RLController_currentPos", [this]() { return currentPos; });
  logger().addLogEntry("RLController_currentPos_w_floatingBase", [this]()
  { return currentPos_w_floatingBase; });
  logger().addLogEntry("RLController_currentVel", [this]() { return currentVel; });
  logger().addLogEntry("RLController_currentVel_w_floatingBase", [this]()
  { return currentVel_w_floatingBase; });

  datastore().make<std::string>("ControlMode", "Torque");
  // datastore().make<std::string>("ControlMode", "Position");
  mc_rtc::log::success("[RLController] RLController init");
}

bool RLController::run()
{
  // Update the solver depending on the control mode
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if (ctrl_mode.compare("Position") == 0) {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  } else {
    return mc_control::fsm::Controller::run(
        mc_solver::FeedbackType::ClosedLoopIntegrateReal);
      // return mc_control::fsm::Controller::run(
      //   mc_solver::FeedbackType::ClosedLoop);

  }
  // return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
}

void RLController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}