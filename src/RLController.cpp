#include "RLController.h"
#include <Eigen/src/Core/Matrix.h>
#include <RBDyn/MultiBodyConfig.h>
#include <cstddef>
#include <mc_rtc/logging.h>

RLController::RLController(mc_rbdyn::RobotModulePtr rm, double dt, 
                           const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{

  // Add constraints
  contactConstraintTest =std::make_unique<mc_solver::ContactConstraint>(timeStep, mc_solver::ContactConstraint::ContactType::Acceleration);
  solver().addConstraintSet(contactConstraintTest);

  selfCollisionConstraint->setCollisionsDampers(solver(), {1.8, 70.0});
  
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(
          robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Get the gains from the configuration or set default values
  kp = config("kp");
  kd = config("kd");

  // Get the default posture target from the robot's posture task
  FSMPostureTask = getPostureTask(robot().name());
  auto posture = FSMPostureTask->posture();
  std::vector<std::string> joint_names;
  joint_names.reserve(robot().mb().joints().size());
  for (const auto &j : robot().mb().joints()) {
      const std::string &joint_name = j.name();
      if(j.type() == rbd::Joint::Type::Rev)
      {
        joint_names.emplace_back(joint_name);  
        if (const auto &t = posture[robot().jointIndexByName(joint_name)]; !t.empty()) {
            postureTarget[joint_name] = {static_cast<double>(t[0])};  
            jointNames.push_back(joint_name);
            // torqueTarget[joint_name] = std::vector<double>(t.size(), 0.0); // Initialize torque target with zeros
            dofNumber+=1;
            mc_rtc::log::info("[RLController] Posture target for joint {}: {} with kp {} and kd {}", joint_name, static_cast<double>(t[0]), kp[joint_name], kd[joint_name]);
        }
      }
  }

  

  mc_rtc::log::info("[RLController] Posture target initialized with {} joints", dofNumber);

  // Keep the posture task for the evaluation of the torque task
  // FSMPostureTask->weight(0.0); // Disable the posture task for now
  // solver().removeTask(FSMPostureTask);

  torqueTask = std::make_shared<mc_tasks::TorqueTask>(solver(), robot().robotIndex(), 1000.0);
  // torqueTask->weight(1000.0);
  // solver().addTask(torqueTask);


  // datastore().make<std::string>("ControlMode", "Torque");
  datastore().make<std::string>("ControlMode", "Position");
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
  }
  // return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
}

std::map<std::string, std::vector<double>> RLController::convertPosToTorque(std::map<std::string, std::vector<double>> & posTarget)
{
  std::map<std::string, std::vector<double>> tauTarget;
  Eigen::VectorXd currentPos_w_floatingBase(dofNumber+7), currentVel_w_floatingBase(dofNumber+7);//, currentTorque_w_floatingBase(dofNumber+7);

  rbd::paramToVector(realRobot().mbc().q, currentPos_w_floatingBase);
  rbd::paramToVector(realRobot().mbc().alpha, currentVel_w_floatingBase);

  Eigen::VectorXd currentTorque = Eigen::VectorXd::Map(realRobot().jointTorques().data(), realRobot().jointTorques().size());
  mc_rtc::log::info("[RLController] Current Torque: {}", currentTorque);

  std::map<std::string, double> currentPosMap;
  std::map<std::string, double> currentVelMap;
  std::map<std::string, double> currentTorqueMap;
  size_t i = 7; // Extract the first 6 elements for floating base + 1 Root joint
  for (const auto &joint_name : jointNames)
  {
    currentPosMap[joint_name] = currentPos_w_floatingBase[i];
    currentVelMap[joint_name] = currentVel_w_floatingBase[i];
    currentTorqueMap[joint_name] = currentTorque[i-7];
    mc_rtc::log::info("[RLController] Joint {}: current pos {} rad; desired pos {} rad", joint_name, currentPosMap[joint_name], posTarget[joint_name][0]);
    mc_rtc::log::info("[RLController] Joint {}: current vel {} rad/s; desired vel {} rad/s", joint_name, currentVelMap[joint_name], 0.0);
    mc_rtc::log::info("[RLController] Joint {}: current torque {} Nm", joint_name, currentTorqueMap[joint_name]);
    i++;
  }

  // Based on posTarget, create tauTarget for each joint with impedance control
  for (const auto &joint_name : jointNames)
  {
    // mc_rtc::log::info("[RLController] {} τ = kp * (q_desired: {} - q_current {}) + kd * (- dq_current {}) + tau_current {}", joint_name, posTarget[joint_name][0], currentPosMap[joint_name], currentVelMap[joint_name], currentTorqueMap[joint_name]);
    // Impedance control: τ = kp * (q_desired - q_current) + kd * (0 - dq_current)
    // double torque = (kp * (posTarget[joint_name][0] - currentPosMap[joint_name]) + kd * (-currentVelMap[joint_name]))*timeStep + currentTorqueMap[joint_name];
    double torque = (kp[joint_name] * (posTarget.at(joint_name)[0] - currentPosMap[joint_name]) + kd[joint_name] * (-currentVelMap[joint_name]));
    mc_rtc::log::info("[RLController] {} τ = {} Nm", joint_name, torque);
    tauTarget[joint_name] = {torque};
    // tauTarget[joint_name] = {currentTorqueMap[joint_name]};
  }
  return tauTarget;
}

void RLController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}