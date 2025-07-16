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
  // refAccel_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase);
  refPos = Eigen::VectorXd::Zero(dofNumber);
  tau_d = Eigen::VectorXd::Zero(dofNumber);
  // tau_d_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase);
  kp_vector = Eigen::VectorXd::Zero(dofNumber);
  kd_vector = Eigen::VectorXd::Zero(dofNumber);
  currentPos = Eigen::VectorXd::Zero(dofNumber);
  currentVel = Eigen::VectorXd::Zero(dofNumber);
  // currentPos_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase);
  // currentVel_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase);

  ddot_qp = Eigen::VectorXd::Zero(dofNumber); // Desired acceleration in the QP solver
  ddot_qp_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase); // Desired acceleration in the QP solver with floating base
  q_cmd = Eigen::VectorXd::Zero(dofNumber); // The commended position send to the internal PD of the robot
  q_cmd_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase); // The commended position send to the internal PD of the robot with floating base
  tau_cmd_after_pd = Eigen::VectorXd::Zero(dofNumber); // The commended position after PD control
  


  similiTorqueTask = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(), 0.0, 1000.0);
  similiTorqueTask->weight(1000.0);
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
  // logger().addLogEntry("RLController_refAccel_w_floatingBase", [this]()
  // { return refAccel_w_floatingBase; });
  logger().addLogEntry("RLController_refPos", [this]() { return refPos; });
  logger().addLogEntry("RLController_tau_d", [this]() { return tau_d; });
  // logger().addLogEntry("RLController_tau_d_w_floatingBase", [this]()
  // { return tau_d_w_floatingBase; });
  logger().addLogEntry("RLController_kp", [this]() { return kp_vector; });
  logger().addLogEntry("RLController_kd", [this]() { return kd_vector; });
  logger().addLogEntry("RLController_currentPos", [this]() { return currentPos; });
  // logger().addLogEntry("RLController_currentPos_w_floatingBase", [this]()
  // { return currentPos_w_floatingBase; });
  logger().addLogEntry("RLController_currentVel", [this]() { return currentVel; });
  // logger().addLogEntry("RLController_currentVel_w_floatingBase", [this]()
  // { return currentVel_w_floatingBase; });
  logger().addLogEntry("RLController_q_cmd", [this]() { return q_cmd; });
  logger().addLogEntry("RLController_q_cmd_w_floatingBase", [this]()
  { return q_cmd_w_floatingBase; });
  logger().addLogEntry("RLController_ddot_qp", [this]() { return ddot_qp; });
  logger().addLogEntry("RLController_ddot_qp_w_floatingBase", [this]()
  { return ddot_qp_w_floatingBase; });

  logger().addLogEntry("RLController_tau_cmd_after_pd_positionCtl", [this]() { return tau_cmd_after_pd; });

  // datastore().make<std::string>("ControlMode", "Torque");
  datastore().make<std::string>("ControlMode", "Position");
  mc_rtc::log::success("[RLController] RLController init");
}

bool RLController::run()
{
  // Update the solver depending on the control mode
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if (ctrl_mode.compare("Position") == 0) {
    auto run = mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoop);
    // auto run = mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoop);
    robot().forwardKinematics();
    robot().forwardVelocity();
    robot().forwardAcceleration();


    rbd::paramToVector(robot().mbc().alphaD, ddot_qp_w_floatingBase);
    ddot_qp = ddot_qp_w_floatingBase.tail(dofNumber); // Exclude the floating base part
    // rbd::paramToVector(robot().mbc().alphaD, ddot_qp_w_floatingBase);

    // Use robot instead of realrobot because we are after the QP
    rbd::ForwardDynamics fd(robot().mb());
    fd.computeH(robot().mb(), robot().mbc());
    fd.computeC(robot().mb(), robot().mbc());
    Eigen::MatrixXd M_w_floatingBase = fd.H();
    Eigen::VectorXd Cg_w_floatingBase = fd.C();
    Eigen::MatrixXd M = M_w_floatingBase.bottomRightCorner(dofNumber, dofNumber);
    Eigen::VectorXd Cg = Cg_w_floatingBase.tail(dofNumber);

    Eigen::MatrixXd Kp_inv = kp_vector.cwiseInverse().asDiagonal();

    q_cmd = currentPos + Kp_inv*(M*ddot_qp + Cg + kd_vector.cwiseProduct(currentVel)); // Inverse PD control to get the commanded position <=> RL position control

    tau_cmd_after_pd = kd_vector.cwiseProduct(currentPos - q_cmd) - kd_vector.cwiseProduct(currentVel); // PD control to get the commanded position after PD control
    
    rbd::paramToVector(robot().mbc().q, q_cmd_w_floatingBase);
    q_cmd_w_floatingBase.tail(dofNumber) = q_cmd; // Copy joint

    // Update the reference position in the robot's mbc
    rbd::vectorToParam(q_cmd_w_floatingBase, realRobot().mbc().q);
    rbd::vectorToParam(q_cmd_w_floatingBase, robot().mbc().q);
    // rbd::vectorToParam(q_cmd, realRobot().mbc().q);
    // rbd::vectorToParam(q_cmd, robot().mbc().q);


    return run;

  } 
  return mc_control::fsm::Controller::run(
      mc_solver::FeedbackType::ClosedLoopIntegrateReal);

}

void RLController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}