#include "RLController.h"
#include <Eigen/src/Core/VectorBlock.h>
#include <RBDyn/MultiBodyConfig.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <mc_rtc/logging.h>
#include <mc_rbdyn/configuration_io.h>
#include <chrono>
#include <cmath>
#include <numeric>


RLController::RLController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  logTiming_ = config("log_timing");
  timingLogInterval_ = config("timing_log_interval");
  isWalkingPolicy = config("is_walking_policy", false);

  //Initialize Constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.2, 400.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.2, 400.0}, 0.9, true));
    // new mc_solver::DynamicsConstraint(robots(), 0, timeStep, {0.1, 0.01, 0.5}, 0.9, true, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Initialize Tasks
  FDTask = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(), 0.0, 1000.0);
  FDTask->stiffness(0.0);
  FDTask->damping(0.0);
  FDTask->refAccel(refAccel);

  torqueTask = std::make_shared<mc_tasks::TorqueTask>(solver(), robot().robotIndex());

  initializeRobot(config);
  initializeRLPolicy(config);
  
  if(useAsyncInference_)
  {
    auto & ctl = *this;
    utils_.startInferenceThread(ctl);
  }

  addGui();
  addLog();
  mc_rtc::log::success("RLController init");
}

bool RLController::run()
{
  leftAnklePos = robot().mbc().bodyPosW[robot().bodyIndexByName("left_ankle_link")].translation();
  rightAnklePos = robot().mbc().bodyPosW[robot().bodyIndexByName("right_ankle_link")].translation();
  ankleDistanceNorm = (leftAnklePos - rightAnklePos).norm();

  bool run = mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  robot().forwardKinematics();
  robot().forwardVelocity();
  robot().forwardAcceleration();
  if(!useQP) // Run RL without taking account of the QP
  {
    q_cmd = q_rl; // Use the RL position as the commanded position
    tau_cmd = kp_vector.cwiseProduct(q_rl - currentPos) - kd_vector.cwiseProduct(currentVel);
    updateRobotCmdAfterQP();
    return true;
  }
  // Use QP
  computeInversePD();
  updateRobotCmdAfterQP();
  return run; // Return false if QP fails
}

void RLController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  mc_rtc::log::success("RLController reset completed");
  // utils_.stopInferenceThread();
}

void RLController::tasksComputation(Eigen::VectorXd & currentTargetPosition)
{
  auto & robot = robots()[0];
  auto & real_robot = realRobot(robots()[0].name());

  auto q = robot.encoderValues();
  currentPos = Eigen::VectorXd::Map(q.data(), q.size());
  auto vel = robot.encoderVelocities();
  currentVel = Eigen::VectorXd::Map(vel.data(), vel.size());

  if(controlledByRL) tau_d = kp_vector.cwiseProduct(currentTargetPosition - currentPos) - kd_vector.cwiseProduct(currentVel);
  else tau_d = high_kp_vector.cwiseProduct(currentTargetPosition - currentPos) - high_kd_vector.cwiseProduct(currentVel);

  tau_rl = kp_vector.cwiseProduct(q_rl - currentPos) - kd_vector.cwiseProduct(currentVel);
  
  switch (taskType)
  {
    case TORQUE_TASK: // Torque Task
    {
      size_t i = 0;
      for (const auto &joint_name : jointNames)
      {
        torque_target[joint_name][0] = tau_d[i];
        i++;
      }
      break;
    }
    case FD_TASK: // Forward Dynamics Task
    {
      rbd::ForwardDynamics fd(real_robot.mb());
      fd.computeH(real_robot.mb(), real_robot.mbc());
      fd.computeC(real_robot.mb(), real_robot.mbc());
      Eigen::MatrixXd M_w_floatingBase = fd.H();
      Eigen::VectorXd Cg_w_floatingBase = fd.C();
      
      auto extTorqueSensor = robot.device<mc_rbdyn::VirtualTorqueSensor>("ExtTorquesVirtSensor");
      Eigen::VectorXd tau_d_w_floating_base = Eigen::VectorXd::Zero(robot.mb().nrDof());
      tau_d_w_floating_base.tail(dofNumber) = tau_d.tail(dofNumber);
      Eigen::VectorXd content = tau_d_w_floating_base - Cg_w_floatingBase; // Add the external torques to the desired torques
      if(!compensateExternalForces) content += extTorqueSensor.torques();
      
      Eigen::VectorXd refAccel_w_floating_base = M_w_floatingBase.llt().solve(content);
      refAccel = refAccel_w_floating_base.tail(dofNumber); // Exclude the floating base part
      break;
    }
    default:
      mc_rtc::log::error("Invalid task type: {}", taskType);
      return;
  }
}

void RLController::updateRobotCmdAfterQP()
{
  auto q = robot().mbc().q;
  auto alpha = robot().mbc().alpha;
  auto tau = robot().mbc().jointTorque;
  
  size_t i = 0;
  for (const auto &joint_name : jointNames)
  {
    q[robot().jointIndexByName(joint_name)][0] = q_cmd[i];
    alpha[robot().jointIndexByName(joint_name)][0] = 0.0;
    tau[robot().jointIndexByName(joint_name)][0] = tau_cmd[i];
    i++;
  }
  // Update q and qdot for position control
  robot().mbc().q = q;
  if(controlledByRL) robot().mbc().alpha = alpha; // For RL policy qdot ref = 0
  // Update joint torques for torque control
  robot().mbc().jointTorque = tau;

  // Both are always updated despite they are not used by the robot
  // They are still used by the QP
}

void RLController::computeInversePD()
{
  // Using QP (TorqueTask or ForwardDynamics Task):  
  rbd::paramToVector(robot().mbc().alphaD, ddot_qp_w_floatingBase);
  ddot_qp = ddot_qp_w_floatingBase.tail(dofNumber); // Exclude the floating base part

  // Use robot instead of realrobot because we are after the QP
  rbd::ForwardDynamics fd(robot().mb());
  fd.computeH(robot().mb(), robot().mbc());
  fd.computeC(robot().mb(), robot().mbc());
  Eigen::MatrixXd M_w_floatingBase = fd.H();
  Eigen::VectorXd Cg_w_floatingBase = fd.C();

  auto extTorqueSensor = robot().device<mc_rbdyn::VirtualTorqueSensor>("ExtTorquesVirtSensor");
  Eigen::VectorXd tau_cmd_w_floatingBase = M_w_floatingBase*ddot_qp_w_floatingBase + Cg_w_floatingBase - extTorqueSensor.torques();
  tau_cmd = tau_cmd_w_floatingBase.tail(dofNumber);
  
  Eigen::MatrixXd Kp_inv = current_kp.cwiseInverse().asDiagonal();

  q_cmd = currentPos + Kp_inv*(tau_cmd + current_kd.cwiseProduct(currentVel)); // Inverse PD control to get the commanded position <=> RL position control
}

void RLController::addLog()
{
  // Robot State variables
  logger().addLogEntry("RLController_refAccel", [this]() { return refAccel; });
  logger().addLogEntry("RLController_tau_d", [this]() { return tau_d; });
  logger().addLogEntry("RLController_kp", [this]() { return current_kp; });
  logger().addLogEntry("RLController_kd", [this]() { return current_kd; });
  logger().addLogEntry("RLController_currentPos", [this]() { return currentPos; });
  logger().addLogEntry("RLController_currentVel", [this]() { return currentVel; });
  logger().addLogEntry("RLController_q_cmd", [this]() { return q_cmd; });
  logger().addLogEntry("RLController_ddot_qp", [this]() { return ddot_qp; });
  logger().addLogEntry("RLController_ddot_qp_w_floatingBase", [this]()
  { return ddot_qp_w_floatingBase; });
  logger().addLogEntry("RLController_tau_cmd", [this]() { return tau_cmd; });

  // RL variables
  logger().addLogEntry("RLController_q_rl", [this]() { return q_rl; });
  logger().addLogEntry("RLController_tau_rl", [this]() { return tau_rl; });
  logger().addLogEntry("RLController_pastAction", [this]() { return a_simuOrder; });
  logger().addLogEntry("RLController_qZero", [this]() { return q_zero_vector; });
  logger().addLogEntry("RLController_a_before", [this]() { return a_before_vector; });
  logger().addLogEntry("RLController_currentObservation", [this]() { return currentObservation_; });
  logger().addLogEntry("RLController_a_vector", [this]() { return a_vector; });
  logger().addLogEntry("RLController_a_simulationOrder", [this]() { return a_simuOrder; });
  logger().addLogEntry("RLController_currentAction", [this]() { return currentAction_; });
  logger().addLogEntry("RLController_latestAction", [this]() { return latestAction_; });
  logger().addLogEntry("RLController_baseAngVel", [this]() { return baseAngVel; });
  logger().addLogEntry("RLController_rpy", [this]() { return rpy; });
  logger().addLogEntry("RLController_legPos", [this]() { return legPos; });
  logger().addLogEntry("RLController_legVel", [this]() { return legVel; });
  logger().addLogEntry("RLController_legAction", [this]() { return legAction; });
  logger().addLogEntry("RLController_phase", [this]() { return phase_; });
  
  // Controller state variables
  logger().addLogEntry("RLController_useQP", [this]() { return useQP; });
  logger().addLogEntry("RLController_controlledByRL", [this]() { return controlledByRL; });
  logger().addLogEntry("RLController_taskType", [this]() { return taskType; });

  // RL Controller
  logger().addLogEntry("RLController_q_lim_upper", [this]() { return jointLimitsPos_upper; });
  logger().addLogEntry("RLController_q_lim_lower", [this]() { return jointLimitsPos_lower; });
  logger().addLogEntry("RLController_q_dot_lim_upper", [this]() { return jointLimitsVel_upper; });
  logger().addLogEntry("RLController_q_dot_lim_lower", [this]() { return jointLimitsVel_lower; });
  logger().addLogEntry("RLController_ankleDistanceNorm", [this]() { return ankleDistanceNorm; });
}

void RLController::addGui()
{
  gui()->addElement({"FSM", "Options"},
  mc_rtc::gui::Checkbox("Compensate External Forces", compensateExternalForces));
}

void RLController::initializeRobot(const mc_rtc::Configuration & config)
{
  // H1 joints in mc_rtc/URDF order (based on unitree_sdk2 reorder_obs function)
  mcRtcJointsOrder = {
    "left_hip_yaw_joint",      
    "left_hip_roll_joint",       
    "left_hip_pitch_joint",    
    "left_knee_joint",         
    "left_ankle_joint",        
    "right_hip_yaw_joint",     
    "right_hip_roll_joint",    
    "right_hip_pitch_joint",   
    "right_knee_joint",        
    "right_ankle_joint",       
    "torso_joint",             
    "left_shoulder_pitch_joint",  
    "left_shoulder_roll_joint",     
    "left_shoulder_yaw_joint",    
    "left_elbow_joint",           
    "right_shoulder_pitch_joint", 
    "right_shoulder_roll_joint",  
    "right_shoulder_yaw_joint",   
    "right_elbow_joint"           
  };

  notControlledJoints = {
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "torso_joint"
  };

  dofNumber = robot().mb().nrDof() - 6; // Remove the floating base part (6 DoF)

  leftAnklePos = robot().mbc().bodyPosW[robot().bodyIndexByName("left_ankle_link")].translation();
  rightAnklePos = robot().mbc().bodyPosW[robot().bodyIndexByName("right_ankle_link")].translation();
  ankleDistanceNorm = (leftAnklePos - rightAnklePos).norm();

  Eigen::VectorXd qlimUpper = rbd::paramToVector(robot().mb(), robot().qu()).tail(dofNumber);
  Eigen::VectorXd qlimLower = rbd::paramToVector(robot().mb(), robot().ql()).tail(dofNumber);
  Eigen::VectorXd vlimUpper = rbd::paramToVector(robot().mb(), robot().vu()).tail(dofNumber);
  Eigen::VectorXd vlimLower = rbd::paramToVector(robot().mb(), robot().vl()).tail(dofNumber);
  Eigen::VectorXd tlimUpper = rbd::paramToVector(robot().mb(), robot().tu()).tail(dofNumber);
  Eigen::VectorXd tlimLower = rbd::paramToVector(robot().mb(), robot().tl()).tail(dofNumber);

  
  jointLimitsPos_upper = qlimUpper - (qlimUpper - qlimLower)*0.01;
  jointLimitsPos_lower = qlimLower + (qlimUpper - qlimLower)*0.01;

  jointLimitsVel_upper = vlimUpper*0.9;
  jointLimitsVel_lower = vlimLower*0.9;

  mc_rtc::log::info("[RLController] Joint limits pos upper: {}", jointLimitsPos_upper.transpose());
  mc_rtc::log::info("[RLController] Joint limits pos lower: {}", jointLimitsPos_lower.transpose());
  mc_rtc::log::info("[RLController] Joint limits vel upper: {}", jointLimitsVel_upper.transpose());
  mc_rtc::log::info("[RLController] Joint limits vel lower: {}", jointLimitsVel_lower.transpose());
  refAccel = Eigen::VectorXd::Zero(dofNumber); // TVM
  q_rl = Eigen::VectorXd::Zero(dofNumber);
  tau_rl = Eigen::VectorXd::Zero(dofNumber);
  q_zero_vector = Eigen::VectorXd::Zero(dofNumber);
  tau_d = Eigen::VectorXd::Zero(dofNumber);
  kp_vector = Eigen::VectorXd::Zero(dofNumber);
  kd_vector = Eigen::VectorXd::Zero(dofNumber);
  high_kp_vector = Eigen::VectorXd::Zero(dofNumber);
  high_kd_vector = Eigen::VectorXd::Zero(dofNumber);
  currentPos = Eigen::VectorXd::Zero(dofNumber);
  currentVel = Eigen::VectorXd::Zero(dofNumber);

  ddot_qp = Eigen::VectorXd::Zero(dofNumber); // Desired acceleration in the QP solver
  ddot_qp_w_floatingBase = Eigen::VectorXd::Zero(robot().mb().nrDof()); // Desired acceleration in the QP solver with floating base
  
  tau_cmd = Eigen::VectorXd::Zero(dofNumber); // Final torque that control the robot
  q_cmd = Eigen::VectorXd::Zero(dofNumber); // The commended position send to the internal PD of the robot
  
  // Get the gains from the configuration or set default values
  std::map<std::string, double> kp = config("kp");
  std::map<std::string, double> kd = config("kd");

  std::map<std::string, double> high_kp = config("high_kp");
  std::map<std::string, double> high_kd = config("high_kd");

  // Get the default posture target from the robot's posture task
  std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask = getPostureTask(robot().name());
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
            high_kp_vector[i] = high_kp.at(joint_name);
            high_kd_vector[i] = high_kd.at(joint_name);
            q_rl[i] = t[0];
            q_zero_vector[i] = t[0];
            torque_target[joint_name] = {0.0};
            mc_rtc::log::info("[RLController] Joint {}: currentTargetPosition {}, kp {}, kd {}", joint_name, q_rl[i], kp_vector[i], kd_vector[i]);
            i++;
        }
      }
  }
  current_kp = high_kp_vector;
  current_kd = high_kd_vector;
  solver().removeTask(FSMPostureTask);
  datastore().make<std::string>("ControlMode", "Torque");
}

void RLController::initializeRLPolicy(const mc_rtc::Configuration & config)
{
  auto & real_robot = realRobot(robots()[0].name());

  baseAngVel = real_robot.bodyVelW("pelvis").angular();
  Eigen::Matrix3d baseRot = real_robot.bodyPosW("pelvis").rotation();
  rpy = mc_rbdyn::rpyFromMat(baseRot);
    
  mc_rtc::log::info("[RLController] Posture target initialized with {} joints", dofNumber); 

  // Initialize reference position and last actions for action blending
  a_before_vector = Eigen::VectorXd::Zero(dofNumber);
  a_vector = Eigen::VectorXd::Zero(dofNumber);
  legPos = Eigen::VectorXd::Zero(10);
  legVel = Eigen::VectorXd::Zero(10);
  legAction = Eigen::VectorXd::Zero(10);

  a_simuOrder = Eigen::VectorXd::Zero(dofNumber);

  mc_rtc::log::info("Reference position initialized with {} joints", q_zero_vector.size());
  q_rl = q_zero_vector;  // Start with reference position
  
  useAsyncInference_ = config("use_async_inference", true);
  mc_rtc::log::info("Async RL inference: {}", useAsyncInference_ ? "enabled" : "disabled");
  currentAction_ = Eigen::VectorXd::Zero(dofNumber);
  latestAction_ = Eigen::VectorXd::Zero(dofNumber);
  
  // Initialize new observation components
  velCmdRL_ = Eigen::Vector3d::Zero();  // Default command (x, y, yaw)
  // velCmdRL_(1) = -20;
  phase_ = 0.0;  // Phase for periodic gait
  startPhase_ = std::chrono::steady_clock::now();  // For phase calculation
  
  std::string policyPath = config("policy_path", std::string(""));
  if(policyPath.empty())
    policyPath = "policy.onnx"; // Default policy path if not specified in config

  mc_rtc::log::info("Loading RL policy from: {}", policyPath);
  try {
    rlPolicy_ = std::make_unique<RLPolicyInterface>(policyPath);
    if(rlPolicy_) {
      mc_rtc::log::success("RL policy loaded successfully");
      // Initialize observation vector with the correct size from the loaded policy
      currentObservation_ = Eigen::VectorXd::Zero(rlPolicy_->getObservationSize());
      mc_rtc::log::info("Initialized observation vector with size: {}", rlPolicy_->getObservationSize());
    } else {
      mc_rtc::log::error_and_throw("RL policy creation failed - policy is null");
    }
  } catch(const std::exception& e) {
    mc_rtc::log::error_and_throw("Failed to load RL policy: {}", e.what());
  }

  std::string simulator = config("Simulator", std::string(""));
  // check if simulator is Maniskill
  if(simulator == "Maniskill")
  {
    mc_rtc::log::info("Using Maniskill handling");
    policySimulatorHandling_ = std::make_unique<PolicySimulatorHandling>("Maniskill");
  }
  else {
    mc_rtc::log::warning("Simulator not recognized or not set, using default handling");
    policySimulatorHandling_ = std::make_unique<PolicySimulatorHandling>();
  }

  // get list of used joints from config
  std::vector<int> usedJoints = config("Used_joints_index", std::vector<int>{});
  if(!usedJoints.empty())
  {
    std::string jointsStr = "[";
    for(size_t i = 0; i < usedJoints.size(); ++i) {
      if(i > 0) jointsStr += ", ";
      jointsStr += std::to_string(usedJoints[i]);
    }
    jointsStr += "]";
    mc_rtc::log::info("Using custom used joints: {}", jointsStr);
    usedJoints_simuOrder = policySimulatorHandling_->getSimulatorIndices(usedJoints);
    std::sort(usedJoints_simuOrder.begin(), usedJoints_simuOrder.end());
    jointsStr = "[";
    for(size_t i = 0; i < usedJoints_simuOrder.size(); ++i) {
      if(i > 0) jointsStr += ", ";
      jointsStr += std::to_string(usedJoints_simuOrder[i]);
    }
    jointsStr += "]";
    mc_rtc::log::info("Using custom used joints: {}", jointsStr);

  }
  else {
    mc_rtc::log::info("No custom used joints specified, using default all joints");
    usedJoints_simuOrder = std::vector<int>(dofNumber);
    std::iota(usedJoints_simuOrder.begin(), usedJoints_simuOrder.end(), 0);
  }
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd> RLController::getPDGains()
{
  std::string robot_name = robot().name();
  std::vector<double> proportionalGains_vec(kp_vector.data(), kp_vector.data() + kp_vector.size());
  std::vector<double> dampingGains_vec(kd_vector.data(), kd_vector.data() + kd_vector.size());
  datastore().call<bool>(robot_name + "::GetPDGains", proportionalGains_vec, dampingGains_vec);
  Eigen::VectorXd p_vec = Eigen::VectorXd::Map(proportionalGains_vec.data(), proportionalGains_vec.size());
  Eigen::VectorXd d_vec = Eigen::VectorXd::Map(dampingGains_vec.data(), dampingGains_vec.size());
  mc_rtc::log::info("[RLController] Current PD Gains for {} are:\n\tkp = {}\n\tkd = {}", robot_name, p_vec.transpose(), d_vec.transpose());
  return std::make_tuple(p_vec, d_vec);
}

bool RLController::setPDGains(Eigen::VectorXd p_vec, Eigen::VectorXd d_vec)
{
  std::string robot_name = robot().name();
  // Update kp and kd use by the controller
  current_kp = p_vec;
  current_kd = d_vec;

  // Update kp and kd use by the robot or simulator (Internal PD)
  mc_rtc::log::info("[RLController] Setting PD gains for {}:\n\tkp = {}\n\tkd = {}", robot_name, p_vec.transpose(), d_vec.transpose());
  const std::vector<double> proportionalGains_vec(p_vec.data(), p_vec.data() + p_vec.size());
  const std::vector<double> dampingGains_vec(d_vec.data(), d_vec.data() + d_vec.size());
  return datastore().call<bool>(robot_name + "::SetPDGains", proportionalGains_vec, dampingGains_vec);
}

bool RLController::isHighGain(double tol)
{
  // Update kp and kd use by the controller
  std::tie(current_kp, current_kd) = getPDGains();
  // Check if the current gains are close to the high gains
  bool highGain = ((current_kp - high_kp_vector).norm() < tol) && ((current_kd - high_kd_vector).norm() < tol);
  mc_rtc::log::info("[RLController] isHighGain: {}", highGain);
  return highGain;
}

void RLController::initializeState(bool torque_control, int task_type, bool controlled_by_rl)
{
  if(torque_control) datastore().get<std::string>("ControlMode") = "Torque";
  else datastore().get<std::string>("ControlMode") = "Position";

  if (!datastore().call<bool>("EF_Estimator::isActive")) {
    datastore().call("EF_Estimator::toggleActive");
  }

  useQP = true;
  if(task_type == PURE_RL) useQP = false;
  else taskType = task_type;
  controlledByRL = controlled_by_rl;
  if(controlledByRL)
  {
    // Set low gains for RL
    if(isHighGain()) setPDGains(kp_vector, kd_vector);
    tasksComputation(q_rl);
  }
  else
  {
    // Set high gains for model-based control
    if(!isHighGain()) setPDGains(high_kp_vector, high_kd_vector);
    tasksComputation(q_zero_vector);
  }
}
