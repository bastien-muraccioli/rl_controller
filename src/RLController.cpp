#include "RLController.h"
#include <Eigen/src/Core/VectorBlock.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <mc_rtc/logging.h>
#include <mc_rbdyn/configuration_io.h>
#include <chrono>

RLController::RLController(mc_rbdyn::RobotModulePtr rm, double dt, 
                           const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  // Add constraints
  // contactConstraintTest =std::make_unique<mc_solver::ContactConstraint>(timeStep, mc_solver::ContactConstraint::ContactType::Acceleration);
  // solver().addConstraintSet(contactConstraintTest);

  // selfCollisionConstraint->setCollisionsDampers(solver(), {1.8, 70.0});`
  logTiming_ = config("log_timing");
  timingLogInterval_ = config("timing_log_interval");

  selfCollisionConstraint->setCollisionsDampers(solver(), {1.2, 400.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    // new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.2, 400.0}, 0.9, true));
    new mc_solver::DynamicsConstraint(robots(), 0, timeStep, {0.1, 0.01, 0.5}, 0.9, false, true));
  solver().addConstraintSet(dynamicsConstraint);

  
  dofNumber_with_floatingBase = robot().mb().nrDof();
  dofNumber = robot().mb().nrDof() - 6; // Remove the floating base part (6 DoF)
  // refAccel = Eigen::VectorXd::Zero(dofNumber_with_floatingBase); // Task
  refAccel = Eigen::VectorXd::Zero(dofNumber); // TVM
  // refAccel_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase);
  q_rl_vector = Eigen::VectorXd::Zero(dofNumber);
  q_zero_vector = Eigen::VectorXd::Zero(dofNumber);
  tau_d = Eigen::VectorXd::Zero(dofNumber);
  // tau_d_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase);
  kp_vector = Eigen::VectorXd::Zero(dofNumber);
  kd_vector = Eigen::VectorXd::Zero(dofNumber);
  high_kp_vector = Eigen::VectorXd::Zero(dofNumber);
  high_kd_vector = Eigen::VectorXd::Zero(dofNumber);
  currentPos = Eigen::VectorXd::Zero(dofNumber);
  currentVel = Eigen::VectorXd::Zero(dofNumber);
  // currentPos_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase);
  // currentVel_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase);

  ddot_qp = Eigen::VectorXd::Zero(dofNumber); // Desired acceleration in the QP solver
  ddot_qp_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase); // Desired acceleration in the QP solver with floating base
  q_cmd = Eigen::VectorXd::Zero(dofNumber); // The commended position send to the internal PD of the robot
  // q_cmd_w_floatingBase = Eigen::VectorXd::Zero(dofNumber_with_floatingBase); // The commended position send to the internal PD of the robot with floating base
  tau_cmd_after_pd = Eigen::VectorXd::Zero(dofNumber); // The commended position after PD control
  
  FDTask = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(), 0.0, 1000.0);
  FDTask->weight(1000.0);
  FDTask->stiffness(0.0);
  FDTask->damping(0.0);
  FDTask->refAccel(refAccel);

  torqueTask = std::make_shared<mc_tasks::TorqueTask>(solver(), robot().robotIndex());

  // Get the gains from the configuration or set default values
  kp = config("kp");
  kd = config("kd");

  high_kp = config("high_kp");
  high_kd = config("high_kd");

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
            high_kp_vector[i] = high_kp.at(joint_name);
            high_kd_vector[i] = high_kd.at(joint_name);
            q_rl_vector[i] = t[0];
            q_zero_vector[i] = t[0];
            torque_target[joint_name] = {0.0};
            mc_rtc::log::info("[RLController] Joint {}: currentTargetPosition {}, kp {}, kd {}", joint_name, q_rl_vector[i], kp_vector[i], kd_vector[i]);
            i++;
        }
      }
  }
  solver().removeTask(FSMPostureTask);

  auto & real_robot = realRobot(robots()[0].name());
  
  //  Eigen::Vector3d baseAngVel = real_robot.bodyVelW()[0].angular();

  baseAngVel = real_robot.bodyVelW("pelvis").angular();
  Eigen::Matrix3d baseRot = real_robot.bodyPosW("pelvis").rotation();
  rpy = mc_rbdyn::rpyFromMat(baseRot);
    
  mc_rtc::log::info("[RLController] Posture target initialized with {} joints", dofNumber);

  datastore().make<std::string>("ControlMode", "Torque");
  // datastore().make<std::string>("ControlMode", "Position");
  initializeAllJoints();
  // initializeImpedanceGains();
  a_maniskillOrder = Eigen::VectorXd::Zero(dofNumber);
  
  // Initialize reference position and last actions for action blending
  a_before_vector = Eigen::VectorXd::Zero(dofNumber);
  a_vector = Eigen::VectorXd::Zero(dofNumber);
  legPos = Eigen::VectorXd::Zero(10);
  legVel = Eigen::VectorXd::Zero(10);
  legAction = Eigen::VectorXd::Zero(10);
  
  mc_rtc::log::info("Reference position initialized with {} joints", q_zero_vector.size());
  // mc_rtc::log::info("Reference positions: {}", q_ref_.transpose().format(Eigen::IOFormat(3, 0, ", ", "", "", "", "[", "]")));
  
  // Initialize 40Hz inference timing
  lastInferenceTime_ = std::chrono::steady_clock::now();
  q_rl_vector = q_zero_vector;  // Start with reference position
  targetPositionValid_ = true;
  
  useAsyncInference_ = config("use_async_inference", true);
  mc_rtc::log::info("Async RL inference: {}", useAsyncInference_ ? "enabled" : "disabled");
  
  shouldStopInference_ = false;
  newObservationAvailable_ = false;
  newActionAvailable_ = false;
  currentObservation_ = Eigen::VectorXd::Zero(35);
  currentAction_ = Eigen::VectorXd::Zero(dofNumber);
  latestAction_ = Eigen::VectorXd::Zero(dofNumber);
  
  std::string policyPath = config("policy_path", std::string(""));
  if(!policyPath.empty())
  {
    mc_rtc::log::info("Loading RL policy from: {}", policyPath);
    rlPolicy_ = std::make_unique<RLPolicyInterface>(policyPath);
  }
  else
  {
    mc_rtc::log::warning("No policy_path specified, creating dummy policy");
    rlPolicy_ = std::make_unique<RLPolicyInterface>();
  }
  
  // torqueTask_ = std::make_shared<mc_tasks::TorqueTask>(solver(), 0, 1000.0);
  // solver().addTask(torqueTask_);
  
  if(useAsyncInference_)
  {
    startInferenceThread();
  }

  logging();

  mc_rtc::log::success("RLController init");
}

void RLController::logging()
{
  logger().addLogEntry("RLController_refAccel", [this]() { return refAccel; });
  // logger().addLogEntry("RLController_refAccel_w_floatingBase", [this]()
  // { return refAccel_w_floatingBase; });
  logger().addLogEntry("RLController_currentTargetPosition", [this]() { return q_rl_vector; });
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
  // logger().addLogEntry("RLController_q_cmd_w_floatingBase", [this]()
  // { return q_cmd_w_floatingBase; });
  logger().addLogEntry("RLController_ddot_qp", [this]() { return ddot_qp; });
  logger().addLogEntry("RLController_ddot_qp_w_floatingBase", [this]()
  { return ddot_qp_w_floatingBase; });

  logger().addLogEntry("RLController_tau_cmd_after_pd_positionCtl", [this]() { return tau_cmd_after_pd; });

  // pastAction_
  logger().addLogEntry("RLController_pastAction", [this]() { return a_maniskillOrder; });
  // q_ref_
  logger().addLogEntry("RLController_qZero", [this]() { return q_zero_vector; });
  // lastActions_
  logger().addLogEntry("RLController_a_before", [this]() { return a_before_vector; });
  // currentObservation_
  logger().addLogEntry("RLController_currentObservation", [this]() { return currentObservation_; });
  // a_vector
  logger().addLogEntry("RLController_a_vector", [this]() { return a_vector; });
  // a_maniskillOrder
  logger().addLogEntry("RLController_a_maniskillOrder", [this]() { return a_maniskillOrder; });
  // currentAction_
  logger().addLogEntry("RLController_currentAction", [this]() { return currentAction_; });
  // latestAction_
  logger().addLogEntry("RLController_latestAction", [this]() { return latestAction_; });
  // baseAngVel
  logger().addLogEntry("RLController_baseAngVel", [this]() { return baseAngVel; });
  // rpy
  logger().addLogEntry("RLController_rpy", [this]() { return rpy; });
  // legPos
  logger().addLogEntry("RLController_legPos", [this]() { return legPos; });
  // legVel
  logger().addLogEntry("RLController_legVel", [this]() { return legVel; });
  // legAction
  logger().addLogEntry("RLController_legAction", [this]() { return legAction; });

  gui()->addElement({"FSM", "Options"},
  mc_rtc::gui::Checkbox("External torques", compensateExternalForces));
}

RLController::~RLController()
{
  stopInferenceThread();
  mc_rtc::log::info("RLController destroyed");
}

bool RLController::run()
{
  if (compensateExternalForcesHasChanged != compensateExternalForces)
  {
    mc_rtc::log::info("Compensate external forces: {}", compensateExternalForces);
    datastore().call("EF_Estimator::toggleActive");
    compensateExternalForcesHasChanged = compensateExternalForces;
  }
  bool run = mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  robot().forwardKinematics();
  robot().forwardVelocity();
  robot().forwardAcceleration();

  auto q = robot().encoderValues();
  currentPos = Eigen::VectorXd::Map(q.data(), q.size());
  auto vel = robot().encoderVelocities();
  currentVel = Eigen::VectorXd::Map(vel.data(), vel.size());

  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if (ctrl_mode.compare("Position") == 0)
    return positionControl(run);
  // ctrl_mode.compare("Torque") == 0 :
  return torqueControl(run);
}

bool RLController::positionControl(bool run) //TODO:only keep basic formula
{
  rbd::paramToVector(robot().mbc().alphaD, ddot_qp_w_floatingBase);
  ddot_qp = ddot_qp_w_floatingBase.tail(dofNumber); // Exclude the floating base part

  // Use robot instead of realrobot because we are after the QP
  rbd::ForwardDynamics fd(robot().mb());
  fd.computeH(robot().mb(), robot().mbc());
  fd.computeC(robot().mb(), robot().mbc());
  Eigen::MatrixXd M_w_floatingBase = fd.H();
  Eigen::VectorXd Cg_w_floatingBase = fd.C();
  Eigen::MatrixXd M = M_w_floatingBase.bottomRightCorner(dofNumber, dofNumber);
  Eigen::VectorXd Cg = Cg_w_floatingBase.tail(dofNumber);

  Eigen::MatrixXd Kp_inv = kp_vector.cwiseInverse().asDiagonal();

  // if (!static_pos)
    q_cmd = currentPos + Kp_inv*(M*ddot_qp + Cg + kd_vector.cwiseProduct(currentVel));
  // else
  // {
    // q_cmd = currentPos + Kp_inv*(high_kp_vector*(q_zero_vector - currentPos) - currentVel.cwiseProduct(high_kd_vector - kd_vector)); //
  // }
  auto q = robot().mbc().q;

  if (useQP)
  {
    size_t i = 0;
    for (const auto &joint_name : jointNames)
    {
      q[robot().jointIndexByName(joint_name)][0] = q_cmd[i];
      i++;
    }

    robot().mbc().q = q; // Update the mbc with the new position
    return run;
  }
  else
  {
    if (!static_pos)//rl
      q_cmd = q_rl_vector;
    size_t i = 0;
    for (const auto &joint_name : jointNames)
    {
      q[robot().jointIndexByName(joint_name)][0] = q_cmd[i];
      i++;
    }

    robot().mbc().q = q; // Update the mbc with the new position
    return true;
  }
}

bool RLController::torqueControl(bool run) //TODO:only keep rl -> rest in state
{
  if (useQP == false)
  {
    auto tau = robot().mbc().jointTorque;
    tau_d = kp_vector.cwiseProduct(q_rl_vector - currentPos) - kd_vector.cwiseProduct(currentVel);
    
    size_t i = 0;
    for (const auto &joint_name : jointNames)
    {
      tau[robot().jointIndexByName(joint_name)][0] = tau_d[i];
      i++;
    }

    robot().mbc().jointTorque = tau; // Update the mbc with the new position
    return true;
  }
  else { 
    return run;
  }
}

void RLController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  mc_rtc::log::success("RLController reset completed");
}

void RLController::initializeAllJoints()
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

  maniskillJointsOrder = {
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "torso_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_ankle_joint",
    "right_ankle_joint",
    "left_elbow_joint",
    "right_elbow_joint"
  };

  maniskillLegsJointsOrder = {
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_ankle_joint",
    "right_ankle_joint"
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
  
  // leg joints (first 10)
  legJoints_ = std::vector<std::string>(mcRtcJointsOrder.begin(), mcRtcJointsOrder.begin() + 10);
  
  legIndicesInManiskill = {0, 1, 3, 4, 7, 8, 11, 12, 15, 16};
  maniskillToMcRtcIdx_ = {0, 3, 7, 11, 15, 1, 4, 8, 12, 16, 2, 5, 9, 13, 17, 6, 10, 14, 18};
  mcRtcToManiskillIdx_ = {0, 5, 10, 1, 6, 11, 15, 2, 7, 12, 16, 3, 8, 13, 17, 4, 9, 14, 18};
}

Eigen::VectorXd RLController::reorderJointsToManiskill(const Eigen::VectorXd & obs)
{  
  if(obs.size() != dofNumber) {
    mc_rtc::log::error("Observation reordering expects dofNumber joints, got {}", obs.size());
    return obs;
  }
  
  Eigen::VectorXd reordered(dofNumber);
  for(int i = 0; i < dofNumber; i++) {
    if(i >= mcRtcToManiskillIdx_.size()) {
      mc_rtc::log::error("Trying to access mcRtcToManiskillIdx_[{}] but size is {}", i, mcRtcToManiskillIdx_.size());
      reordered[i] = 0.0;
      continue;
    }
    
    int srcIdx = mcRtcToManiskillIdx_[i];
    if(srcIdx >= obs.size()) {
      mc_rtc::log::error("Index {} out of bounds for obs size {}", srcIdx, obs.size());
      reordered[i] = 0.0;
    } else {
      reordered[i] = obs(srcIdx);
    }
  }
  return reordered;
}

Eigen::VectorXd RLController::reorderJointsFromManiskill(const Eigen::VectorXd & action)
{  
  if(action.size() != dofNumber) {
    mc_rtc::log::error("Action reordering expects dofNumber joints, got {}", action.size());
    return action;
  }
  
  Eigen::VectorXd reordered(dofNumber);
  for(int i = 0; i < dofNumber; i++) {
    if(i >= maniskillToMcRtcIdx_.size()) {
      mc_rtc::log::error("Trying to access maniskillToMcRtcIdx_[{}] but size is {}", i, maniskillToMcRtcIdx_.size());
      reordered[i] = 0.0;
      continue;
    }
    
    int srcIdx = maniskillToMcRtcIdx_[i];
    if(srcIdx >= action.size()) {
      mc_rtc::log::error("Action reorder index {} out of bounds for action size {}", srcIdx, action.size());
      reordered[i] = 0.0;
    } else {
      reordered[i] = action(srcIdx);
    }
  }
  return reordered;
}

Eigen::VectorXd RLController::getCurrentObservation()
{
  // Observation: [base angular velocity (3), roll (1), pitch (1), joint pos (10), joint vel (10), past action (10)]
  
  Eigen::VectorXd obs(35);
  obs = Eigen::VectorXd::Zero(35);
  
  // const auto & robot = this->robot();

  auto & robot = robots()[0];
  auto & real_robot = realRobot(robots()[0].name());
  
  //  Eigen::Vector3d baseAngVel = real_robot.bodyVelW()[0].angular();

  // baseAngVel = real_robot.bodyVelW("pelvis").angular();
  baseAngVel = real_robot.bodyVelW("pelvis").angular();
  obs.segment(0, 3) = baseAngVel; //base angular vel
  
  Eigen::Matrix3d baseRot = real_robot.bodyPosW("pelvis").rotation();
  // Eigen::Matrix3d baseRot = real_robot.bodyTransform("pelvis").rotation();
  rpy = mc_rbdyn::rpyFromMat(baseRot);
  obs(3) = rpy(0);  // roll
  obs(4) = rpy(1);  // pitch
  
  Eigen::VectorXd reorderedPos = reorderJointsToManiskill(currentPos);
  Eigen::VectorXd reorderedVel = reorderJointsToManiskill(currentVel);
  
  for(size_t i = 0; i < legIndicesInManiskill.size(); ++i)
  {
    int idx = legIndicesInManiskill[i];
    if(idx >= reorderedPos.size()) {
      mc_rtc::log::error("Leg joint index {} out of bounds for reordered size {}", idx, reorderedPos.size());
      legPos(i) = 0.0;
      legVel(i) = 0.0;
    } else {
      legPos(i) = reorderedPos(idx);
      legVel(i) = reorderedVel(idx);
    }
  }
  
  obs.segment(5, 10) = legPos;
  obs.segment(15, 10) = legVel;
  
  // past action: reorder to ManiSkill format and extract leg joints
  for(size_t i = 0; i < legIndicesInManiskill.size(); ++i)
  {
    int idx = legIndicesInManiskill[i];
    if(idx >= a_maniskillOrder.size()) {
      mc_rtc::log::error("Past action index {} out of bounds for size {}", idx, a_maniskillOrder.size());
      legAction(i) = 0.0;
    } else {
      legAction(i) = a_maniskillOrder(idx);
    }
  }
  obs.segment(25, 10) = legAction;
  return obs;
}

void RLController::applyAction(const Eigen::VectorXd & action)
{
  if(action.size() != dofNumber)
  {
    mc_rtc::log::error("Action size mismatch: expected dofNumber, got {}", action.size());
    return;
  }
  
  // Check if it's time for new inference (40Hz = 25ms period)
  auto currentTime = std::chrono::steady_clock::now();
  auto timeSinceLastInference = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastInferenceTime_);
  
  bool shouldRunInference = timeSinceLastInference.count() >= INFERENCE_PERIOD_MS;
  
  if(shouldRunInference) {
    // Get current observation for logging
    Eigen::VectorXd currentObs = getCurrentObservation();
    
    // Update lastActions_
    a_before_vector = a_vector;
    // Run new inference and update target position
    a_vector = reorderJointsFromManiskill(action);
    
    // Apply action blending formula: target_qpos = default_qpos + 0.75 * action + 0.25 * previous_actions
    q_rl_vector = q_zero_vector + 0.75 * a_vector + 0.25 * a_before_vector;

    // For not controlled joints, use the zero position
    for(const auto & joint : notControlledJoints)
    {
      auto it = std::find(mcRtcJointsOrder.begin(), mcRtcJointsOrder.end(), joint);
      if(it != mcRtcJointsOrder.end())
      {
        size_t idx = std::distance(mcRtcJointsOrder.begin(), it);
        if(idx < q_rl_vector.size())
        {
          q_rl_vector(idx) = q_zero_vector(idx); // Set to zero position
        }
        else
        {
          mc_rtc::log::error("Joint {} index {} out of bounds for q_rl_vector size {}", joint, idx, q_rl_vector.size());
        }
      }
      else
      {
        mc_rtc::log::error("Joint {} not found in mcRtcJointsOrder", joint);
      }
    }

    a_maniskillOrder = reorderJointsToManiskill(a_vector);
    
    // Update timing
    lastInferenceTime_ = currentTime;
    targetPositionValid_ = true;
    
    static int inferenceCounter = 0;
    inferenceCounter++;
    
    mc_rtc::log::info("=== RLController Policy I/O Inference #{} ===", inferenceCounter);
    mc_rtc::log::info("Policy Input (35 obs): [");
    for(int i = 0; i < 35; ++i) {
      mc_rtc::log::info("  [{}]: {:.6f}", i, currentObs(i));
    }
    mc_rtc::log::info("]");
    mc_rtc::log::info("Blended Target Position (dofNumber): [");
    for(int i = 0; i < dofNumber; ++i) {
      mc_rtc::log::info("  [{}]: {:.6f}", i, q_rl_vector(i));
    }
    mc_rtc::log::info("]");
    mc_rtc::log::info("=== End Policy I/O ===");
  }
  
  if(!targetPositionValid_) {
    mc_rtc::log::warning("No valid target position available for impedance control");
    return;
  }
  
  // Get current joint positions and velocities
  Eigen::VectorXd q_current(dofNumber);
  Eigen::VectorXd q_dot_current(dofNumber);
  auto & real_robot = realRobot(robots()[0].name());
  auto q = real_robot.encoderValues();
  q_current = Eigen::VectorXd::Map(q.data(), q.size());
  auto vel = real_robot.encoderVelocities();
  q_dot_current = Eigen::VectorXd::Map(vel.data(), vel.size());

  const auto & robot = this->robot();
  
  for(size_t i = 0; i < mcRtcJointsOrder.size(); ++i)
  {
    if(robot.hasJoint(mcRtcJointsOrder[i]))
    {
      auto jIndex = robot.jointIndexByName(mcRtcJointsOrder[i]);
      q_current(i) = robot.mbc().q[jIndex][0];
      q_dot_current(i) = robot.mbc().alpha[jIndex][0];
    }
    else
    {
      q_current(i) = 0.0;
      q_dot_current(i) = 0.0;
    }
  }
  TasksSimulation(q_rl_vector);
}

void RLController::startInferenceThread()
{
  mc_rtc::log::info("Starting RL inference thread");
  inferenceThread_ = std::make_unique<std::thread>(&RLController::inferenceThreadFunction, this);
}

void RLController::stopInferenceThread()
{
  if(inferenceThread_ && inferenceThread_->joinable())
  {
    mc_rtc::log::info("Stopping RL inference thread");
    shouldStopInference_ = true;
    inferenceCondition_.notify_one();
    inferenceThread_->join();
    inferenceThread_.reset();
  }
}

void RLController::inferenceThreadFunction()
{
  mc_rtc::log::info("RL inference thread started");
  
  while(!shouldStopInference_)
  {
    //wait for new observation or stop signal
    std::unique_lock<std::mutex> lock(observationMutex_);
    inferenceCondition_.wait(lock, [this] { 
      return newObservationAvailable_.load() || shouldStopInference_.load(); 
    });
    
    if(shouldStopInference_) break;
    
    // copy observation for processing
    Eigen::VectorXd obs = currentObservation_;
    newObservationAvailable_ = false;
    lock.unlock();
    
    try
    {
      auto startTime = std::chrono::high_resolution_clock::now();
      Eigen::VectorXd action = rlPolicy_->predict(obs);
      auto endTime = std::chrono::high_resolution_clock::now();
      
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
      
      //update shared action
      {
        std::lock_guard<std::mutex> actionLock(actionMutex_);
        currentAction_ = action;
        newActionAvailable_ = true;
      }
    }
    catch(const std::exception & e)
    {
      mc_rtc::log::error("RL inference error: {}", e.what());
      //keep using the previous action
    }
  }
  
  mc_rtc::log::info("RL inference thread stopped");
}

void RLController::updateObservationForInference()
{
  Eigen::VectorXd obs = getCurrentObservation();
  
  //update shared observation
  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    currentObservation_ = obs;
    newObservationAvailable_ = true;
  }
  
  // notify inference thread
  inferenceCondition_.notify_one();
}

Eigen::VectorXd RLController::getLatestAction()
{
  if(newActionAvailable_)
  {
    std::lock_guard<std::mutex> lock(actionMutex_);
    if(newActionAvailable_)
    {
      latestAction_ = currentAction_;
      newActionAvailable_ = false;
    }
  }
  return latestAction_;
} 

void RLController::TasksSimulation(Eigen::VectorXd & currentTargetPosition, bool highGains)
{
  auto & robot = robots()[0];
  auto & real_robot = realRobot(robots()[0].name());

  auto q = real_robot.encoderValues();
  currentPos = Eigen::VectorXd::Map(q.data(), q.size());
  auto vel = real_robot.encoderVelocities();
  currentVel = Eigen::VectorXd::Map(vel.data(), vel.size());

  if(highGains)
    tau_d = high_kp_vector.cwiseProduct(currentTargetPosition - currentPos) + high_kd_vector.cwiseProduct(-currentVel);
  else
    tau_d = kp_vector.cwiseProduct(currentTargetPosition - currentPos) + kd_vector.cwiseProduct(-currentVel);

  switch (taskType)
  {
    case 0: // Torque Task
    {
      size_t i = 0;
      for (const auto &joint_name : jointNames)
      {
        torque_target[joint_name][0] = tau_d[i];
        i++;
      }
      break;
    }
    case 1: // Forward Dynamics Task
    {
      rbd::ForwardDynamics fd(real_robot.mb());
      fd.computeH(real_robot.mb(), real_robot.mbc());
      fd.computeC(real_robot.mb(), real_robot.mbc());
      Eigen::MatrixXd M_w_floatingBase = fd.H();
      Eigen::VectorXd Cg_w_floatingBase = fd.C();
      Eigen::MatrixXd M = M_w_floatingBase.bottomRightCorner(dofNumber, dofNumber);
      Eigen::VectorXd Cg = Cg_w_floatingBase.tail(dofNumber);
      auto extTorqueSensor = robot.device<mc_rbdyn::VirtualTorqueSensor>("ExtTorquesVirtSensor");
      Eigen::VectorXd externalTorques = extTorqueSensor.torques().tail(dofNumber); // Exclude the floating base part
      
      Eigen::VectorXd content = tau_d - Cg + externalTorques; // Add the external torques to the desired torques
      refAccel = M.llt().solve(content);
      break;
    }
    default:
      mc_rtc::log::error("Invalid task type: {}", taskType);
      return;
  }
}

