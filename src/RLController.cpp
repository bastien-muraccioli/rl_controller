#include "RLController.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <mc_rtc/logging.h>
#include <mc_rbdyn/configuration_io.h>

RLController::RLController(mc_rbdyn::RobotModulePtr rm, double dt, 
                           const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  mc_rtc::log::success("RLController init");
}

bool RLController::run()
{
  return mc_control::fsm::Controller::run();
}

void RLController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  
  initializeAllJoints();
  initializeImpedanceGains();
  pastAction_ = Eigen::VectorXd::Zero(19);
  
  std::string policyPath = config()("policy_path", std::string(""));
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
  
  torqueTask_ = std::make_shared<mc_tasks::TorqueTask>(solver(), 0, 1000.0);
  solver().addTask(torqueTask_);
  
  mc_rtc::log::success("RLController reset completed");
}

void RLController::initializeAllJoints()
{
  allJoints_ = {
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
  
  legJoints_ = std::vector<std::string>(allJoints_.begin(), allJoints_.begin() + 10);
  
  maniskillToMcRtcIdx_ = {0, 3, 7, 11, 15, 1, 4, 8, 12, 16, 2, 5, 9, 13, 17, 6, 10, 14, 18};
  mcRtcToManiskillIdx_ = {0, 5, 10, 1, 6, 11, 15, 2, 7, 12, 16, 3, 8, 13, 17, 4, 9, 14, 18};
}

void RLController::initializeImpedanceGains()
{
  kp_ = Eigen::VectorXd(19);
  kd_ = Eigen::VectorXd(19);
  
  kp_.segment(0, 5) << 100.0, 100.0, 100.0, 100.0, 20.0;  // Left leg
  kd_.segment(0, 5) << 10.0, 10.0, 10.0, 10.0, 4.0;
  
  kp_.segment(5, 5) << 100.0, 100.0, 100.0, 100.0, 20.0;  // Right leg  
  kd_.segment(5, 5) << 10.0, 10.0, 10.0, 10.0, 4.0;
  
  kp_(10) = 100.0; kd_(10) = 10.0;
  
  kp_.segment(11, 4) << 100.0, 100.0, 100.0, 100.0;  // Left arm
  kd_.segment(11, 4) << 5.0, 5.0, 5.0, 5.0;
  
  kp_.segment(15, 4) << 100.0, 100.0, 100.0, 100.0;  // Right arm
  kd_.segment(15, 4) << 5.0, 5.0, 5.0, 5.0;
  
  mc_rtc::log::info("Initialized impedance gains for 19 joints: Kp range [{}, {}], Kd range [{}, {}]", 
                    kp_.minCoeff(), kp_.maxCoeff(), kd_.minCoeff(), kd_.maxCoeff());
}

Eigen::VectorXd RLController::reorderObservationToManiskill(const Eigen::VectorXd & obs)
{  
  if(obs.size() != 19) {
    mc_rtc::log::error("Observation reordering expects 19 joints, got {}", obs.size());
    return obs;
  }
  
  Eigen::VectorXd reordered(19);
  for(int i = 0; i < 19; i++) {
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

Eigen::VectorXd RLController::reorderActionFromManiskill(const Eigen::VectorXd & action)
{  
  if(action.size() != 19) {
    mc_rtc::log::error("Action reordering expects 19 joints, got {}", action.size());
    return action;
  }
  
  Eigen::VectorXd reordered(19);
  for(int i = 0; i < 19; i++) {
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
  
  const auto & robot = this->robot();
  
  Eigen::Vector3d baseAngVel = robot.bodyVelW()[0].angular();
  obs.segment(0, 3) = baseAngVel;
  
  Eigen::Matrix3d baseRot = robot.posW().rotation().transpose();
  Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(baseRot);
  obs(3) = rpy(0);  
  obs(4) = rpy(1);  
  
  Eigen::VectorXd allPos(19), allVel(19);
  for(size_t i = 0; i < allJoints_.size(); ++i)
  {
    if(robot.hasJoint(allJoints_[i]))
    {
      auto jIndex = robot.jointIndexByName(allJoints_[i]);

      allPos(i) = robot.mbc().q[jIndex][0];
      allVel(i) = robot.mbc().alpha[jIndex][0];
    }
    else
    {
      mc_rtc::log::warning("Joint {} not found in robot model", allJoints_[i]);
      allPos(i) = 0.0;
      allVel(i) = 0.0;
    }
  }
  
  Eigen::VectorXd reorderedPos = reorderObservationToManiskill(allPos);
  Eigen::VectorXd reorderedVel = reorderObservationToManiskill(allVel);
  
  std::vector<int> legIndicesInManiskill = {0, 1, 3, 4, 7, 8, 11, 12, 15, 16};
  Eigen::VectorXd legPos(10), legVel(10);
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
  
  obs.segment(25, 10) = pastAction_.head(10);
  
  return obs;
}

void RLController::applyAction(const Eigen::VectorXd & action)
{
  if(action.size() != 19)
  {
    mc_rtc::log::error("Action size mismatch: expected 19, got {}", action.size());
    return;
  }
  
  Eigen::VectorXd reorderedAction = reorderActionFromManiskill(action);
  
  pastAction_ = reorderedAction;
  
  Eigen::VectorXd currentPos(19);
  Eigen::VectorXd currentVel(19);
  
  const auto & robot = this->robot();
  
  for(size_t i = 0; i < allJoints_.size(); ++i)
  {
    if(robot.hasJoint(allJoints_[i]))
    {
      auto jIndex = robot.jointIndexByName(allJoints_[i]);
      
      currentPos(i) = robot.mbc().q[jIndex][0];
      currentVel(i) = robot.mbc().alpha[jIndex][0];
    }
    else
    {
      currentPos(i) = 0.0;
      currentVel(i) = 0.0;
    }
  }
  
  Eigen::VectorXd desiredTorques = computeImpedanceTorques(reorderedAction, currentPos, currentVel);
  
  std::map<std::string, std::vector<double>> torqueMap;
  for(size_t i = 0; i < allJoints_.size(); ++i)
  {
    if(i >= allJoints_.size()) {
      mc_rtc::log::error("Trying to access allJoints_[{}] but size is {}", i, allJoints_.size());
      break;
    }
    mc_rtc::log::info("Adding torque for joint {} ({}): {}", i, allJoints_[i], desiredTorques(i));
    torqueMap[allJoints_[i]] = {desiredTorques(i)};
  }
  mc_rtc::log::info("========================================================================");
  torqueTask_->target(torqueMap);
}

Eigen::VectorXd RLController::computeImpedanceTorques(const Eigen::VectorXd & desiredPos, 
                                                     const Eigen::VectorXd & currentPos,
                                                     const Eigen::VectorXd & currentVel)
{
  Eigen::VectorXd positionError = desiredPos - currentPos;
  Eigen::VectorXd velocityError = -currentVel;  // Assuming velocity is 0
  
  Eigen::VectorXd torques = kp_.cwiseProduct(positionError) + kd_.cwiseProduct(velocityError);
  
  return torques;
} 
