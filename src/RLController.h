#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorqueTask.h>
#include <mc_tasks/PostureTask.h>

#include "api.h"

#include "RLPolicyInterface.h"
#include "PolicySimulatorHandling.h"
#include "utils.h"

#include <memory>
#include <Eigen/Dense>
#include <mutex>
#include <atomic>

#include <chrono>
#include <vector>

#include <numeric>

#define TORQUE_TASK 0
#define FD_TASK 1
#define PURE_RL 2

struct RLController_DLLAPI RLController : public mc_control::fsm::Controller
{
  RLController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void addLog();
  void addGui();
  void initializeRobot(const mc_rtc::Configuration & config);
  void initializeRLPolicy(const mc_rtc::Configuration & config);
  void initializeState(bool torque_control, int task_type, bool controlled_by_rl);

  bool positionControl(bool run);
  bool torqueControl(bool run);

  void tasksComputation(Eigen::VectorXd & currentTargetPosition);
  std::tuple<Eigen::VectorXd, Eigen::VectorXd> getPDGains();
  bool setPDGains(Eigen::VectorXd p_vec, Eigen::VectorXd d_vec);
  bool isHighGain(double tol = 1e-9);

  bool isWalkingPolicy = false;

  // Tasks
  std::shared_ptr<mc_tasks::PostureTask> FDTask;
  std::shared_ptr<mc_tasks::TorqueTask> torqueTask;

  std::map<std::string, std::vector<double>> torque_target; // Target torques for the torque task;
  int taskType = FD_TASK;
  bool useQP = true;
  bool controlledByRL = true;

  // Robot specific data
  std::vector<std::string> jointNames;
  size_t dofNumber_with_floatingBase = 0;
  size_t dofNumber = 0;

  // Gains
  Eigen::VectorXd kp_vector;
  Eigen::VectorXd kd_vector;
  Eigen::VectorXd high_kp_vector;
  Eigen::VectorXd high_kd_vector;
  Eigen::VectorXd current_kp;
  Eigen::VectorXd current_kd;

  // Options
  bool compensateExternalForces = false;

  // Robot state 
  Eigen::VectorXd refAccel;
  Eigen::VectorXd tau_d;
  Eigen::VectorXd currentPos;
  Eigen::VectorXd currentVel;

  // For position control
  Eigen::VectorXd ddot_qp;
  Eigen::VectorXd ddot_qp_w_floatingBase;
  Eigen::VectorXd tau_qp;
  Eigen::VectorXd tau_qp_w_floatingBase;
  Eigen::VectorXd q_cmd;
  Eigen::VectorXd q_cmd_w_floatingBase;
  Eigen::VectorXd tau_cmd_after_pd;

  // For RL
  Eigen::VectorXd q_zero_vector;               // Reference joint positions
  Eigen::VectorXd a_before_vector;             // Last actions applied
  Eigen::VectorXd a_vector;                    // Action in mc_rtc order

  std::vector<std::string> notControlledJoints; // Joints that are not controlled by the RL controller: arms + torso, in that case q_rl = q_zero
  std::vector<std::string> mcRtcJointsOrder;
  
  Eigen::VectorXd a_simuOrder;
  std::vector<int> usedJoints_simuOrder; // Indices of the leg joints in the Maniskill order

  // RL policy 
  std::unique_ptr<RLPolicyInterface> rlPolicy_;
  std::unique_ptr<PolicySimulatorHandling> policySimulatorHandling_;
  utils utils_; // Utility functions for RL controller
  
  std::chrono::steady_clock::time_point lastInferenceTime_;
  Eigen::VectorXd q_rl_vector;  // Hold target position between inference calls
  

  
  // observation data - Policy specific
  Eigen::Vector3d baseAngVel; // Angular velocity of the base
  Eigen::Vector3d rpy; // Roll, Pitch, Yaw angles of the base
  Eigen::VectorXd legPos, legVel, legAction; // Leg position, velocity and action in mc_rtc order

  Eigen::Vector3d cmd_;                        // Command vector [vx, vy, yaw_rate]
  double phase_;                               // Current phase for periodic gait
  double phaseFreq_;                           // Phase frequency (1.2 Hz)
  std::chrono::steady_clock::time_point startPhase_; // Start time for phase calculation
    
  Eigen::VectorXd currentObservation_;   // Protected by observationMutex_
  Eigen::VectorXd currentAction_;        // Protected by actionMutex_
  Eigen::VectorXd latestAction_;         // current action being used by control loop
  
  bool useAsyncInference_;

  // Timing and statistics
  bool logTiming_ = false;
  size_t timingLogInterval_ = 1000;  // Log every N steps
}; 