#include "RLExecution.h"
#include "../RLController.h"
#include <mc_rtc/logging.h>
#include <chrono>

void RLExecution::configure(const mc_rtc::Configuration & config)
{
  if(config.has("log_timing"))
  {
    logTiming_ = config("log_timing");
  }
  
  if(config.has("timing_log_interval"))
  {
    timingLogInterval_ = config("timing_log_interval");
  }
}

void RLExecution::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController&>(ctl_);
  
  mc_rtc::log::info("RLExecution state started");
  
  stepCount_ = 0;
  startTime_ = std::chrono::duration<double>(
    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    
  if(!ctl.rlPolicy_ || !ctl.rlPolicy_->isLoaded())
  {
    mc_rtc::log::error("RL policy not loaded in RLExecution state");
    return;
  }
  
  if(!ctl.torqueTask_)
  {
    mc_rtc::log::error("TorqueTask not available in RLExecution state");
    return;
  }
  
  mc_rtc::log::success("RLExecution state initialization completed");
  
  ctl_.gui()->addElement(
    {"RLController", "RLExecution"},
    mc_rtc::gui::Label("Steps", [this]() { return std::to_string(stepCount_); }),
    mc_rtc::gui::Label("Policy Loaded", [&ctl]() { 
      return ctl.rlPolicy_->isLoaded() ? "Yes" : "No"; 
    }),
    mc_rtc::gui::Label("Observation Size", [&ctl]() { 
      return std::to_string(ctl.rlPolicy_->getObservationSize()); 
    }),
    mc_rtc::gui::Label("Action Size", [&ctl]() { 
      return std::to_string(ctl.rlPolicy_->getActionSize()); 
    })
  );
}

bool RLExecution::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLController&>(ctl_);
  
  auto startTime = std::chrono::high_resolution_clock::now();
  
  try
  {
    Eigen::VectorXd observation = ctl.getCurrentObservation();
    
    Eigen::VectorXd action = ctl.rlPolicy_->predict(observation);
    
    ctl.applyAction(action);
    
    stepCount_++;
    
    if(logTiming_ && (stepCount_ % timingLogInterval_ == 0))
    {
      auto endTime = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
      
      double currentTime = std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
      double avgFreq = stepCount_ / (currentTime - startTime_);
      
      mc_rtc::log::info("RLExecution Step {}: inference time = {} μs, avg freq = {:.1f} Hz", 
                        stepCount_, duration.count(), avgFreq);
                        
      mc_rtc::log::info("Observation: min={:.3f}, max={:.3f}, norm={:.3f}", 
                        observation.minCoeff(), observation.maxCoeff(), observation.norm());
      mc_rtc::log::info("Action: min={:.3f}, max={:.3f}, norm={:.3f}", 
                        action.minCoeff(), action.maxCoeff(), action.norm());
                        
      mc_rtc::log::info("Obs breakdown - BaseAngVel: [{:.3f}, {:.3f}, {:.3f}], Roll: {:.3f}, Pitch: {:.3f}",
                        observation(0), observation(1), observation(2), observation(3), observation(4));
      mc_rtc::log::info("JointPos: [{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}]",
                        observation(5), observation(6), observation(7), observation(8), observation(9),
                        observation(10), observation(11), observation(12), observation(13), observation(14));
    }
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("RLExecution error at step {}: {}", stepCount_, e.what());
    
    Eigen::VectorXd zeroAction = Eigen::VectorXd::Zero(19);
    ctl.applyAction(zeroAction);
  }
  
  return false;
}

void RLExecution::teardown(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("RLExecution state ending after {} steps", stepCount_);
  
  ctl_.gui()->removeCategory({"RLController", "RLExecution"});
  
  double currentTime = std::chrono::duration<double>(
    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
  double totalTime = currentTime - startTime_;
  double avgFreq = stepCount_ / totalTime;
  
  mc_rtc::log::info("RLExecution final stats: {} steps in {:.2f}s, avg freq = {:.1f} Hz", 
                    stepCount_, totalTime, avgFreq);
}

EXPORT_SINGLE_STATE("RLExecution", RLExecution) 