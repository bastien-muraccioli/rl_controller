#include "RL_utils.h"

namespace RL_utils
{
  void start_rl_state(mc_control::fsm::Controller & ctl_, std::string state_name)
  {
    auto & ctl = static_cast<RLController&>(ctl_);
    mc_rtc::log::info("{} state started", state_name);

    ctl.stepCount_ = 0;
    ctl.startTime_ = std::chrono::duration<double>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
      
    if(!ctl.rlPolicy_ || !ctl.rlPolicy_->isLoaded())
    {
      mc_rtc::log::error("RL policy not loaded in {} state", state_name);
      return;
    }

    mc_rtc::log::success("{} state initialization completed", state_name);

    ctl.gui()->addElement(
      {"RLController", state_name},
      mc_rtc::gui::Label("Steps", [&ctl]() { return std::to_string(ctl.stepCount_); }),
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

  bool run_rl_state(mc_control::fsm::Controller & ctl_, std::string state_name)
  {
    auto & ctl = static_cast<RLController&>(ctl_);
    
    auto startTime = std::chrono::high_resolution_clock::now();
    
    try
    {
      Eigen::VectorXd action;
      
      if(ctl.useAsyncInference_)
      {
        ctl.updateObservationForInference();
        action = ctl.getLatestAction();
      }
      else
      {
        
        Eigen::VectorXd observation = ctl.getCurrentObservation();
        
        action = ctl.rlPolicy_->predict(observation);
      }
      
      ctl.applyAction(action);
      
      ctl.stepCount_++;

      if(ctl.logTiming_ && (ctl.stepCount_ % ctl.timingLogInterval_ == 0))
      {
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
        
        double currentTime = std::chrono::duration<double>(
          std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        double avgFreq = ctl.stepCount_ / (currentTime - ctl.startTime_);
        
        const char* mode = ctl.useAsyncInference_ ? "async" : "sync";
        mc_rtc::log::info("{} Step {} ({}): control loop time = {} μs, avg freq = {:.1f} Hz",
                          state_name, ctl.stepCount_, mode, duration.count(), avgFreq);

        mc_rtc::log::info("Action: min={:.3f}, max={:.3f}, norm={:.3f}",
                          action.minCoeff(), action.maxCoeff(), action.norm());
      }
    }
    catch(const std::exception & e)
    {
      mc_rtc::log::error("{} error at step {}: {}", state_name, ctl.stepCount_, e.what());

      Eigen::VectorXd zeroAction = Eigen::VectorXd::Zero(19);
      ctl.applyAction(zeroAction);
    }
    
    return false;
  }

  void teardown_rl_state(mc_control::fsm::Controller & ctl_, std::string state_name)
  {
    auto & ctl = static_cast<RLController&>(ctl_);
    mc_rtc::log::info("{} state ending after {} steps", state_name, ctl.stepCount_);

    ctl_.gui()->removeCategory({"RLController", state_name});
    
    double currentTime = std::chrono::duration<double>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    double totalTime = currentTime - ctl.startTime_;
    double avgFreq = ctl.stepCount_ / totalTime;

    mc_rtc::log::info("{} final stats: {} steps in {:.2f}s, avg freq = {:.1f} Hz",
                      state_name, ctl.stepCount_, totalTime, avgFreq);
  }
}