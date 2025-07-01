#pragma once

#include <mc_control/fsm/State.h>

struct MC_CONTROL_FSM_STATE_DLLAPI RLExecution : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  // State-specific data
  size_t stepCount_ = 0;
  double startTime_ = 0.0;
  
  // Timing and statistics
  bool logTiming_ = false;
  size_t timingLogInterval_ = 1000;  // Log every N steps
}; 