#include "RLController_Initial.h"
#include "../RLController.h"
#include <mc_rtc/logging.h>

void RLController_Initial::configure(const mc_rtc::Configuration & config)
{
  // No configuration needed for initial state
}

void RLController_Initial::start(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("RLController_Initial state started");
}

bool RLController_Initial::run(mc_control::fsm::Controller & ctl)
{
  output("OK");
  return true;
}

void RLController_Initial::teardown(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("RLController_Initial state ending");
}

EXPORT_SINGLE_STATE("RLController_Initial", RLController_Initial)
