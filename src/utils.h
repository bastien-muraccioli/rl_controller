#pragma once

#include "RLController.h"

namespace utils
{
    // RL states
    void start_rl_state(mc_control::fsm::Controller & ctl_, std::string state_name);
    void run_rl_state(mc_control::fsm::Controller & ctl_, std::string state_name);
    void teardown_rl_state(mc_control::fsm::Controller & ctl_, std::string state_name);
}