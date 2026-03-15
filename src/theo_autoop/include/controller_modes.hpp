#pragma once

#include <cstdint>

enum class ControllerMode : int8_t {
    Idle,       // doing nothing
    Responding, // responding to some requested pose, generates feedback to brokerage when response completes
    Active      // processing commands on topic
};