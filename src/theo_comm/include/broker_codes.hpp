#pragma once

#include <cstdint>

// Broker State Machine Codes
// 0 |-> confirm to next state
// 1 |-> reset
// 2 |-> check broker state

enum class BroadcastBrokerCodes : int8_t {
    Confirm,// confirm to next state
    Reset,  // reset brokerage to IDLE state
    Check,  // request information on broker state
};

