#pragma once

#include <cstdint>

// Broker State Machine States
// 0 |-> broadcaster
// 1 |-> transmitter
// 2 |-> robotic

enum class BroadcastBrokerId : int8_t {
    Broadcaster, 
    Transmitter, 
    Robotic
};
