#pragma once

// Broker State Machine States
// 0 |-> broadcaster
// 1 |-> transmitter
// 2 |-> robotic

enum class BroadcastBrokerId : int {
    Broadcaster, 
    Transmitter, 
    Robotic
};
