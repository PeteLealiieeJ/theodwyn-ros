#pragma once

// Broker State Machine Codes
// 0 |-> confirm to next state
// 1 |-> reset
// 2 |-> check broker state

enum class BroadcastBrokerCodes : int {
    Confirm,// confirm to next state
    Reset,  // received broadcast request from transmitter
    Check,  // robot confirmed broadcast request, transmitter may broadcast to shared topic
};

