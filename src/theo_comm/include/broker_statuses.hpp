#pragma once

// Broker State Machine States
// 0 |-> idle
// 1 |-> trajectroy requested, standby 
// 2 |-> broadcasting configuration trajectory
// 3 |-> configuration set, standby
// 4 |-> broadcasting requested trajectory
// 5 |-> broadcast concluded/halted, (likely back to idle)

enum class BroadcastBrokerStatus : int {
    Idle,                       // waiting
    Standby_After_Received,     // received broadcast request from transmitter
    Broadcasting_Configuration, // robot confirmed broadcast request, transmitter may broadcast to shared topic
    Standby_After_Configuration,// robot reached configuration state, waiting for transmitter to broadcast trajeectory
    Broadcasting_Trajectory,    // transmitter confirmed configuration, transmitter may broadcast to shared topic
    Unknown                     // visible state
};
