#pragma once

enum class ControllerMode : int {
    Idle,       // doing nothing
    Responding, // responding to some requested pose, generates feedback when response completes
    Active      // processing commands on topic
};