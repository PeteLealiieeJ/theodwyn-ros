#ifndef THEODWYNROS_THEOCOMM_BROKERINFO_
#define THEODWYNROS_THEOCOMM_BROKERINFO_

#include <cstdint>

namespace Theodwyn
{
    namespace BrokerInfo
    {
        enum class BroadcastId : int8_t
        {
            Broadcaster,
            Transmitter,
            Robotic
        };

        enum class BroadcastCodes : int8_t
        {
            Confirm, // confirm to next state
            Reset,   // reset brokerage to IDLE state
            Check,   // request information on broker state
        };

        enum class BroadcastStatus : int8_t
        {
            Idle,                        // waiting
            Standby_After_Received,      // received broadcast request from transmitter
            Broadcasting_Configuration,  // robot confirmed broadcast request, transmitter may broadcast to shared topic
            Standby_After_Configuration, // robot reached configuration state, waiting for transmitter to broadcast trajeectory
            Broadcasting_Trajectory,     // transmitter confirmed configuration, transmitter may broadcast to shared topic
            Unknown                      // unknown state (held by interacting systems)
        };
    }
}

#endif