#ifndef THEODWYNROS_THEOCOMM_BROKERINFO_
#define THEODWYNROS_THEOCOMM_BROKERINFO_

#include <cstdint>

namespace Theodwyn
{
    /**
     * @brief Shared enumerations describing the broker handshake between
     *        the trajectory transmitter and the robotic (controller) side.
     */
    namespace BrokerInfo
    {
        /// @brief Identifies which party is making a request to the broker.
        enum class BroadcastId : int8_t
        {
            Broadcaster, ///< Generic broadcaster
            Transmitter, ///< Trajectory transmitter node
            Robotic      ///< Robot-side node (e.g. PID controller)
        };

        /// @brief Request codes a client may send to the broker service.
        enum class BroadcastCodes : int8_t
        {
            Confirm, ///< Confirm transition to the next state
            Reset,   ///< Reset brokerage to the Idle state
            Check,   ///< Request information on the broker state
        };

        /// @brief States of the broker's broadcast state machine.
        enum class BroadcastStatus : int8_t
        {
            Idle,                        ///< Waiting
            Standby_After_Received,      ///< Received broadcast request from transmitter
            Broadcasting_Configuration,  ///< Robot confirmed broadcast request, transmitter may broadcast to shared topic
            Standby_After_Configuration, ///< Robot reached configuration state, waiting for transmitter to broadcast trajectory
            Broadcasting_Trajectory,     ///< Transmitter confirmed configuration, transmitter may broadcast to shared topic
            Unknown                      ///< Unknown state (held by interacting systems)
        };
    }
}

#endif
