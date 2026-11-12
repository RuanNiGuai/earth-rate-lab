#ifndef REMOTE_COMMAND_GATE_H
#define REMOTE_COMMAND_GATE_H
#include <stdbool.h>
#include <stdint.h>

/* NORMAL is zero. Execution retains its echo until NORMAL or an explicit
 * successor from the receiver. Never overwrite an unexecuted command, replay
 * the same legacy command, or advance on an older in-flight PONG. ARC commands
 * additionally have a transaction sequence and keep their existing handoff.
 */
static inline bool remote_command_can_advance(uint8_t current, uint8_t echoed,
        uint8_t incoming, bool latest_pong, bool new_arc_transaction)
{
    return current != 0 && echoed == current && incoming != 0 &&
        (new_arc_transaction || (latest_pong && incoming != current));
}
/* F04 STOP must mean the producer stopped, not merely that its command was
 * received. Otherwise Receiver stops forwarding during the delayed dispatch.
 * Leave legacy command echo behavior unchanged when the feature is disabled.
 */
static inline uint8_t remote_command_ack_flag(uint8_t received, uint8_t executed,
                                              bool f04_enabled)
{
    if (executed) return executed;
    if (f04_enabled && (received == 0x34u || received == 0x35u)) return 0;
    return received;
}
#endif
