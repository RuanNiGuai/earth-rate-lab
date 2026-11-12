#include <assert.h>
#include <stdio.h>
#include "remote_command_gate.h"

int main(void)
{
    /* OFF was executed; NORMAL was lost; receiver requests INFO then ON. */
    assert(remote_command_can_advance(0x23, 0x23, 0x30, true, false));
    assert(remote_command_can_advance(0x30, 0x30, 0x22, true, false));
    assert(remote_command_can_advance(0x22, 0x22, 0x23, true, false));
    /* Retried legacy commands must not execute twice. */
    assert(!remote_command_can_advance(0x23, 0x23, 0x23, true, false));
    assert(!remote_command_can_advance(0x22, 0x22, 0x22, true, false));
    /* A pending/actively executing command cannot be replaced. */
    assert(!remote_command_can_advance(0x22, 0, 0x23, true, false));
    assert(!remote_command_can_advance(0x22, 0x30, 0x23, true, false));
    /* An older PONG is not proof of a successor, including old ON after OFF. */
    assert(!remote_command_can_advance(0x23, 0x23, 0x22, false, false));
    assert(!remote_command_can_advance(0x30, 0x30, 0x23, false, false));
    /* NORMAL and idle still use the established admission/clear paths. */
    assert(!remote_command_can_advance(0, 0, 0x23, true, false));
    assert(!remote_command_can_advance(0x23, 0x23, 0, true, false));
    /* Repeated ARC pose flags advance only with a distinct transaction. */
    assert(remote_command_can_advance(0x51, 0x51, 0x51, true, true));
    assert(!remote_command_can_advance(0x51, 0x51, 0x51, true, false));
    assert(!remote_command_can_advance(0x51, 0, 0x51, true, true));
    assert(remote_command_ack_flag(0x34, 0, true) == 0);
    assert(remote_command_ack_flag(0x35, 0, true) == 0);
    assert(remote_command_ack_flag(0x34, 0x34, true) == 0x34);
    assert(remote_command_ack_flag(0x35, 0x35, true) == 0x35);
    assert(remote_command_ack_flag(0x35, 0, false) == 0x35);
    assert(remote_command_ack_flag(0x30, 0, true) == 0x30);
    puts("20 remote-command transition/completion assertions passed");
    return 0;
}
