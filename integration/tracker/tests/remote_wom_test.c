#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "remote_wom.h"
int main(void)
{
    remote_wom_t s={0}; uint8_t ping[13]={0};
    remote_wom_prepare(&s,0xfffe,0x1234,false);
    assert(!remote_wom_take(&s,10000,false));
    remote_wom_normal(&s,4,100); assert(!s.due_ms); /* no result sent */
    ping[2]=4; remote_wom_ping(&s,ping);
    assert(ping[7]==0x29 && ping[8]==0xff && ping[9]==0xfe && ping[10]==0x12 && ping[11]==0x34);
    remote_wom_normal(&s,3,100); assert(!s.due_ms); /* stale response */
    remote_wom_normal(&s,4,100);
    assert(!remote_wom_take(&s,2099,false));
    assert(remote_wom_take(&s,2100,false));
    assert(!remote_wom_take(&s,2200,false)); /* exactly once */
    remote_wom_prepare(&s,0xfffe,4,true); remote_wom_ping(&s,ping);
    assert(ping[7]==0x2a && ping[9]==2);
    remote_wom_normal(&s,4,100); assert(!remote_wom_take(&s,10000,false));
    remote_wom_prepare(&s,0xfffe,0,false); assert(s.flag==0x2a);
    remote_wom_prepare(&s,3,1,false); assert(s.flag==0x2a); /* timeout not supported */
    remote_wom_prepare(&s,0xfffe,1,false); remote_wom_ping(&s,ping); remote_wom_normal(&s,4,100);
    assert(!remote_wom_take(&s,2100,true)); /* busy state appeared after admission */
    assert(!remote_wom_take(&s,2200,false));
    remote_wom_prepare(&s,0xfffe,1,false); remote_wom_ping(&s,ping); remote_wom_normal(&s,4,100);
    remote_wom_cancel(&s); assert(!remote_wom_take(&s,10000,false));
    memset(ping,0x55,sizeof(ping)); remote_wom_ping(&s,ping); assert(ping[7]==0x55);
    puts("PASS: accepted entry, token, wire result, stale/no ACK, delay, one-shot, busy, cancel, unsupported timeout");
}
