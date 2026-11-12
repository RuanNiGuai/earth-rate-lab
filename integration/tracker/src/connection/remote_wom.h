#ifndef REMOTE_WOM_H
#define REMOTE_WOM_H
#include <stdbool.h>
#include <stdint.h>

/* One-shot diagnostic entry only. No timeout/NVS changes. Sleep is deferred
 * until the receiver stops advertising the request after our result PING. */
typedef struct {
    uint16_t token, value;
    uint8_t flag, reply_counter;
    bool reply_sent;
    int64_t due_ms;
} remote_wom_t;

static inline void remote_wom_prepare(remote_wom_t *s, uint16_t value,
                                      uint16_t token, bool blocked)
{
    *s=(remote_wom_t){.token=token};
    if(value!=0xfffeu || !token) { s->flag=0x2a; s->value=1; }
    else if(blocked) { s->flag=0x2a; s->value=2; }
    else { s->flag=0x29; s->value=0xfffe; }
}

static inline void remote_wom_ping(remote_wom_t *s, uint8_t ping[13])
{
    if(!s->flag) return;
    ping[7]=s->flag;
    ping[8]=s->value>>8; ping[9]=s->value;
    ping[10]=s->token>>8; ping[11]=s->token;
    s->reply_counter=ping[2]; s->reply_sent=true;
}

static inline void remote_wom_normal(remote_wom_t *s, uint8_t counter, int64_t now)
{
    if(!s->reply_sent || counter!=s->reply_counter) return;
    s->due_ms=s->flag==0x29 ? now+2000 : 0;
    s->flag=0; s->reply_sent=false;
}

static inline void remote_wom_cancel(remote_wom_t *s) { *s=(remote_wom_t){0}; }

static inline bool remote_wom_take(remote_wom_t *s, int64_t now, bool blocked)
{
    if(!s->due_ms || now<s->due_ms) return false;
    s->due_ms=0;
    return !blocked;
}
#endif
