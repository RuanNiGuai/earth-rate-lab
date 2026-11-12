#include "calwire_relay.h"
#include "calwire_protocol.h"
#include "globals.h"
#include "connection/esb.h"
#include "esb_ota.h"
#include "receiver_ota.h"
#include "data_collect.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
static uint8_t pending[48];
static bool active;
static uint32_t deadline;
K_MSGQ_DEFINE(cw_responses,48,4,4);
static int digit(char c){return c>='0'&&c<='9'?c-'0':c>='a'&&c<='f'?c-'a'+10:-1;}
int calwire_relay_submit(const char *hex){
    uint8_t p[48];if(!hex||strlen(hex)!=96)return -EINVAL;
    for(unsigned i=0;i<48;i++){int a=digit(hex[2*i]),b=digit(hex[2*i+1]);if(a<0||b<0)return -EINVAL;p[i]=(a<<4)|b;}
    if(!cw_valid(p,48,0x26))return -EINVAL;
    if(esb_ota_relay_is_active()||receiver_ota_is_active()||esb_channel_is_busy()||
       data_collect_is_active()||data_collect_batch_is_active())return -EBUSY;
    unsigned key=irq_lock();
    if(p[1]>=stored_trackers||memcmp(&stored_tracker_addr[p[1]],p+8,6)){
        irq_unlock(key);return -ENOENT;
    }
    if(active&&(int32_t)(k_uptime_get_32()-deadline)<0&&memcmp(p,pending,48)){
        irq_unlock(key);return -EBUSY;
    }
    memcpy(pending,p,48);deadline=k_uptime_get_32()+3000;active=true;
    irq_unlock(key);return 0;
}
bool calwire_relay_ack(uint8_t slot,uint8_t out[48]){
    unsigned key=irq_lock();
    bool ok=active&&(int32_t)(k_uptime_get_32()-deadline)<0&&pending[1]==slot&&
        slot<stored_trackers&&!memcmp(&stored_tracker_addr[slot],pending+8,6);
    if(ok)memcpy(out,pending,48);
    irq_unlock(key);return ok;
}
void calwire_relay_receive(const uint8_t *p,size_t n){
    if(!cw_valid(p,n,0x27))return;
    unsigned key=irq_lock();
    bool match=active&&p[1]==pending[1]&&p[3]==pending[3]&&!memcmp(p+4,pending+4,12)&&
        p[16]>=4&&!memcmp(p+18,pending+44,4);
    if(match){
        /* Keep pending if the USB response queue is full; host retries. */
        if(!k_msgq_put(&cw_responses,p,K_NO_WAIT))active=false;
    }
    irq_unlock(key);
}
static void worker(void){
    uint8_t p[48];char hex[97];static const char chars[]="0123456789abcdef";
    while(true){
        k_msgq_get(&cw_responses,p,K_FOREVER);
        for(unsigned i=0;i<48;i++){hex[2*i]=chars[p[i]>>4];hex[2*i+1]=chars[p[i]&15];}
        hex[96]=0;printk("CALWIRE %s\n",hex);
    }
}
K_THREAD_DEFINE(calwire_relay_thread,1024,worker,NULL,NULL,NULL,7,0,0);
