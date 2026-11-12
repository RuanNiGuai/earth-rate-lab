/* Host-only adapter. VQF/Earth implementations and three policy functions are
 * compiled from production sources by fusion_replay.py. Never opens hardware.
 * Config: tau, rest sigma, forgetting, motion mode, accel gate, min rest,
 * rest LP tau, gyro threshold, accel threshold. Motion 1 freezes MBE only;
 * motion 2 also requires low gyro rate. Gate 3 adds norm to production rate.
 */
#ifdef _WIN32
__declspec(dllexport)
#endif
int replay(int n, const double *gyro, const double *accel, const double *dt,
           const double *earth, const double *config, int ng, int na, double *out)
{
    set_params();
    params.tauAcc=(float)config[0]; params.biasSigmaRest=(float)config[1];
    params.biasForgettingTime=(float)config[2]; params.restMinT=(float)config[5];
    params.restFilterTau=(float)config[6]; params.restThGyr=(float)config[7];
    params.restThAcc=(float)config[8];
    initVqf(&params,&state,&coeffs,(float)(dt[0]/ng),(float)(na<0 ? dt[0]*(-na) : dt[0]/na),-1);
    memset(&earth_pair,0,sizeof(earth_pair));
    mbe_gate_enabled=true; accel_rate_allowed=true; accel_recovery_pending=false;
    accel_trusted_s=0; accel_gate_request=0;
    accel_gate_mode=(int)config[4]==3 ? 2 : (int)config[4];
    float eb[3], trusted=0;
    bool earth_requested=false;
    float anchor[3]={0};
    float stable_seconds=0;
    for(int j=0;j<3;j++) { eb[j]=(float)earth[j]*DEG_TO_RAD; earth_requested|=earth[j]!=0; }
    for(int i=0;i<n;i++) {
        float g[3],a[3],am[3];
        for(int j=0;j<3;j++) { g[j]=(float)gyro[3*i+j]*DEG_TO_RAD; a[j]=(float)accel[3*i+j]; am[j]=a[j]*CONST_EARTH_GRAVITY; }
        mbe_gyro_mag=sqrtf(g[0]*g[0]+g[1]*g[1]+g[2]*g[2]);
        for(int k=0;k<ng;k++) {
            updateGyr(&params,&state,&coeffs,g);
            earth_pair_gyro(&earth_pair,g);
            if(na<0 ? isfinite(accel[3*i]) : (k+1)%(ng/na)==0) {
                motion_bias_gate(a[0],a[1],a[2]);
                if((int)config[3]==1 || ((int)config[3]==2 && mbe_gyro_mag>.05f)) {
                    params.motionBiasEstEnabled=false;
                    earth_pair.params.motionBiasEstEnabled=false;
                }
                bool valid=true;
                if((int)config[4]==3) valid=fabsf(sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])-1)<.05f;
                trusted=accel_observation_trusted(a,valid);
                /* Candidate-only guard: a small variance alone cannot prove
                 * rest. After bootstrap, reject a rest-bias observation whose
                 * low-pass rate is inconsistent with the current bias. This
                 * cannot distinguish an equally large physical bias step. */
                vqf_state_t *bs=earth_pair.enabled ? &earth_pair.state : &state;
                bool guard=false;
                if(config[9]>0 && i*dt[0]>15) {
                    float dev=0;
                    for(int j=0;j<3;j++) {
                        float e=bs->restLastGyrLp[j]-(config[10]>0 ? anchor[j] : bs->bias[j]); dev+=e*e;
                    }
                    guard=dev>(float)(config[9]*config[9])*DEG_TO_RAD*DEG_TO_RAD;
                }
                if(config[10]>0) {
                    if(i*dt[0]<=15) memcpy(anchor,bs->bias,sizeof(anchor));
                    else if(!guard && bs->restDetected) {
                        float alpha=1-expf(-coeffs.accTs/(float)config[10]);
                        for(int j=0;j<3;j++) anchor[j]+=alpha*(bs->bias[j]-anchor[j]);
                    }
                }
                params.restMinT=earth_pair.params.restMinT=guard ? 1e6f : (float)config[5];
                if(guard) { state.restDetected=false;state.restT=0;earth_pair.state.restDetected=false;earth_pair.state.restT=0; }
                /* Experimental scheduling only. Existing detector plus trusted
                 * acceleration must remain stable for 30 seconds before rest
                 * observations gradually gain weight. No quaternion/bias reset. */
                if(config[11]>0) {
                    stable_seconds=(!guard && trusted && bs->restDetected) ? stable_seconds+coeffs.accTs : 0;
                    float blend=fminf(1,fmaxf(0,(stable_seconds-30)/30));
                    float sigma=(float)config[1]*(1-blend)+.005f*blend;
                    float p=sigma*100;p*=p;
                    coeffs.biasRestW=p*p/coeffs.biasV+p;
                    if(earth_pair.enabled)earth_pair.coeffs.biasRestW=coeffs.biasRestW;
                }
                updateAccGated(&params,&state,&coeffs,am,trusted);
                earth_pair_accel_gated(&earth_pair,am,trusted);
                if(earth_requested && !earth_pair.enabled) {
                    float q[4], ew[3];
                    getQuat6D(&state,q);
                    q[1]=-q[1]; q[2]=-q[2]; q[3]=-q[3];
                    earth_pair_project(q,eb,ew);
                    earth_pair_start(&earth_pair,&params,&state,&coeffs,ew);
                }
            }
        }
        vqf_state_t *s=earth_pair.enabled ? &earth_pair.state : &state;
        float q[4]; getQuat6D(s,q);
        for(int j=0;j<4;j++) out[10*i+j]=q[j];
        for(int j=0;j<3;j++) out[10*i+4+j]=s->bias[j]/DEG_TO_RAD;
        out[10*i+7]=s->restDetected;
        out[10*i+8]=trusted;
        out[10*i+9]=earth_pair.enabled;
        for(int j=0;j<10;j++) if(!isfinite(out[10*i+j])) return 1;
    }
    return 0;
}
