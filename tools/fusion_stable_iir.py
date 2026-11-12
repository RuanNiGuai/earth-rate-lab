"""Host-only equivalent Butterworth state-variable realization.

The two states are integrator-equivalent capacitor states rather than DF2T
delay states. Snapshot bytes are incompatible even though sizes are unchanged.
Production firmware is not changed by this experimental source transformation.
"""
REPLACEMENTS = {
'filterCoeffs': r'''
static void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, vqf_double_t b[3], vqf_double_t a[2])
{
    if(tau<=0 || Ts<=0 || tau<Ts/2) { b[0]=1; b[1]=b[2]=a[0]=a[1]=0; return; }
    vqf_double_t g=tanf(Ts/(M_SQRT2*tau));
    b[0]=1/(1+g*(g+M_SQRT2)); b[1]=g*b[0]; b[2]=g*b[1];
    a[0]=g; a[1]=1; /* a[1] is enabled; a[0] records g for diagnostics. */
}
''',
'filterInitialState': r'''
static void filterInitialState(vqf_real_t x, const vqf_double_t b[], const vqf_double_t a[], vqf_double_t state[2])
{
    (void)b; (void)a; state[0]=0; state[1]=x;
}
''',
'filterStep': r'''
static vqf_real_t filterStep(vqf_real_t x, const vqf_double_t b[3], const vqf_double_t a[2], vqf_double_t s[2])
{
    if(a[1]==0) { s[0]=0;s[1]=x;return x; }
    vqf_double_t difference=x-s[1];
    vqf_double_t band=b[0]*s[0]+b[1]*difference;
    vqf_double_t low=s[1]+b[1]*s[0]+b[2]*difference;
    s[0]=2*band-s[0];s[1]=2*low-s[1];return low;
}
''',
'filterAdaptStateForCoeffChange': r'''
static void filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N, const vqf_double_t b_old[3],
                                         const vqf_double_t a_old[2], const vqf_double_t b_new[3],
                                         const vqf_double_t a_new[2], vqf_double_t state[])
{
    (void)last_y;(void)N;(void)b_old;(void)a_old;(void)b_new;(void)a_new;(void)state;
    /* Trapezoidal integrator states are retained across cutoff changes. This
     * has different transient behavior from DF2T; adaptive tau is not enabled. */
}
'''
}


def transform(core, pair, extract):
    for name, body in REPLACEMENTS.items():core=core.replace(extract(core,name),body+'\n',1)
    old='''p->state.restGyrLpState[2*j]-=d[j]*(1-c->restGyrLpB[0]);
        p->state.restGyrLpState[2*j+1]-=d[j]*(c->restGyrLpB[2]-c->restGyrLpA[1]);'''
    if old not in pair:raise ValueError('Earth bias translation source changed')
    pair=pair.replace(old,'p->state.restGyrLpState[2*j+1]-=d[j];',1)
    # The optional yaw realignment uses the same translation with a different
    # delta. Even though replay does not request it, never leave a mixed state
    # representation in the generated Earth module.
    old='''p->state.restGyrLpState[2*j]-=diff[j]*(1-c->restGyrLpB[0]);
        p->state.restGyrLpState[2*j+1]-=diff[j]*(c->restGyrLpB[2]-c->restGyrLpA[1]);'''
    if old not in pair:raise ValueError('Earth realignment translation source changed')
    pair=pair.replace(old,'p->state.restGyrLpState[2*j+1]-=diff[j];',1)
    pair=pair.replace('    const vqf_coeffs_t *c=&p->coeffs;\n','',1)
    return core,pair
