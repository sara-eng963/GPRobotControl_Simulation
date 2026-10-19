#include "../Guidance/virtual_ft_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void word(uint8_t *p, uint32_t x)
{
    p[0]=(uint8_t)(x>>24); p[1]=(uint8_t)(x>>16);
    p[2]=(uint8_t)(x>>8); p[3]=(uint8_t)x;
}
static void frame(uint8_t b[GP_VFT_PACKET_SIZE], uint32_t seq,
                  uint32_t enabled, float fx)
{
    memset(b,0,GP_VFT_PACKET_SIZE);
    word(b,GP_VFT_MAGIC); word(b+4,GP_VFT_VERSION);
    word(b+8,seq); word(b+12,enabled);
    uint32_t bits; memcpy(&bits,&fx,sizeof(bits));
    word(b+16,bits);
}
int main(void)
{
    GpVirtualFtState s={0};
    const GpVirtualFtLimits l={5.0,1.0,150000U};
    uint8_t b[GP_VFT_PACKET_SIZE]; double w[6]={0};
    frame(b,1,1,2.0f);
    assert(gp_vft_accept(&s,&l,b,sizeof(b),100U));
    assert(gp_vft_read(&s,&l,120U,w) && w[0]==2.0);
    frame(b,2,1,0.0f);
    assert(gp_vft_accept(&s,&l,b,sizeof(b),130U));
    assert(gp_vft_read(&s,&l,140U,w) && w[0]==0.0);
    frame(b,3,1,-4.0f);
    assert(gp_vft_accept(&s,&l,b,sizeof(b),150U));
    assert(gp_vft_read(&s,&l,160U,w) && w[0]==-4.0);
    assert(!gp_vft_read(&s,&l,150151U,w));
    assert(s.fault_latched && w[0]==0.0);
    assert(s.fault_reason == GP_VFT_FAULT_TIMEOUT);
    assert(!gp_vft_accept(&s,&l,b,sizeof(b),150152U));

    gp_vft_reset(&s);
    frame(b,1,1,INFINITY);
    assert(!gp_vft_accept(&s,&l,b,sizeof(b),0U) && s.fault_latched);
    assert(s.fault_reason == GP_VFT_FAULT_INVALID_PACKET);
    gp_vft_reset(&s);
    frame(b,1,1,6.0f);
    assert(!gp_vft_accept(&s,&l,b,sizeof(b),0U));
    gp_vft_reset(&s);
    frame(b,1,1,1.0f);
    assert(!gp_vft_accept(&s,&l,b,sizeof(b)-1,0U));
    gp_vft_reset(&s);
    assert(gp_vft_accept(&s,&l,b,sizeof(b),0U));
    assert(!gp_vft_accept(&s,&l,b,sizeof(b),100U)); /* duplicate */
    assert(s.fault_reason == GP_VFT_FAULT_SEQUENCE);
    gp_vft_reset(&s);
    frame(b,1,0,1.0f);
    assert(!gp_vft_accept(&s,&l,b,sizeof(b),0U)); /* unsafe disarm */
    gp_vft_reset(&s);
    frame(b,1,0,0.0f);
    assert(gp_vft_accept(&s,&l,b,sizeof(b),0U));
    assert(!gp_vft_read(&s,&l,10U,w)); /* explicit disable */
    /* Sender and receiver share one atomic wire format: test encode->accept. */
    gp_vft_reset(&s);
    const double cmd[6] = {2.5, -1.25, 0.0, 0.25, -0.5, 1.0};
    assert(gp_vft_encode(10U, true, cmd, b));
    assert(gp_vft_accept(&s, &l, b, sizeof(b), 50U));
    assert(gp_vft_read(&s, &l, 60U, w));
    for (unsigned i = 0U; i < 6U; ++i)
        assert(fabs(w[i] - cmd[i]) < 1e-6);
    assert(!gp_vft_encode(11U, true,
                          (const double[6]){NAN,0,0,0,0,0}, b));
    assert(!gp_vft_encode(11U, false, cmd, b));
    assert(gp_vft_encode(11U, true, (const double[6]){0}, b));
    assert(gp_vft_accept(&s, &l, b, sizeof(b), 61U));
    assert(gp_vft_read(&s, &l, 62U, w) && w[0] == 0.0);

    puts("PASS: virtual F/T protocol validation, zero-release and watchdog");
    return 0;
}
