/* Host protocol tests against the shared Batch 2 model. No hardware claims. */
#include "w25q_nor_model.h"
#include "../Renode/Storage/w25q512jv_flash.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static Model model;
static uint8_t *image;
static unsigned clock_calls;
static uint8_t rejected_opcode;
static bool reject_command(void *ctx, uint8_t op, uint32_t at,
                           const uint8_t *tx, size_t n, uint8_t *rx, size_t nr)
{
    return op != rejected_opcode && w25_model_command(ctx,op,at,tx,n,rx,nr);
}

static W25Q512JVBus bus(void)
{
    return (W25Q512JVBus){w25_model_command,w25_model_time,w25_model_idle,&model};
}
static void fresh(W25Q512JV *f)
{
    w25_model_init(&model,image,67108864U);
    W25Q512JVBus b=bus();
    assert(w25q512jv_init(f,&b));
    assert(model.now >= 5000 && model.busy_rejections == 0);
}
static bool raw(uint8_t op, uint32_t at, const uint8_t *tx, size_t n,
                uint8_t *rx, size_t nr)
{
    return w25_model_command(&model,op,at,tx,n,rx,nr);
}
static uint8_t status(void)
{
    uint8_t s=0xFF;
    assert(raw(0x05,0,NULL,0,&s,1));
    return s;
}
static void finish(void) { model.now=model.busy_until; assert((status() & 3U) == 0); }
static uint64_t frozen(void *ctx) { (void)ctx; ++clock_calls; return 17; }
static uint64_t backwards(void *ctx) { (void)ctx; return clock_calls++ == 0 ? 20000 : 19999; }

static void test_invalid_objects(void)
{
    W25Q512JV f={0};
    uint8_t value=0xA5;
    assert(!w25q512jv_read(NULL,0,&value,1));
    assert(!w25q512jv_program_page(NULL,0,&value,1));
    assert(!w25q512jv_erase_sector(NULL,0));
    assert(!w25q512jv_read(&f,0,&value,1));
    assert(!w25q512jv_program_page(&f,0,&value,1));
    assert(!w25q512jv_erase_sector(&f,0));
    fresh(&f);
    W25Q512JVBus b=bus();
    assert(!w25q512jv_init(NULL,&b));
    assert(!w25q512jv_init(&f,NULL) && !f.identified);
    assert(!w25q512jv_read(&f,0,&value,1));
    assert(w25q512jv_init(&f,&b));
    b.command=NULL;
    assert(!w25q512jv_init(&f,&b) && !f.identified);
    b=bus(); b.monotonic_us=NULL;
    assert(!w25q512jv_init(&f,&b) && !f.identified);
    f.identified=true; f.bus.monotonic_us=NULL;
    assert(!w25q512jv_read(&f,0,&value,1));
    assert(!w25q512jv_program_page(&f,0,&value,1));
    assert(!w25q512jv_erase_sector(&f,0));
    fresh(&f);
    assert(!w25q512jv_read(&f,0,NULL,1));
    assert(!w25q512jv_read(&f,0,&value,0));
    assert(!w25q512jv_program_page(&f,0,NULL,1));
    assert(!w25q512jv_program_page(&f,0,&value,0));
    assert(!w25q512jv_erase_sector(&f,1));
    puts("[PASS] Null/zeroed objects, invalid callbacks, failed re-init and invalid requests");
}

static void test_startup_and_mcu_reset(void)
{
    W25Q512JV f={0};
    w25_model_init(&model,image,67108864U);
    uint8_t id[3];
    assert(!raw(0x9F,0,NULL,0,id,3)); /* before tVSL */
    model.now=20;
    assert(raw(0x06,0,NULL,0,NULL,0)); /* WREN inhibited before tPUW */
    assert((status() & 2U) == 0);
    W25Q512JVBus b=bus();
    assert(w25q512jv_init(&f,&b));
    assert(model.now >= 5020);
    uint8_t v=0x3C;
    assert(w25q512jv_program_page(&f,0,&v,1));
    for (unsigned erase=0; erase<2; ++erase) {
        fresh(&f);
        model.program_latency_us=3500;
        model.erase_latency_us=400000;
        image[0]=erase ? 0 : 0xFF;
        assert(raw(0x06,0,NULL,0,NULL,0));
        assert(raw(erase ? 0x21 : 0x12,0,erase ? NULL : &v,erase ? 0 : 1,NULL,0));
        uint64_t completion=model.busy_until;
        assert(status() & 1U);
        memset(&f,0,sizeof(f)); /* MCU-only reset: device pending state retained */
        assert(w25q512jv_init(&f,&b));
        assert(model.now >= completion && model.busy_rejections == 0);
        assert(image[0] == (erase ? 0xFF : v));
        assert(model.op_calls[0x66] == 0 && model.op_calls[0x99] == 0);
    }
    fresh(&f);
    b.idle=NULL; model.clock_step_us=100;
    assert(w25q512jv_init(&f,&b)); /* running clock without an idle callback */
    uint32_t bad_ids[]={0x204020,0xEF7020,0xEF4021};
    for (unsigned i=0; i<3; ++i) {
        model.jedec_id=bad_ids[i];
        assert(!w25q512jv_init(&f,&b) && !f.identified);
        unsigned events=model.event_count;
        assert(!w25q512jv_read(&f,0,&v,1) && model.event_count == events);
    }
    model.jedec_id=0xEF4020;
    b=bus(); b.command=reject_command;
    rejected_opcode=0x05;
    assert(!w25q512jv_init(&f,&b) && !f.identified);
    rejected_opcode=0x9F;
    assert(!w25q512jv_init(&f,&b) && !f.identified);
    b=bus(); assert(w25q512jv_init(&f,&b));
    puts("[PASS] Power-up inhibit, exact JEDEC, optional idle and MCU-only reset during WIP");
}

static void test_bounded_failures(void)
{
    W25Q512JV f={0};
    w25_model_init(&model,image,67108864U);
    W25Q512JVBus b=bus();
    model.stuck_wip=true;
    assert(!w25q512jv_init(&f,&b) && !f.identified);
    assert(model.now <= 505000 && model.op_calls[0x9F] == 0);
    w25_model_init(&model,image,67108864U);
    b.monotonic_us=frozen; clock_calls=0;
    assert(!w25q512jv_init(&f,&b) && !f.identified);
    assert(clock_calls <= 2000002 && model.op_calls[0x05] == 0);
    b.monotonic_us=backwards; clock_calls=0;
    assert(!w25q512jv_init(&f,&b) && !f.identified);
    fresh(&f);
    f.bus.monotonic_us=frozen; clock_calls=0; model.stuck_wip=true;
    uint8_t v=0x5A;
    assert(!w25q512jv_read(&f,0,&v,1));
    assert(clock_calls <= 2000002); /* frozen timer with busy device cannot hang */
    fresh(&f);
    f.bus.monotonic_us=backwards; clock_calls=0;
    assert(!w25q512jv_read(&f,0,&v,1));
    fresh(&f);
    model.program_latency_us=20000;
    assert(!w25q512jv_program_page(&f,0,&v,1));
    assert(model.pending_op == 0x12);
    fresh(&f);
    model.erase_latency_us=600000;
    assert(!w25q512jv_erase_sector(&f,0));
    assert(model.pending_op == 0x21);
    fresh(&f);
    model.status_latency_us=10001;
    assert(!w25q512jv_program_page(&f,0,&v,1)); /* ready arrives after deadline */
    assert(image[0] == v); /* failure does not mean bytes remained untouched */
    puts("[PASS] Stuck WIP, frozen/backward clocks, program/erase timeout and late ready response");
}

static void test_wel_busy_and_protection(void)
{
    W25Q512JV f;
    fresh(&f);
    uint8_t v=0xA5, out;
    assert((status() & 3U) == 0);
    assert(raw(0x12,0,&v,1,NULL,0)); /* ignored without WREN */
    assert(image[0] == 0xFF);
    assert(raw(0x06,0,NULL,0,NULL,0) && (status() & 2U));
    assert(raw(0x12,0,&v,1,NULL,0));
    assert((status() & 3U) == 3U);
    assert(!raw(0x06,0,NULL,0,NULL,0));
    assert(!raw(0x13,0,NULL,0,&out,1));
    assert(!raw(0x21,0,NULL,0,NULL,0));
    assert(!raw(0x9F,0,NULL,0,&out,1));
    finish();
    assert(image[0] == v);
    model.reject_wren=true;
    unsigned programs=model.op_calls[0x12];
    assert(!w25q512jv_program_page(&f,1,&v,1));
    assert(model.op_calls[0x12] == programs);
    model.reject_wren=false;
    for (unsigned clears=0; clears<2; ++clears) {
        model.protect_begin=0; model.protect_end=SECTOR;
        model.ignored_clears_wel=clears != 0;
        assert(!w25q512jv_program_page(&f,1,&v,1));
        assert(image[1] == 0xFF);
        assert(!w25q512jv_erase_sector(&f,0));
        assert(image[0] == v);
        assert(model.protect_end == SECTOR);
    }
    model.protect_end=0; model.keep_wel=true;
    assert(!w25q512jv_program_page(&f,2,&v,1)); /* WEL never clears */
    assert(model.wel && image[2] == v);
    assert(model.unknown_commands == 0 && model.op_calls[0x98] == 0);
    puts("[PASS] WREN/WEL, auto-clear, busy rejection, WREN rejection and protected operations");
}

static void test_nor_wrap_and_address_bounds(void)
{
    W25Q512JV f;
    fresh(&f);
    uint8_t p[256], out[256];
    for (unsigned i=0; i<256; ++i) p[i]=(uint8_t)(i ^ 0xA5);
    const uint32_t sectors[]={0,0x00FFF000U,0x01000000U,0x03FFF000U};
    for (unsigned k=0; k<4; ++k) {
        uint32_t base=sectors[k];
        p[0]=(uint8_t)(0xA5U+k); /* distinct bank data exposes read aliasing */
        image[base+33]=0; /* erase must affect this physical target */
        assert(w25q512jv_erase_sector(&f,base));
        assert(image[base+33] == 0xFF);
        assert(w25q512jv_program_page(&f,base,p,256));
        assert(memcmp(image+base,p,256) == 0);
        assert(w25q512jv_read(&f,base,out,256) && memcmp(p,out,256) == 0);
        assert(w25q512jv_program_page(&f,base+256+17,p,7));
        assert(image[base+256+16] == 0xFF && image[base+256+24] == 0xFF);
    }
    uint8_t v=0x3C;
    assert(w25q512jv_program_page(&f,0x00FFFFFF,&v,1));
    assert(w25q512jv_read(&f,0x00FFFFFF,out,2));
    assert(out[0] == v && out[1] == 0xA7 && out[1] != image[0]);
    v=0x5E;
    assert(w25q512jv_program_page(&f,0x03FFFFFF,&v,1));
    assert(image[0x03FFFFFF] == v && image[0x00FFFFFF] == 0x3C);
    assert(w25q512jv_read(&f,0x03FFFFFF,out,1) && out[0] == v);
    unsigned events=model.event_count;
    assert(!w25q512jv_read(&f,0x03FFFFFF,out,2));
    assert(!w25q512jv_read(&f,0x04000000,out,1));
    assert(!w25q512jv_program_page(&f,0x03FFFFFF,p,2));
    assert(!w25q512jv_erase_sector(&f,0x04000000));
    assert(!w25q512jv_program_page(&f,255,p,2));
    assert(!w25q512jv_program_page(&f,0,p,257));
    assert(model.event_count == events);
    assert(w25q512jv_erase_sector(&f,0));
    v=0x0F; assert(w25q512jv_program_page(&f,0,&v,1));
    v=0xF0; assert(!w25q512jv_program_page(&f,0,&v,1));
    assert(image[0] == 0); /* physical AND, never 0->1 */
    assert(w25q512jv_erase_sector(&f,0));
    assert(raw(0x06,0,NULL,0,NULL,0));
    assert(raw(0x12,248,p,16,NULL,0));
    finish();
    for (unsigned i=0; i<16; ++i) assert(image[(248+i)%256] == p[i]);
    assert(image[256] == 0xFF); /* raw device wraps inside its page */
    assert(!raw(0x03,0,NULL,0,out,1)); /* wrong 3-byte read deliberately detected */
    assert(!raw(0x02,0,p,1,NULL,0));
    assert(!raw(0x20,0,NULL,0,NULL,0));
    assert(model.unknown_commands == 3);
    assert(model.op_calls[0x13] && model.op_calls[0x12] && model.op_calls[0x21]);
    puts("[PASS] Full/partial pages, raw page wrap, NOR AND, 16MiB/final address and wrong commands");
}

static void test_device_reset(void)
{
    W25Q512JV f;
    for (unsigned erase=0; erase<2; ++erase) {
        fresh(&f);
        uint8_t p[4]={0x12,0x34,0x56,0x78};
        if (erase) memset(image,0,SECTOR);
        assert(raw(0x06,0,NULL,0,NULL,0));
        assert(raw(erase ? 0x21 : 0x12,0,erase ? NULL : p,erase ? 0 : 4,NULL,0));
        model.reset_partial=2;
        assert(raw(0x66,0,NULL,0,NULL,0));
        assert(raw(0x99,0,NULL,0,NULL,0));
        assert(!model.pending_op && !model.wel);
        uint8_t sr;
        assert(!raw(0x05,0,NULL,0,&sr,1)); /* tRST recovery */
        W25Q512JVBus b=bus();
        assert(w25q512jv_init(&f,&b));
        assert(image[0] == (erase ? 0xFF : p[0]));
        assert(image[1] == (erase ? 0xFF : p[1]));
        assert(image[2] == (erase ? 0 : 0xFF));
    }
    fresh(&f);
    assert(raw(0x66,0,NULL,0,NULL,0));
    (void)status(); /* intervening instruction disarms reset */
    assert(raw(0x06,0,NULL,0,NULL,0));
    assert(raw(0x99,0,NULL,0,NULL,0) && model.wel);
    w25_model_power_cycle(&model);
    assert(!model.wel && !model.pending_op);
    puts("[PASS] Explicit device reset sequence, recovery delay, partial program/erase and power-up WEL");
}

int main(void)
{
    image=malloc(67108864U); assert(image);
    test_invalid_objects();
    test_startup_and_mcu_reset();
    test_bounded_failures();
    test_wel_busy_and_protection();
    test_nor_wrap_and_address_bounds();
    test_device_reset();
    free(image);
    puts("ALL SHARED-MODEL W25Q512JV PROTOCOL TESTS PASS");
    return 0;
}
