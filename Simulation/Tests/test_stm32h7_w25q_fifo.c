/* Host register-mock tests for the STM32H745 QUADSPI FIFO adapter.
 * NOT physical timing, real Renode, or true STM32 silicon verification.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../Renode/Storage/stm32h7_w25q_bus.h"

#define QSPI_BASE UINT32_C(0x52005000)
#define CR   (QSPI_BASE+0x00U)
#define DCR  (QSPI_BASE+0x04U)
#define SR   (QSPI_BASE+0x08U)
#define FCR  (QSPI_BASE+0x0CU)
#define DLR  (QSPI_BASE+0x10U)
#define CCR  (QSPI_BASE+0x14U)
#define AR   (QSPI_BASE+0x18U)
#define DR   (QSPI_BASE+0x20U)
#define RCC  UINT32_C(0x580244D4)
#define TCF  (1U<<1)
#define TEF  (1U<<0)
#define FTF  (1U<<2)
#define BUSY (1U<<5)

typedef struct {
    uint32_t cr,dcr,dlr,ccr,ar,rcc;
    uint32_t received_address[64];
    uint8_t received_opcodes[64];
    unsigned commands, aborts, overflow, polls, peak_fifo;
    unsigned transferred, expected, fifo, head, tail;
    uint8_t queue[32];
    uint8_t captured[256];
    unsigned captured_n;
    uint8_t opcode;
    uint32_t address;
    bool active, tcf, tef, read_mode, stalled;
    bool inject_tef, inject_stall;
    uint64_t now_us;
} Fake;
static Fake f;

uint64_t w25q_board_monotonic_us(void) { return ++f.now_us; }

static uint8_t value_for(uint32_t address) {
    return (uint8_t)((address * 37U + 11U) & 0xFFU);
}
static void start_command(void) {
    f.opcode = (uint8_t)(f.ccr & 0xFFU);
    f.address = f.ar;
    f.read_mode = ((f.ccr >> 26) & 3U) == 1U;
    f.active = true;
    f.tcf = false;
    f.tef = f.inject_tef;
    f.stalled = f.inject_stall;
    f.expected = (f.ccr & (1UL<<24)) ? f.dlr+1U : 0U;
    f.transferred = f.fifo = f.head = f.tail = 0U;
    if (f.commands < 64) {
        f.received_opcodes[f.commands] = f.opcode;
        f.received_address[f.commands] = f.address;
    }
    ++f.commands;
    if (f.expected == 0U && !f.stalled && !f.tef) {
        f.tcf = true;
        f.active = false;
    }
}
static void pump_one(void) {
    if (!f.active || f.stalled || f.tef) return;
    if (f.read_mode) {
        if (f.transferred+f.fifo < f.expected && f.fifo < sizeof(f.queue)) {
            f.queue[f.tail++ % sizeof(f.queue)] = value_for(f.address + f.transferred+f.fifo);
            ++f.fifo;
        }
        if (f.transferred+f.fifo == f.expected) f.tcf = true;
        if (f.tcf && f.fifo == 0U) f.active = false;
    } else {
        /* Simulate a much slower physical shift register than the CPU. */
        if (++f.polls % 29U != 0U) return;
        if (f.fifo != 0U) {
            ++f.head;
            --f.fifo;
            ++f.transferred;
        }
        if (f.transferred == f.expected) {
            f.tcf = true;
            f.active = false;
        }
    }
}
uint32_t w25_test_read32(uintptr_t address) {
    if (address == RCC) return f.rcc;
    if (address == CR) return f.cr;
    if (address == DCR) return f.dcr;
    if (address == DLR) return f.dlr;
    if (address == CCR) return f.ccr;
    if (address == AR) return f.ar;
    assert(address == SR);
    pump_one();
    return (f.tef ? TEF : 0U) |
           (f.tcf ? TCF : 0U) |
           ((f.active && !f.read_mode && f.fifo < sizeof(f.queue)) ? FTF : 0U) |
           (f.active ? BUSY : 0U) |
           ((uint32_t)f.fifo << 8);
}
void w25_test_write32(uintptr_t address, uint32_t value) {
    switch(address) {
        case RCC: f.rcc = value; return;
        case CR:
            if ((value & (1U<<1)) != 0U) {
                ++f.aborts;
                f.active=false; f.tcf=false; f.tef=false;
                f.stalled=false; f.fifo=0;
                value &= ~(1U<<1); /* hardware clears ABORT */
            }
            f.cr=value;
            return;
        case DCR: f.dcr=value; return;
        case FCR: if (value & TCF) f.tcf=false; if (value & TEF) f.tef=false; return;
        case DLR: f.dlr=value; return;
        case CCR:
            f.ccr=value;
            if ((value & (3UL<<10)) == 0U) start_command();
            return;
        case AR: f.ar=value; start_command(); return;
        default: assert(!"Unexpected QUADSPI register write");
    }
}
uint8_t w25_test_read8(uintptr_t address) {
    assert(address == DR && f.active && f.read_mode && f.fifo > 0);
    uint8_t b=f.queue[f.head++ % sizeof(f.queue)];
    --f.fifo;
    ++f.transferred;
    if (f.tcf && f.fifo==0U) f.active=false;
    return b;
}
void w25_test_write8(uintptr_t address, uint8_t value) {
    assert(address == DR && f.active && !f.read_mode);
    if (f.fifo == sizeof(f.queue)) { ++f.overflow; return; }
    f.queue[f.tail++ % sizeof(f.queue)] = value;
    ++f.fifo;
    if (f.fifo > f.peak_fifo) f.peak_fifo = f.fifo;
    if (f.captured_n < sizeof(f.captured)) f.captured[f.captured_n++] = value;
}

static void reset(void) { memset(&f,0,sizeof(f)); }
static W25Q512JVBus make_bus(void) {
    W25Q512JVBus b;
    assert(stm32h7_w25q_bus_make(&b));
    assert((f.rcc & (1U<<14)) != 0U);
    assert((f.cr & 1U) != 0U);
    assert(((f.cr >> 24)&0xFFU)==7U);
    assert(((f.dcr >> 16)&0x1FU)==25U);
    assert(((f.dcr >> 8)&7U)==3U);
    return b;
}
static void test_page_program(void) {
    reset(); W25Q512JVBus b=make_bus();
    uint8_t p[256]; for(unsigned i=0;i<sizeof(p);++i) p[i]=(uint8_t)(i^0xA5U);
    assert(b.command(b.ctx,W25_CMD_PROG4,UINT32_C(0x01000200),p,sizeof(p),NULL,0));
    assert(!f.overflow && f.peak_fifo==32U &&
           f.captured_n==sizeof(p) && f.commands==1);
    assert(f.received_address[0]==UINT32_C(0x01000200));
    assert(memcmp(p,f.captured,sizeof(p))==0);
    puts("[PASS] 256-byte program paced by FTF; FIFO saturates at 32, no overflow");
}
static void test_reads(void) {
    reset(); W25Q512JVBus b=make_bus();
    uint8_t data[600];
    const uint32_t base=UINT32_C(0x01FFFFE0);
    assert(b.command(b.ctx,W25_CMD_READ4,base,NULL,0,data,sizeof(data)));
    assert(f.commands==3 && !f.overflow);
    assert(f.received_address[0]==base && f.received_address[1]==base+256U &&
           f.received_address[2]==base+512U);
    for(unsigned i=0;i<sizeof(data);++i) assert(data[i]==value_for(base+i));
    puts("[PASS] 600-byte read drains FIFO, correctly splits bursts above 16MiB");
}
static void test_command_and_bounds(void) {
    reset(); W25Q512JVBus b=make_bus();
    assert(b.command(b.ctx,W25_CMD_WREN,0,NULL,0,NULL,0));
    assert(b.command(b.ctx,W25_CMD_ERASE4K4,UINT32_C(0x03000000),NULL,0,NULL,0));
    assert(f.commands==2 && f.received_opcodes[1]==W25_CMD_ERASE4K4 &&
           f.received_address[1]==UINT32_C(0x03000000));
    uint8_t buf[8]={0};
    const unsigned old=f.commands;
    assert(!b.command(b.ctx,W25_CMD_READ4,W25Q512JV_SIZE_BYTES-4U,NULL,0,buf,sizeof(buf)));
    assert(!b.command(b.ctx,W25_CMD_PROG4,0xFFU,buf,sizeof(buf),NULL,0));
    assert(!b.command(b.ctx,0x5AU,0,NULL,0,NULL,0));
    assert(f.commands==old);
    puts("[PASS] 32-bit erase address, invalid transfers rejected before MMIO");
}
static void test_faults(void) {
    reset(); W25Q512JVBus b=make_bus();
    f.inject_tef=true;
    uint8_t buf[20];
    assert(!b.command(b.ctx,W25_CMD_READ4,0,NULL,0,buf,sizeof(buf)));
    assert(f.aborts!=0U);
    f.inject_tef=false;
    assert(b.command(b.ctx,W25_CMD_READ4,0,NULL,0,buf,sizeof(buf)));
    f.inject_stall=true;
    const unsigned before=f.aborts;
    assert(!b.command(b.ctx,W25_CMD_READ4,0,NULL,0,buf,sizeof(buf)));
    assert(f.aborts>before);
    puts("[PASS] injected transfer error and FIFO timeout fail safely; recovery works");
}
int main(void) {
    test_page_program(); test_reads(); test_command_and_bounds(); test_faults();
    puts("ALL HOST STM32 QUADSPI REGISTER-MOCK TESTS PASS");
    return 0;
}
