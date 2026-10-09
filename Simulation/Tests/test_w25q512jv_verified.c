/* Host protocol/storage verification against a stateful W25Q512JV NOR model.
 * NOT a silicon, signal-integrity, STM32 timing, or Renode test. */
#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../Renode/Storage/qspi_nor_validated_storage.h"
#include "../Renode/Storage/w25_artifact_crc.h"

typedef struct {
    uint8_t *mem;
    uint64_t time_us,busy_until_us;
    uint32_t page_ops,erase_ops,read_ops,last_address;
    bool wel,wrong_id,corrupt_after_program;
    uint64_t program_latency_us,erase_latency_us;
} FakeNOR;
static FakeNOR nor;
static uint64_t fake_time(void *ctx) {return ((FakeNOR*)ctx)->time_us;}
static void fake_idle(void *ctx) {((FakeNOR*)ctx)->time_us+=100;}

static bool fake_cmd(void *ctx,uint8_t cmd,uint32_t at,const uint8_t *tx,size_t txsz,
                     uint8_t *rx,size_t rxsz) {
    FakeNOR *f=(FakeNOR*)ctx;
    bool busy=f->time_us < f->busy_until_us;
    if (cmd == W25_CMD_RDSR1 && rx && rxsz==1U) {
        *rx=(busy?W25_STATUS_WIP:0U)|(f->wel?W25_STATUS_WEL:0U);
        return true;
    }
    if (busy) return false;
    if (cmd == W25_CMD_JEDEC && rx && rxsz==3U) {
        rx[0]=f->wrong_id?0x20U:0xEFU;rx[1]=0x40U;rx[2]=0x20U;
        return true;
    }
    if (cmd == W25_CMD_WREN && txsz==0U && rxsz==0U) {
        f->wel=true;return true;
    }
    if ((uint64_t)at+txsz>W25Q512JV_SIZE_BYTES ||
        (uint64_t)at+rxsz>W25Q512JV_SIZE_BYTES) return false;
    if (cmd == W25_CMD_READ4 && rx && rxsz>0) {
        memcpy(rx,f->mem+at,rxsz); ++f->read_ops; f->last_address=at;return true;
    }
    if (cmd == W25_CMD_PROG4 && f->wel && tx && txsz>0 && txsz<=256 &&
        (at%256)+txsz<=256) {
        for (size_t i=0;i<txsz;++i) f->mem[at+i]&=tx[i];
        if (f->corrupt_after_program) f->mem[at]^=1;
        f->busy_until_us=f->time_us+f->program_latency_us;
        f->wel=false;++f->page_ops;return true;
    }
    if (cmd == W25_CMD_ERASE4K4 && f->wel && at%4096==0 && txsz==0) {
        memset(f->mem+at,0xFF,4096);
        f->busy_until_us=f->time_us+f->erase_latency_us;
        f->wel=false;++f->erase_ops;return true;
    }
    return false;
}

static W25Q512JVBus bus(void) {
    W25Q512JVBus b={fake_cmd,fake_time,fake_idle,&nor};return b;
}
static PvExecutionSample sample(uint32_t index) {
    PvExecutionSample v;
    for (int i=0;i<6;i++) v.target_position_units[i]=(int32_t)(index*137U+(uint32_t)i*31U);
    return v;
}
static ValidatedTrajectory meta(uint32_t count,uint32_t crc,uint32_t program) {
    ValidatedTrajectory m;
    memset(&m,0,sizeof(m));
    m.program_id=program;m.sample_period_us=2000;
    m.sample_count=count;m.sample_data_crc=~crc;
    m.duration_s=((double)count-1.0)*0.002;
    m.artifact_crc=w25_artifact_crc(&m);
    return m;
}
static void prepare(QspiNorValidatedStorage *s,uint32_t base,uint32_t capacity) {
    assert(qspi_nor_validated_storage_init(s,base,capacity));
}
static bool publish(QspiNorValidatedStorage *s,uint32_t count,uint32_t program,bool stop_early) {
    if (!qspi_nor_validated_storage_begin(s)) return false;
    uint32_t crc=0xFFFFFFFFU;
    for (uint32_t i=0;i<count;++i) {
        PvExecutionSample v=sample(i+program);
        if (!qspi_nor_validated_storage_write_sample(i,&v,s))return false;
        crc=w25q512jv_crc32(crc,&v,sizeof(v));
    }
    if (stop_early)return true;
    ValidatedTrajectory m=meta(count,crc,program);
    return qspi_nor_validated_storage_commit(&m,s);
}
static void inspect(QspiNorValidatedStorage *s,uint32_t count,uint32_t program) {
    assert(qspi_nor_validated_storage_load_committed(s));
    assert(s->metadata.program_id==program);
    assert(s->metadata.sample_count==count);
    PvExecutionSample v;
    for (uint32_t i=0;i<count;++i) {
        assert(qspi_nor_validated_storage_read_sample(i,&v,s));
        PvExecutionSample want=sample(i+program);
        assert(memcmp(&v,&want,sizeof(v))==0);
    }
}
int main(void) {
    nor.mem=malloc(W25Q512JV_SIZE_BYTES);
    assert(nor.mem!=NULL);
    memset(nor.mem,0xFF,W25Q512JV_SIZE_BYTES);
    nor.program_latency_us=700;
    nor.erase_latency_us=50000;
    W25Q512JVBus b=bus();
    nor.wrong_id=true;
    assert(!qspi_nor_validated_storage_set_bus(&b));
    nor.wrong_id=false;
    assert(qspi_nor_validated_storage_set_bus(&b));
    printf("[PASS] Exact JEDEC EF 40 20 required; incorrect ID rejected\n");

    QspiNorValidatedStorage s={0}, rebooted={0};
    prepare(&s,0,1024);
    assert(!qspi_nor_validated_storage_load_committed(&s));
    assert(publish(&s,300,1,false));
    assert(nor.page_ops < 40); /* 300 samples batched, not 300 commands. */
    prepare(&rebooted,0,1024);
    inspect(&rebooted,300,1);
    printf("[PASS] 256-byte page batching, CRC verified reload (300 samples)\n");

    assert(publish(&rebooted,200,2,true)); /* Power loss before commit. */
    QspiNorValidatedStorage after_crash={0};
    prepare(&after_crash,0,1024);
    inspect(&after_crash,300,1);
    printf("[PASS] Interrupted replacement preserves previous trajectory\n");

    assert(publish(&after_crash,480,3,false));
    QspiNorValidatedStorage select_newest={0};
    prepare(&select_newest,0,1024);
    inspect(&select_newest,480,3);
    /* Corrupt newest payload -> loader rejects it and recovers older slot. */
    uint32_t newest_data=select_newest.base_address+
        (uint32_t)select_newest.active_slot*select_newest.slot_bytes+4096;
    nor.mem[newest_data+64]^=0x80;
    QspiNorValidatedStorage fallback={0};
    prepare(&fallback,0,1024);
    inspect(&fallback,300,1);
    printf("[PASS] Latest valid slot selected; payload corruption triggers fallback\n");

    QspiNorValidatedStorage high={0},high2={0};
    prepare(&high,0x02000000U,128);  /* Beyond old 16 MiB address limit */
    assert(publish(&high,90,4,false));
    prepare(&high2,0x02000000U,128);
    inspect(&high2,90,4);
    assert(nor.last_address>0x01000000U);
    printf("[PASS] 32-bit commands access storage beyond 16 MiB\n");

    /* Inject excessive Page Program latency. Must fail, never mark committed. */
    nor.program_latency_us=50000;
    QspiNorValidatedStorage timed={0};
    prepare(&timed,0x03000000U,64);
    assert(!publish(&timed,30,5,false));
    assert(!timed.committed);
    printf("[PASS] Program timeout detected (device WIP, not controller TCF)\n");
    free(nor.mem);
    puts("ALL HOST W25Q512JV MODEL TESTS PASS");
    return 0;
}
