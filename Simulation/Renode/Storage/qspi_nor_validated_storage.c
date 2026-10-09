#include "qspi_nor_validated_storage.h"
#include "w25_artifact_crc.h"
#include <stddef.h>
#include <string.h>
#ifdef W25Q_STM32H7_BACKEND
#include "stm32h7_w25q_bus.h"
#endif

#define STORAGE_MAGIC UINT32_C(0x50565452)
#define STORAGE_VERSION 2U
#define STORAGE_COMMIT_MARKER UINT32_C(0x434F4D4D)
#define INVALID_SLOT (-1)

typedef struct {
    uint32_t magic, version, header_size, sample_size;
    uint32_t capacity_samples, sample_count, generation;
    ValidatedTrajectory metadata;
    uint32_t header_crc;
    uint32_t commit_marker;
} StorageHeader;
_Static_assert(sizeof(StorageHeader) <= W25Q512JV_SECTOR_BYTES,
               "trajectory header exceeds 4KiB");

static W25Q512JV flash;
static bool flash_ready;

/* A transport failure is not evidence that a slot is empty/corrupt. Updates
 * require a complete scan; reload may still recover a separately valid slot. */
typedef enum { SLOT_INVALID, SLOT_VALID, SLOT_IO_ERROR } SlotCheck;
static bool discover_slots(QspiNorValidatedStorage *s);

static uint32_t slot_address(const QspiNorValidatedStorage *s, int slot) {
    return s->base_address + (uint32_t)slot * s->slot_bytes;
}
static uint32_t data_address(const QspiNorValidatedStorage *s, int slot) {
    return slot_address(s, slot) + W25Q512JV_SECTOR_BYTES;
}
static uint32_t header_crc(const StorageHeader *h) {
    return ~w25q512jv_crc32(UINT32_C(0xFFFFFFFF), h,
                             offsetof(StorageHeader, header_crc));
}
static bool read_bytes(uint32_t at, void *dst, size_t n) {
    return flash_ready && w25q512jv_read(&flash, at, dst, n);
}
static bool verify_bytes(uint32_t at, const uint8_t *expected, size_t n) {
    uint8_t buf[256];
    while (n > 0U) {
        size_t take = n < sizeof(buf) ? n : sizeof(buf);
        if (!read_bytes(at,buf,take)) return false;
        for (size_t i=0; i<take; ++i)
            if (buf[i] != expected[i]) return false;
        at += (uint32_t)take; expected += take; n -= take;
    }
    return true;
}
static bool verify_erased_sector(uint32_t at) {
    uint8_t buf[256];
    for (uint32_t offset=0; offset<W25Q512JV_SECTOR_BYTES; offset+=sizeof(buf)) {
        if (!read_bytes(at+offset,buf,sizeof(buf))) return false;
        for (size_t i=0; i<sizeof(buf); ++i)
            if (buf[i] != 0xFFU) return false;
    }
    return true;
}
static bool write_bytes(uint32_t at, const uint8_t *src, size_t n) {
    while (n > 0) {
        size_t take = W25Q512JV_PAGE_BYTES - (at % W25Q512JV_PAGE_BYTES);
        if (take > n) take = n;
        if (!w25q512jv_program_page(&flash, at, src, take)) return false;
        at += (uint32_t)take;
        src += take;
        n -= take;
    }
    return true;
}

bool qspi_nor_validated_storage_set_bus(const W25Q512JVBus *bus) {
    flash_ready = false;
    flash_ready = w25q512jv_init(&flash, bus);
    return flash_ready;
}

bool qspi_nor_validated_storage_init(QspiNorValidatedStorage *s,
                                     uint32_t base_address, uint32_t capacity_samples) {
#ifdef W25Q_STM32H7_BACKEND
    if (!flash_ready) {
        W25Q512JVBus b;
        if (!stm32h7_w25q_bus_make(&b) ||
            !qspi_nor_validated_storage_set_bus(&b)) return false;
    }
#endif
    if (!flash_ready || s == NULL || capacity_samples == 0U ||
        base_address % W25Q512JV_SECTOR_BYTES != 0) return false;
    const uint64_t one = (uint64_t)W25Q512JV_SECTOR_BYTES +
                         (uint64_t)capacity_samples * sizeof(PvExecutionSample);
    const uint64_t footprint = ((one + W25Q512JV_SECTOR_BYTES - 1U) /
                    W25Q512JV_SECTOR_BYTES) * W25Q512JV_SECTOR_BYTES;
    /* Two whole slots required: staging writes never erase last committed trajectory. */
    if (footprint > UINT32_MAX || (uint64_t)base_address + 2U * footprint >
        W25Q512JV_SIZE_BYTES) return false;
    memset(s, 0, sizeof(*s));
    s->base_address = base_address;
    s->capacity_samples = capacity_samples;
    s->slot_bytes = (uint32_t)footprint;
    s->active_slot = INVALID_SLOT;
    s->write_slot = INVALID_SLOT;
    s->next_generation = 1;
    return true;
}

void qspi_nor_validated_storage_bind(QspiNorValidatedStorage *s,
                                     PathValidationStorage *out) {
    if (out == NULL) return;
    out->begin = qspi_nor_validated_storage_begin;
    out->write_sample = qspi_nor_validated_storage_write_sample;
    out->commit = qspi_nor_validated_storage_commit;
    out->abort = qspi_nor_validated_storage_abort;
    out->capacity_samples = s != NULL ? s->capacity_samples : 0;
    out->context = s;
}

bool qspi_nor_validated_storage_begin(void *context) {
    QspiNorValidatedStorage *s = (QspiNorValidatedStorage *)context;
    if (s == NULL || !flash_ready || s->writing) return false;
    /* Always rediscover before choosing a write target. This covers reboot
     * without load_committed and an ambiguous previous commit/abort result. */
    if (!discover_slots(s)) return false;
    s->write_slot = s->active_slot == 0 ? 1 : 0;
    if (!w25q512jv_erase_sector(&flash, slot_address(s,s->write_slot)) ||
        !verify_erased_sector(slot_address(s,s->write_slot))) {
        s->write_slot = INVALID_SLOT;
        return false;
    }
    s->next_sector_to_erase = data_address(s,s->write_slot);
    s->write_address = data_address(s,s->write_slot);
    s->page_used = 0;
    s->sample_count = 0;
    s->crc_state = UINT32_C(0xFFFFFFFF);
    s->writing = true;
    /* Keep the active slot intact, but block playback until this write ends. */
    return true;
}

static bool flush_page(QspiNorValidatedStorage *s) {
    if (s->page_used == 0) return true;
    uint32_t end = s->write_address + s->page_used;
    while (s->next_sector_to_erase < end) {
        if (!w25q512jv_erase_sector(&flash,s->next_sector_to_erase)) return false;
        s->next_sector_to_erase += W25Q512JV_SECTOR_BYTES;
    }
    if (!write_bytes(s->write_address, s->page, s->page_used)) return false;
    s->write_address += s->page_used;
    s->page_used = 0;
    return true;
}

bool qspi_nor_validated_storage_write_sample(uint32_t index,
                    const PvExecutionSample *sample, void *context) {
    QspiNorValidatedStorage *s=(QspiNorValidatedStorage *)context;
    if (s == NULL || sample == NULL || !s->writing ||
        index != s->sample_count || index >= s->capacity_samples) return false;
    const uint8_t *p=(const uint8_t *)sample;
    size_t n=sizeof(*sample);
    while (n) {
        uint32_t space = W25Q512JV_PAGE_BYTES - s->page_used;
        uint32_t k = (uint32_t)n < space ? (uint32_t)n : space;
        memcpy(s->page+s->page_used,p,k);
        s->page_used += k;
        p += k; n -= k;
        if (s->page_used == W25Q512JV_PAGE_BYTES && !flush_page(s)) return false;
    }
    s->crc_state = w25q512jv_crc32(s->crc_state,sample,sizeof(*sample));
    ++s->sample_count;
    return true;
}

static SlotCheck verify_data(uint32_t base, uint32_t count, uint32_t expected_crc) {
    uint8_t buf[256];
    uint32_t state=UINT32_C(0xFFFFFFFF);
    uint64_t remaining=(uint64_t)count*sizeof(PvExecutionSample);
    while (remaining) {
        uint32_t take=remaining>sizeof(buf)?sizeof(buf):(uint32_t)remaining;
        if (!read_bytes(base,buf,take)) return SLOT_IO_ERROR;
        state=w25q512jv_crc32(state,buf,take);
        base+=take; remaining-=take;
    }
    return ~state == expected_crc ? SLOT_VALID : SLOT_INVALID;
}

bool qspi_nor_validated_storage_commit(const ValidatedTrajectory *m, void *context) {
    QspiNorValidatedStorage *s=(QspiNorValidatedStorage *)context;
    if (s == NULL || m == NULL || !s->writing ||
        m->sample_count != s->sample_count || s->sample_count == 0U ||
        m->sample_data_crc != ~s->crc_state ||
        m->segment_count > TEACHING_MAX_SEGMENTS ||
        m->artifact_crc != w25_artifact_crc(m) ||
        m->sample_period_us != PATH_VALIDATION_SAMPLE_PERIOD_US ||
        !flush_page(s)) return false;
    const uint32_t dat=data_address(s,s->write_slot);
    if (verify_data(dat,s->sample_count,m->sample_data_crc) != SLOT_VALID) return false;
    StorageHeader h;
    memset(&h,0xFF,sizeof(h));
    h.magic=STORAGE_MAGIC; h.version=STORAGE_VERSION;
    h.header_size=sizeof(h); h.sample_size=sizeof(PvExecutionSample);
    h.capacity_samples=s->capacity_samples; h.sample_count=s->sample_count;
    h.generation=s->next_generation;
    h.metadata=*m;
    h.header_crc=header_crc(&h);
    h.commit_marker=UINT32_MAX;
    uint32_t at=slot_address(s,s->write_slot);
    /* Compare the entire header, including the still-erased marker and any
     * trailing padding. Bus success/WIP-clear do not prove program success. */
    if (!write_bytes(at,(const uint8_t *)&h,offsetof(StorageHeader,commit_marker)) ||
        !verify_bytes(at,(const uint8_t *)&h,sizeof(h))) return false;
    h.commit_marker=STORAGE_COMMIT_MARKER;
    if (!write_bytes(at+offsetof(StorageHeader,commit_marker),
            (const uint8_t *)&h.commit_marker,sizeof(h.commit_marker)) ||
        !verify_bytes(at,(const uint8_t *)&h,sizeof(h)) ||
        verify_data(dat,s->sample_count,m->sample_data_crc) != SLOT_VALID) return false;
    const int8_t new_slot=s->write_slot;
    s->write_slot=INVALID_SLOT;
    s->writing=false;
    s->committed=true;
    s->readback_verified=true;
    s->active_slot=new_slot;
    s->next_generation++;
    s->metadata=*m;
    return true;
}

void qspi_nor_validated_storage_abort(void *context) {
    QspiNorValidatedStorage *s=(QspiNorValidatedStorage *)context;
    if (s == NULL) return;
    s->writing=false; s->write_slot=INVALID_SLOT; s->page_used=0;
    s->committed=false; s->readback_verified=false;
    s->sample_count=0;
    memset(&s->metadata,0,sizeof(s->metadata));
}

static SlotCheck valid_slot(QspiNorValidatedStorage *s, int slot, StorageHeader *h) {
    if (!read_bytes(slot_address(s,slot),h,sizeof(*h))) return SLOT_IO_ERROR;
    if (h->magic != STORAGE_MAGIC || h->version != STORAGE_VERSION ||
        h->header_size != sizeof(*h) ||
        h->sample_size != sizeof(PvExecutionSample) ||
        h->capacity_samples != s->capacity_samples ||
        h->sample_count == 0U || h->sample_count > s->capacity_samples ||
        h->metadata.sample_count != h->sample_count ||
        h->metadata.sample_period_us != PATH_VALIDATION_SAMPLE_PERIOD_US ||
        h->metadata.segment_count > TEACHING_MAX_SEGMENTS ||
        h->metadata.artifact_crc != w25_artifact_crc(&h->metadata) ||
        h->commit_marker != STORAGE_COMMIT_MARKER ||
        h->header_crc != header_crc(h)) return SLOT_INVALID;
    return verify_data(data_address(s,slot), h->sample_count, h->metadata.sample_data_crc);
}

static bool discover_slots(QspiNorValidatedStorage *s) {
    StorageHeader a,b;
    const SlotCheck sa=valid_slot(s,0,&a), sb=valid_slot(s,1,&b);
    s->scan_io_error_mask=(uint8_t)((sa == SLOT_IO_ERROR ? 1U : 0U) |
                                  (sb == SLOT_IO_ERROR ? 2U : 0U));
    const bool complete=sa != SLOT_IO_ERROR && sb != SLOT_IO_ERROR;
    bool va=sa == SLOT_VALID, vb=sb == SLOT_VALID;
    if (!va && !vb) {
        s->committed=false; s->readback_verified=false;
        s->active_slot=INVALID_SLOT; s->sample_count=0;
        s->next_generation=1;
        memset(&s->metadata,0,sizeof(s->metadata));
        return complete;
    }
    int chosen=va?0:1;
    if (va && vb && (int32_t)(b.generation-a.generation)>0) chosen=1;
    StorageHeader *h=chosen==0?&a:&b;
    s->active_slot=(int8_t)chosen;
    s->sample_count=h->sample_count;
    s->metadata=h->metadata;
    s->next_generation=h->generation+1U;
    s->committed=true;
    s->readback_verified=true;
    return complete;
}

bool qspi_nor_validated_storage_load_committed(QspiNorValidatedStorage *s) {
    if (s == NULL || s->writing || !flash_ready) return false;
    (void)discover_slots(s);
    return s->committed;
}

bool qspi_nor_validated_storage_read_samples(uint32_t index,
                    PvExecutionSample *samples, uint32_t count, void *context) {
    QspiNorValidatedStorage *s=(QspiNorValidatedStorage *)context;
    if (s == NULL || samples == NULL || count == 0U ||
        !s->committed || !s->readback_verified || s->writing ||
        s->active_slot==INVALID_SLOT || index > s->sample_count ||
        count > s->sample_count-index) return false;
    return read_bytes(data_address(s,s->active_slot)+index*sizeof(*samples),
                      samples,(size_t)count*sizeof(*samples));
}

bool qspi_nor_validated_storage_read_sample(uint32_t index,
                    PvExecutionSample *sample, void *context) {
    return qspi_nor_validated_storage_read_samples(index,sample,1,context);
}
