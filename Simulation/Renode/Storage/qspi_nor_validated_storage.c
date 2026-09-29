#include "qspi_nor_validated_storage.h"

#include <stddef.h>

#define REG32(address) (*(volatile uint32_t *)(address))
#define REG8(address)  (*(volatile uint8_t *)(address))

#define RCC_BASE        0x58024400UL
#define RCC_AHB3ENR     (RCC_BASE + 0xD4UL)
#define RCC_QSPIEN      (1UL << 14)

#define QSPI_BASE       0x52005000UL
#define QSPI_CR         (QSPI_BASE + 0x00UL)
#define QSPI_DCR        (QSPI_BASE + 0x04UL)
#define QSPI_SR         (QSPI_BASE + 0x08UL)
#define QSPI_FCR        (QSPI_BASE + 0x0CUL)
#define QSPI_DLR        (QSPI_BASE + 0x10UL)
#define QSPI_CCR        (QSPI_BASE + 0x14UL)
#define QSPI_AR         (QSPI_BASE + 0x18UL)
#define QSPI_DR         (QSPI_BASE + 0x20UL)

#define QSPI_CR_EN      (1UL << 0)
#define QSPI_SR_TCF     (1UL << 1)
#define QSPI_FCR_CTCF   (1UL << 1)
#define QSPI_DCR_FSIZE_64MIB (25UL << 16)

#define QSPI_CCR_IMODE_1LINE (1UL << 8)
#define QSPI_CCR_ADMODE_1LINE (1UL << 10)
#define QSPI_CCR_ADSIZE_24BIT (2UL << 12)
#define QSPI_CCR_DMODE_1LINE (1UL << 24)
#define QSPI_CCR_FMODE_INDIRECT_READ (1UL << 26)

#define FLASH_CMD_WRITE_ENABLE 0x06U
#define FLASH_CMD_PAGE_PROGRAM 0x02U
#define FLASH_CMD_ERASE_4K     0x20U
#define FLASH_CMD_READ         0x03U

#define QSPI_TIMEOUT_LOOPS     1000000UL
#define QSPI_READ_CHUNK_BYTES  32U
#define FLASH_24BIT_LIMIT      0x01000000UL

#define STORAGE_MAGIC          0x50565452UL /* "PVTR" */
#define STORAGE_VERSION        1UL
#define STORAGE_COMMIT_MARKER  0x434F4D4DUL /* "COMM" */

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t header_size;
    uint32_t sample_size;
    uint32_t capacity_samples;
    uint32_t sample_count;

    ValidatedTrajectory metadata;

    uint32_t commit_marker;

} QspiNorTrajectoryHeader;

_Static_assert(
    sizeof(QspiNorTrajectoryHeader) <= QSPI_NOR_STORAGE_SECTOR_BYTES,
    "Trajectory header must fit in one 4 KiB sector."
);

static void copy_bytes(
    void *destination,
    const void *source,
    uint32_t length
)
{
    uint8_t *dst = (uint8_t *)destination;
    const uint8_t *src = (const uint8_t *)source;

    for (uint32_t i = 0U; i < length; ++i)
    {
        dst[i] = src[i];
    }
}

static void zero_bytes(
    void *destination,
    uint32_t length
)
{
    uint8_t *dst = (uint8_t *)destination;

    for (uint32_t i = 0U; i < length; ++i)
    {
        dst[i] = 0U;
    }
}

static uint32_t sample_data_base(
    const QspiNorValidatedStorage *storage
)
{
    return storage->base_address + QSPI_NOR_STORAGE_SECTOR_BYTES;
}

static void qspi_clear_tcf(void)
{
    REG32(QSPI_FCR) = QSPI_FCR_CTCF;
}

static bool qspi_wait_tcf(void)
{
    uint32_t timeout = QSPI_TIMEOUT_LOOPS;

    while (
        (REG32(QSPI_SR) & QSPI_SR_TCF) == 0U &&
        timeout > 0U
    )
    {
        --timeout;
    }

    return timeout != 0U;
}

static void qspi_hw_init(void)
{
    REG32(RCC_AHB3ENR) |= RCC_QSPIEN;

    REG32(QSPI_CR) = 0U;
    REG32(QSPI_DCR) = QSPI_DCR_FSIZE_64MIB;
    qspi_clear_tcf();
    REG32(QSPI_CR) = QSPI_CR_EN;
}

static bool qspi_instruction_only(uint8_t instruction)
{
    qspi_clear_tcf();

    REG32(QSPI_CCR) =
        (uint32_t)instruction |
        QSPI_CCR_IMODE_1LINE;

    if (!qspi_wait_tcf())
    {
        return false;
    }

    qspi_clear_tcf();
    return true;
}

static bool flash_write_enable(void)
{
    return qspi_instruction_only(FLASH_CMD_WRITE_ENABLE);
}

static bool flash_erase_4k(uint32_t address)
{
    qspi_clear_tcf();

    REG32(QSPI_CCR) =
        FLASH_CMD_ERASE_4K |
        QSPI_CCR_IMODE_1LINE |
        QSPI_CCR_ADMODE_1LINE |
        QSPI_CCR_ADSIZE_24BIT;

    REG32(QSPI_AR) = address;

    if (!qspi_wait_tcf())
    {
        return false;
    }

    qspi_clear_tcf();
    return true;
}

static bool flash_program_page(
    uint32_t address,
    const uint8_t *data,
    uint32_t length
)
{
    if (
        data == NULL ||
        length == 0U ||
        length > QSPI_NOR_STORAGE_PAGE_BYTES
    )
    {
        return false;
    }

    const uint32_t page_offset =
        address % QSPI_NOR_STORAGE_PAGE_BYTES;

    if (
        page_offset + length >
        QSPI_NOR_STORAGE_PAGE_BYTES
    )
    {
        return false;
    }

    if (!flash_write_enable())
    {
        return false;
    }

    qspi_clear_tcf();
    REG32(QSPI_DLR) = length - 1U;

    REG32(QSPI_CCR) =
        FLASH_CMD_PAGE_PROGRAM |
        QSPI_CCR_IMODE_1LINE |
        QSPI_CCR_ADMODE_1LINE |
        QSPI_CCR_ADSIZE_24BIT |
        QSPI_CCR_DMODE_1LINE;

    REG32(QSPI_AR) = address;

    for (uint32_t i = 0U; i < length; ++i)
    {
        REG8(QSPI_DR) = data[i];
    }

    if (!qspi_wait_tcf())
    {
        return false;
    }

    qspi_clear_tcf();
    return true;
}

static bool flash_program_bytes(
    uint32_t address,
    const void *data,
    uint32_t length
)
{
    if (
        data == NULL ||
        length == 0U
    )
    {
        return false;
    }

    const uint8_t *bytes =
        (const uint8_t *)data;

    uint32_t offset = 0U;

    while (offset < length)
    {
        const uint32_t current_address =
            address + offset;

        const uint32_t page_remaining =
            QSPI_NOR_STORAGE_PAGE_BYTES -
            (current_address % QSPI_NOR_STORAGE_PAGE_BYTES);

        uint32_t chunk =
            length - offset;

        if (chunk > page_remaining)
        {
            chunk = page_remaining;
        }

        if (!flash_program_page(
                current_address,
                bytes + offset,
                chunk
            ))
        {
            return false;
        }

        offset += chunk;
    }

    return true;
}

static bool flash_read_chunk(
    uint32_t address,
    uint8_t *data,
    uint32_t length
)
{
    if (
        data == NULL ||
        length == 0U ||
        length > QSPI_READ_CHUNK_BYTES
    )
    {
        return false;
    }

    qspi_clear_tcf();
    REG32(QSPI_DLR) = length - 1U;

    REG32(QSPI_CCR) =
        FLASH_CMD_READ |
        QSPI_CCR_IMODE_1LINE |
        QSPI_CCR_ADMODE_1LINE |
        QSPI_CCR_ADSIZE_24BIT |
        QSPI_CCR_DMODE_1LINE |
        QSPI_CCR_FMODE_INDIRECT_READ;

    REG32(QSPI_AR) = address;

    if (!qspi_wait_tcf())
    {
        return false;
    }

    for (uint32_t i = 0U; i < length; ++i)
    {
        data[i] = REG8(QSPI_DR);
    }

    qspi_clear_tcf();
    return true;
}

static bool flash_read_bytes(
    uint32_t address,
    void *data,
    uint32_t length
)
{
    if (
        data == NULL ||
        length == 0U
    )
    {
        return false;
    }

    uint8_t *bytes =
        (uint8_t *)data;

    uint32_t offset = 0U;

    while (offset < length)
    {
        uint32_t chunk =
            length - offset;

        if (chunk > QSPI_READ_CHUNK_BYTES)
        {
            chunk = QSPI_READ_CHUNK_BYTES;
        }

        if (!flash_read_chunk(
                address + offset,
                bytes + offset,
                chunk
            ))
        {
            return false;
        }

        offset += chunk;
    }

    return true;
}

static bool ensure_sample_region_erased(
    QspiNorValidatedStorage *storage,
    uint32_t write_address,
    uint32_t length
)
{
    if (
        storage == NULL ||
        length == 0U
    )
    {
        return false;
    }

    const uint32_t last_byte =
        write_address + length - 1U;

    while (
        storage->next_sector_to_erase <=
        last_byte
    )
    {
        if (
            !flash_write_enable() ||
            !flash_erase_4k(
                storage->next_sector_to_erase
            )
        )
        {
            return false;
        }

        storage->next_sector_to_erase +=
            QSPI_NOR_STORAGE_SECTOR_BYTES;
    }

    return true;
}

bool qspi_nor_validated_storage_init(
    QspiNorValidatedStorage *storage,
    uint32_t base_address,
    uint32_t capacity_samples
)
{
    if (
        storage == NULL ||
        capacity_samples == 0U ||
        (base_address % QSPI_NOR_STORAGE_SECTOR_BYTES) != 0U
    )
    {
        return false;
    }

    const uint64_t end_address =
        (uint64_t)base_address +
        QSPI_NOR_STORAGE_SECTOR_BYTES +
        ((uint64_t)capacity_samples *
         sizeof(PvExecutionSample));

    /*
     * This first implementation intentionally uses 3-byte flash addresses.
     */
    if (end_address > FLASH_24BIT_LIMIT)
    {
        return false;
    }

    zero_bytes(
        storage,
        (uint32_t)sizeof(*storage)
    );

    storage->base_address = base_address;
    storage->capacity_samples = capacity_samples;
    storage->next_sector_to_erase =
        sample_data_base(storage);

    qspi_hw_init();

    return true;
}

void qspi_nor_validated_storage_bind(
    QspiNorValidatedStorage *storage,
    PathValidationStorage *interface_out
)
{
    if (interface_out == NULL)
    {
        return;
    }

    interface_out->begin =
        qspi_nor_validated_storage_begin;

    interface_out->write_sample =
        qspi_nor_validated_storage_write_sample;

    interface_out->commit =
        qspi_nor_validated_storage_commit;

    interface_out->abort =
        qspi_nor_validated_storage_abort;

    interface_out->capacity_samples =
        storage != NULL
            ? storage->capacity_samples
            : 0U;

    interface_out->context =
        storage;
}

bool qspi_nor_validated_storage_begin(void *context)
{
    QspiNorValidatedStorage *storage =
        (QspiNorValidatedStorage *)context;

    if (storage == NULL)
    {
        return false;
    }

    /*
     * Erase the header sector first. Until commit writes the final marker,
     * any partially-written sample data is not considered a valid artifact.
     */
    if (
        !flash_write_enable() ||
        !flash_erase_4k(storage->base_address)
    )
    {
        return false;
    }

    storage->sample_count = 0U;
    storage->writing = true;
    storage->committed = false;
    storage->next_sector_to_erase =
        sample_data_base(storage);

    zero_bytes(
        &storage->metadata,
        (uint32_t)sizeof(storage->metadata)
    );

    return true;
}

bool qspi_nor_validated_storage_write_sample(
    uint32_t sample_index,
    const PvExecutionSample *sample,
    void *context
)
{
    QspiNorValidatedStorage *storage =
        (QspiNorValidatedStorage *)context;

    if (
        storage == NULL ||
        sample == NULL ||
        !storage->writing ||
        sample_index >= storage->capacity_samples ||
        sample_index != storage->sample_count
    )
    {
        return false;
    }

    const uint32_t address =
        sample_data_base(storage) +
        sample_index *
        (uint32_t)sizeof(PvExecutionSample);

    if (
        !ensure_sample_region_erased(
            storage,
            address,
            (uint32_t)sizeof(PvExecutionSample)
        )
    )
    {
        return false;
    }

    if (
        !flash_program_bytes(
            address,
            sample,
            (uint32_t)sizeof(PvExecutionSample)
        )
    )
    {
        return false;
    }

    storage->sample_count++;

    return true;
}

bool qspi_nor_validated_storage_commit(
    const ValidatedTrajectory *metadata,
    void *context
)
{
    QspiNorValidatedStorage *storage =
        (QspiNorValidatedStorage *)context;

    if (
        storage == NULL ||
        metadata == NULL ||
        !storage->writing ||
        metadata->sample_count != storage->sample_count
    )
    {
        return false;
    }

    QspiNorTrajectoryHeader header;

    zero_bytes(
        &header,
        (uint32_t)sizeof(header)
    );

    header.magic = STORAGE_MAGIC;
    header.version = STORAGE_VERSION;
    header.header_size =
        (uint32_t)sizeof(header);
    header.sample_size =
        (uint32_t)sizeof(PvExecutionSample);
    header.capacity_samples =
        storage->capacity_samples;
    header.sample_count =
        storage->sample_count;

    copy_bytes(
        &header.metadata,
        metadata,
        (uint32_t)sizeof(*metadata)
    );

    /*
     * Program everything except commit_marker first. The final marker is a
     * separate write so an interrupted commit does not look complete.
     */
    const uint32_t marker_offset =
        (uint32_t)offsetof(
            QspiNorTrajectoryHeader,
            commit_marker
        );

    if (
        !flash_program_bytes(
            storage->base_address,
            &header,
            marker_offset
        )
    )
    {
        return false;
    }

    const uint32_t marker =
        STORAGE_COMMIT_MARKER;

    if (
        !flash_program_bytes(
            storage->base_address +
            marker_offset,
            &marker,
            (uint32_t)sizeof(marker)
        )
    )
    {
        return false;
    }

    copy_bytes(
        &storage->metadata,
        metadata,
        (uint32_t)sizeof(*metadata)
    );

    storage->writing = false;
    storage->committed = true;

    return true;
}

void qspi_nor_validated_storage_abort(void *context)
{
    QspiNorValidatedStorage *storage =
        (QspiNorValidatedStorage *)context;

    if (storage == NULL)
    {
        return;
    }

    /*
     * The header sector was erased at begin(). Without the commit marker,
     * sample bytes left in the data area are intentionally undiscoverable.
     */
    storage->writing = false;
    storage->committed = false;
    storage->sample_count = 0U;

    zero_bytes(
        &storage->metadata,
        (uint32_t)sizeof(storage->metadata)
    );
}

bool qspi_nor_validated_storage_load_committed(
    QspiNorValidatedStorage *storage
)
{
    if (storage == NULL)
    {
        return false;
    }

    QspiNorTrajectoryHeader header;

    if (
        !flash_read_bytes(
            storage->base_address,
            &header,
            (uint32_t)sizeof(header)
        )
    )
    {
        return false;
    }

    if (
        header.magic != STORAGE_MAGIC ||
        header.version != STORAGE_VERSION ||
        header.header_size != sizeof(header) ||
        header.sample_size != sizeof(PvExecutionSample) ||
        header.capacity_samples != storage->capacity_samples ||
        header.sample_count > storage->capacity_samples ||
        header.metadata.sample_count != header.sample_count ||
        header.commit_marker != STORAGE_COMMIT_MARKER
    )
    {
        return false;
    }

    storage->sample_count =
        header.sample_count;

    copy_bytes(
        &storage->metadata,
        &header.metadata,
        (uint32_t)sizeof(storage->metadata)
    );

    storage->writing = false;
    storage->committed = true;

    return true;
}

bool qspi_nor_validated_storage_read_sample(
    uint32_t sample_index,
    PvExecutionSample *sample,
    void *context
)
{
    QspiNorValidatedStorage *storage =
        (QspiNorValidatedStorage *)context;

    if (
        storage == NULL ||
        sample == NULL ||
        !storage->committed ||
        sample_index >= storage->sample_count
    )
    {
        return false;
    }

    const uint32_t address =
        sample_data_base(storage) +
        sample_index *
        (uint32_t)sizeof(PvExecutionSample);

    return flash_read_bytes(
        address,
        sample,
        (uint32_t)sizeof(*sample)
    );
}
