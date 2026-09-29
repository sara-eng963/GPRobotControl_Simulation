#include <stdint.h>
#include "../../../../StateMachine/States/state_path_validation.h"

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

#define TEST_FLASH_ADDRESS     0x00002000UL
#define TEST_SAMPLE_COUNT      4U
#define TEST_RESULT_ADDRESS    0x24000000UL

#define TEST_MAGIC             0x5056534DUL /* "PVSM" */
#define TEST_RUNNING           0x52554E4EUL /* "RUNN" */
#define TEST_PASS              0x50415353UL /* "PASS" */
#define TEST_FAIL_TIMEOUT      0x54494D45UL /* "TIME" */
#define TEST_FAIL_SIZE         0x53495A45UL /* "SIZE" */
#define TEST_FAIL_COMPARE      0x434D5021UL /* "CMP!" */

#define QSPI_TIMEOUT_LOOPS     1000000UL

_Static_assert(PATH_VALIDATION_DOF == 6, "Test expects a 6-DoF robot.");
_Static_assert(sizeof(PvExecutionSample) == 24U, "PvExecutionSample must remain 24 bytes.");

typedef struct
{
    uint32_t magic;
    uint32_t status;
    uint32_t sample_size;
    uint32_t sample_count;
    uint32_t total_bytes;
    uint32_t mismatch_sample;
    uint32_t mismatch_joint;
    uint32_t qspi_status;
    int32_t first_sample_readback[PATH_VALIDATION_DOF];
} PvSampleTestResult;

static volatile PvSampleTestResult *const result =
    (volatile PvSampleTestResult *)TEST_RESULT_ADDRESS;

static const PvExecutionSample expected_samples[TEST_SAMPLE_COUNT] =
{
    { {  1000,  -2000,   3000,  -4000,   5000,  -6000 } },
    { {  1100,  -1900,   3100,  -3900,   5100,  -5900 } },
    { {  1200,  -1800,   3200,  -3800,   5200,  -5800 } },
    { {  1300,  -1700,   3300,  -3700,   5300,  -5700 } }
};

static PvExecutionSample readback_samples[TEST_SAMPLE_COUNT];

static void stop_forever(void)
{
    for (;;)
    {
        __asm volatile ("wfi");
    }
}

static void clear_tcf(void)
{
    REG32(QSPI_FCR) = QSPI_FCR_CTCF;
}

static int wait_tcf(void)
{
    uint32_t timeout = QSPI_TIMEOUT_LOOPS;

    while (
        (REG32(QSPI_SR) & QSPI_SR_TCF) == 0U &&
        timeout > 0U
    )
    {
        --timeout;
    }

    result->qspi_status = REG32(QSPI_SR);
    return timeout != 0U;
}

static int instruction_only(uint8_t instruction)
{
    clear_tcf();

    REG32(QSPI_CCR) =
        (uint32_t)instruction |
        QSPI_CCR_IMODE_1LINE;

    if (!wait_tcf())
    {
        return 0;
    }

    clear_tcf();
    return 1;
}

static int write_enable(void)
{
    return instruction_only(FLASH_CMD_WRITE_ENABLE);
}

static int erase_4k(uint32_t address)
{
    clear_tcf();

    REG32(QSPI_CCR) =
        FLASH_CMD_ERASE_4K |
        QSPI_CCR_IMODE_1LINE |
        QSPI_CCR_ADMODE_1LINE |
        QSPI_CCR_ADSIZE_24BIT;

    REG32(QSPI_AR) = address;

    if (!wait_tcf())
    {
        return 0;
    }

    clear_tcf();
    return 1;
}

static int page_program(
    uint32_t address,
    const uint8_t *data,
    uint32_t length
)
{
    if (
        data == 0 ||
        length == 0U ||
        length > 256U
    )
    {
        return 0;
    }

    clear_tcf();
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

    if (!wait_tcf())
    {
        return 0;
    }

    clear_tcf();
    return 1;
}

#define QSPI_READ_CHUNK_BYTES 32U

static int flash_read_chunk(
    uint32_t address,
    uint8_t *data,
    uint32_t length
)
{
    if (
        data == 0 ||
        length == 0U ||
        length > QSPI_READ_CHUNK_BYTES
    )
    {
        return 0;
    }

    clear_tcf();
    REG32(QSPI_DLR) = length - 1U;

    REG32(QSPI_CCR) =
        FLASH_CMD_READ |
        QSPI_CCR_IMODE_1LINE |
        QSPI_CCR_ADMODE_1LINE |
        QSPI_CCR_ADSIZE_24BIT |
        QSPI_CCR_DMODE_1LINE |
        QSPI_CCR_FMODE_INDIRECT_READ;

    REG32(QSPI_AR) = address;

    if (!wait_tcf())
    {
        return 0;
    }

    for (uint32_t i = 0U; i < length; ++i)
    {
        data[i] = REG8(QSPI_DR);
    }

    clear_tcf();
    return 1;
}

static int flash_read(
    uint32_t address,
    uint8_t *data,
    uint32_t length
)
{
    if (
        data == 0 ||
        length == 0U
    )
    {
        return 0;
    }

    uint32_t offset = 0U;

    /*
     * The Renode STM32H7 QUADSPI model has a 32-byte FIFO. A single 96-byte
     * indirect read cannot complete before firmware starts draining the FIFO.
     * Read in <=32-byte transactions so each transaction can complete and be
     * consumed before the next one begins. This also mirrors the chunked
     * access pattern needed by the later trajectory streaming backend.
     */
    while (offset < length)
    {
        uint32_t chunk = length - offset;

        if (chunk > QSPI_READ_CHUNK_BYTES)
        {
            chunk = QSPI_READ_CHUNK_BYTES;
        }

        if (!flash_read_chunk(
                address + offset,
                data + offset,
                chunk
            ))
        {
            return 0;
        }

        offset += chunk;
    }

    return 1;
}

static int samples_match(void)
{
    for (uint32_t sample = 0U; sample < TEST_SAMPLE_COUNT; ++sample)
    {
        for (uint32_t joint = 0U; joint < PATH_VALIDATION_DOF; ++joint)
        {
            if (
                readback_samples[sample].target_position_units[joint] !=
                expected_samples[sample].target_position_units[joint]
            )
            {
                result->mismatch_sample = sample;
                result->mismatch_joint = joint;
                return 0;
            }
        }
    }

    result->mismatch_sample = 0xFFFFFFFFUL;
    result->mismatch_joint = 0xFFFFFFFFUL;
    return 1;
}

int main(void)
{
    const uint32_t sample_size = (uint32_t)sizeof(PvExecutionSample);
    const uint32_t total_bytes = sample_size * TEST_SAMPLE_COUNT;

    result->magic = TEST_MAGIC;
    result->status = TEST_RUNNING;
    result->sample_size = sample_size;
    result->sample_count = TEST_SAMPLE_COUNT;
    result->total_bytes = total_bytes;
    result->mismatch_sample = 0xFFFFFFFFUL;
    result->mismatch_joint = 0xFFFFFFFFUL;
    result->qspi_status = 0U;

    if (sample_size != 24U || total_bytes > 256U)
    {
        result->status = TEST_FAIL_SIZE;
        stop_forever();
    }

    REG32(RCC_AHB3ENR) |= RCC_QSPIEN;
    REG32(QSPI_CR) = 0U;
    REG32(QSPI_DCR) = QSPI_DCR_FSIZE_64MIB;
    clear_tcf();
    REG32(QSPI_CR) = QSPI_CR_EN;

    if (!write_enable() || !erase_4k(TEST_FLASH_ADDRESS))
    {
        result->status = TEST_FAIL_TIMEOUT;
        stop_forever();
    }

    if (
        !write_enable() ||
        !page_program(
            TEST_FLASH_ADDRESS,
            (const uint8_t *)expected_samples,
            total_bytes
        )
    )
    {
        result->status = TEST_FAIL_TIMEOUT;
        stop_forever();
    }

    if (
        !flash_read(
            TEST_FLASH_ADDRESS,
            (uint8_t *)readback_samples,
            total_bytes
        )
    )
    {
        result->status = TEST_FAIL_TIMEOUT;
        stop_forever();
    }

    for (uint32_t joint = 0U; joint < PATH_VALIDATION_DOF; ++joint)
    {
        result->first_sample_readback[joint] =
            readback_samples[0].target_position_units[joint];
    }

    if (!samples_match())
    {
        result->status = TEST_FAIL_COMPARE;
        stop_forever();
    }

    result->status = TEST_PASS;
    stop_forever();
}
