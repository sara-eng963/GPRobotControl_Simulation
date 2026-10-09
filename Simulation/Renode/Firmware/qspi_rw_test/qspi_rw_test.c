#include <stdint.h>

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

#define FLASH_CMD_WRITE_ENABLE  0x06U
#define FLASH_CMD_READ_STATUS   0x05U
#define FLASH_CMD_READ          0x03U
#define FLASH_CMD_PAGE_PROGRAM  0x02U
#define FLASH_CMD_ERASE_4K      0x20U

#define FLASH_STATUS_WEL        (1U << 1)

#define TEST_ADDRESS            0x00000100UL
#define TEST_LENGTH             16U
#define TEST_RESULT_ADDRESS     0x24000000UL

#define TEST_MAGIC              0x51535257UL /* "QSRW" */
#define TEST_RUNNING            0x52554E4EUL /* "RUNN" */
#define TEST_PASS               0x50415353UL /* "PASS" */
#define TEST_FAIL_TIMEOUT       0x54494D45UL /* "TIME" */
#define TEST_FAIL_WEL           0x57454C21UL /* "WEL!" */
#define TEST_FAIL_PREPROGRAM    0x50524521UL /* "PRE!" */
#define TEST_FAIL_ERASE         0x45524153UL /* "ERAS" */
#define TEST_FAIL_PROGRAM       0x50524F47UL /* "PROG" */

#define QSPI_TIMEOUT_LOOPS      1000000UL

typedef struct
{
    uint32_t magic;
    uint32_t status;
    uint32_t stage;
    uint32_t flash_status_after_wren;
    uint32_t mismatch_index;
    uint32_t qspi_status;
    uint8_t final_readback[TEST_LENGTH];
} QspiReadWriteTestResult;

static volatile QspiReadWriteTestResult *const result =
    (volatile QspiReadWriteTestResult *)TEST_RESULT_ADDRESS;

static const uint8_t pre_erase_pattern[TEST_LENGTH] =
{
    0x00U, 0x11U, 0x22U, 0x33U,
    0x44U, 0x55U, 0x66U, 0x77U,
    0x88U, 0x99U, 0xAAU, 0xBBU,
    0xCCU, 0xDDU, 0xEEU, 0x00U
};

static const uint8_t final_pattern[TEST_LENGTH] =
{
    0xDEU, 0xADU, 0xBEU, 0xEFU,
    0x12U, 0x34U, 0x56U, 0x78U,
    0x9AU, 0xBCU, 0xDEU, 0xF0U,
    0x55U, 0xAAU, 0x0FU, 0xF0U
};

static uint8_t scratch[TEST_LENGTH];

static void stop_forever(void)
{
    for (;;)
    {
        __asm volatile ("wfi");
    }
}

static void qspi_clear_transfer_complete(void)
{
    REG32(QSPI_FCR) = QSPI_FCR_CTCF;
}

static int qspi_wait_transfer_complete(void)
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

static int qspi_instruction_only(uint8_t instruction)
{
    qspi_clear_transfer_complete();

    REG32(QSPI_CCR) =
        (uint32_t)instruction |
        QSPI_CCR_IMODE_1LINE;

    if (!qspi_wait_transfer_complete())
    {
        return 0;
    }

    qspi_clear_transfer_complete();
    return 1;
}

static int flash_read_status(uint8_t *status)
{
    if (status == 0)
    {
        return 0;
    }

    qspi_clear_transfer_complete();
    REG32(QSPI_DLR) = 0U;

    REG32(QSPI_CCR) =
        FLASH_CMD_READ_STATUS |
        QSPI_CCR_IMODE_1LINE |
        QSPI_CCR_DMODE_1LINE |
        QSPI_CCR_FMODE_INDIRECT_READ;

    if (!qspi_wait_transfer_complete())
    {
        return 0;
    }

    *status = REG8(QSPI_DR);
    qspi_clear_transfer_complete();

    return 1;
}

static int flash_write_enable(void)
{
    uint8_t status = 0U;

    if (!qspi_instruction_only(FLASH_CMD_WRITE_ENABLE))
    {
        return 0;
    }

    if (!flash_read_status(&status))
    {
        return 0;
    }

    result->flash_status_after_wren = status;

    return (status & FLASH_STATUS_WEL) != 0U;
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

    qspi_clear_transfer_complete();
    REG32(QSPI_DLR) = length - 1U;

    REG32(QSPI_CCR) =
        FLASH_CMD_READ |
        QSPI_CCR_IMODE_1LINE |
        QSPI_CCR_ADMODE_1LINE |
        QSPI_CCR_ADSIZE_24BIT |
        QSPI_CCR_DMODE_1LINE |
        QSPI_CCR_FMODE_INDIRECT_READ;

    REG32(QSPI_AR) = address;

    if (!qspi_wait_transfer_complete())
    {
        return 0;
    }

    for (uint32_t i = 0U; i < length; ++i)
    {
        data[i] = REG8(QSPI_DR);
    }

    qspi_clear_transfer_complete();
    return 1;
}

static int flash_page_program(
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

    qspi_clear_transfer_complete();
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

    if (!qspi_wait_transfer_complete())
    {
        return 0;
    }

    qspi_clear_transfer_complete();
    return 1;
}

static int flash_erase_4k(uint32_t address)
{
    qspi_clear_transfer_complete();

    REG32(QSPI_CCR) =
        FLASH_CMD_ERASE_4K |
        QSPI_CCR_IMODE_1LINE |
        QSPI_CCR_ADMODE_1LINE |
        QSPI_CCR_ADSIZE_24BIT;

    REG32(QSPI_AR) = address;

    if (!qspi_wait_transfer_complete())
    {
        return 0;
    }

    qspi_clear_transfer_complete();
    return 1;
}

static int buffers_match(
    const uint8_t *actual,
    const uint8_t *expected,
    uint32_t length
)
{
    for (uint32_t i = 0U; i < length; ++i)
    {
        if (actual[i] != expected[i])
        {
            result->mismatch_index = i;
            return 0;
        }
    }

    result->mismatch_index = 0xFFFFFFFFUL;
    return 1;
}

static int buffer_is_erased(
    const uint8_t *data,
    uint32_t length
)
{
    for (uint32_t i = 0U; i < length; ++i)
    {
        if (data[i] != 0xFFU)
        {
            result->mismatch_index = i;
            return 0;
        }
    }

    result->mismatch_index = 0xFFFFFFFFUL;
    return 1;
}

int main(void)
{
    result->magic = TEST_MAGIC;
    result->status = TEST_RUNNING;
    result->stage = 0U;
    result->flash_status_after_wren = 0U;
    result->mismatch_index = 0xFFFFFFFFUL;
    result->qspi_status = 0U;

    for (uint32_t i = 0U; i < TEST_LENGTH; ++i)
    {
        result->final_readback[i] = 0U;
        scratch[i] = 0U;
    }

    REG32(RCC_AHB3ENR) |= RCC_QSPIEN;

    REG32(QSPI_CR) = 0U;
    REG32(QSPI_DCR) = QSPI_DCR_FSIZE_64MIB;
    qspi_clear_transfer_complete();
    REG32(QSPI_CR) = QSPI_CR_EN;

    result->stage = 1U;

    if (!flash_write_enable())
    {
        result->status =
            (result->qspi_status & QSPI_SR_TCF) == 0U
                ? TEST_FAIL_TIMEOUT
                : TEST_FAIL_WEL;
        stop_forever();
    }

    result->stage = 2U;

    if (!flash_page_program(
            TEST_ADDRESS,
            pre_erase_pattern,
            TEST_LENGTH
        ))
    {
        result->status = TEST_FAIL_TIMEOUT;
        stop_forever();
    }

    if (
        !flash_read(TEST_ADDRESS, scratch, TEST_LENGTH) ||
        !buffers_match(scratch, pre_erase_pattern, TEST_LENGTH)
    )
    {
        result->status = TEST_FAIL_PREPROGRAM;
        stop_forever();
    }

    result->stage = 3U;

    if (!flash_write_enable())
    {
        result->status = TEST_FAIL_WEL;
        stop_forever();
    }

    if (!flash_erase_4k(TEST_ADDRESS))
    {
        result->status = TEST_FAIL_TIMEOUT;
        stop_forever();
    }

    if (
        !flash_read(TEST_ADDRESS, scratch, TEST_LENGTH) ||
        !buffer_is_erased(scratch, TEST_LENGTH)
    )
    {
        result->status = TEST_FAIL_ERASE;
        stop_forever();
    }

    result->stage = 4U;

    if (!flash_write_enable())
    {
        result->status = TEST_FAIL_WEL;
        stop_forever();
    }

    if (!flash_page_program(
            TEST_ADDRESS,
            final_pattern,
            TEST_LENGTH
        ))
    {
        result->status = TEST_FAIL_TIMEOUT;
        stop_forever();
    }

    result->stage = 5U;

    if (!flash_read(TEST_ADDRESS, scratch, TEST_LENGTH))
    {
        result->status = TEST_FAIL_TIMEOUT;
        stop_forever();
    }

    for (uint32_t i = 0U; i < TEST_LENGTH; ++i)
    {
        result->final_readback[i] = scratch[i];
    }

    if (!buffers_match(scratch, final_pattern, TEST_LENGTH))
    {
        result->status = TEST_FAIL_PROGRAM;
        stop_forever();
    }

    result->stage = 6U;
    result->status = TEST_PASS;

    stop_forever();
}
