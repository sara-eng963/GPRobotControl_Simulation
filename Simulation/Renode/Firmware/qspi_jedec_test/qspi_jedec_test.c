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
#define QSPI_DR         (QSPI_BASE + 0x20UL)

#define QSPI_CR_EN      (1UL << 0)
#define QSPI_SR_TCF     (1UL << 1)
#define QSPI_FCR_CTCF   (1UL << 1)

#define QSPI_DCR_FSIZE_64MIB (25UL << 16)

#define QSPI_CCR_IMODE_1LINE (1UL << 8)
#define QSPI_CCR_DMODE_1LINE (1UL << 24)
#define QSPI_CCR_FMODE_INDIRECT_READ (1UL << 26)

#define JEDEC_READ_ID_COMMAND 0x9FUL

#define TEST_RESULT_ADDRESS 0x24000000UL

#define TEST_MAGIC       0x51535049UL /* "QSPI" */
#define TEST_RUNNING     0x52554E4EUL /* "RUNN" */
#define TEST_PASS        0x50415353UL /* "PASS" */
#define TEST_FAIL_ID     0x4641494CUL /* "FAIL" */
#define TEST_FAIL_TIMEOUT 0x54494D45UL /* "TIME" */

#define EXPECTED_MANUFACTURER 0xEFU
#define EXPECTED_MEMORY_TYPE  0x40U
#define EXPECTED_CAPACITY     0x20U

#define QSPI_TIMEOUT_LOOPS 1000000UL

typedef struct
{
    uint32_t magic;
    uint32_t status;
    uint32_t jedec_id;
    uint32_t qspi_status;
} QspiJedecTestResult;

static volatile QspiJedecTestResult *const result =
    (volatile QspiJedecTestResult *)TEST_RESULT_ADDRESS;

static void stop_forever(void)
{
    for (;;)
    {
        __asm volatile ("wfi");
    }
}

int main(void)
{
    result->magic = TEST_MAGIC;
    result->status = TEST_RUNNING;
    result->jedec_id = 0U;
    result->qspi_status = 0U;

    /*
     * Enable the STM32H7 QUADSPI peripheral clock through RCC AHB3ENR.
     */
    REG32(RCC_AHB3ENR) |= RCC_QSPIEN;

    /*
     * Start from a disabled peripheral, describe the simulated 64 MiB flash,
     * clear any stale transfer-complete flag, then enable QUADSPI.
     */
    REG32(QSPI_CR) = 0U;
    REG32(QSPI_DCR) = QSPI_DCR_FSIZE_64MIB;
    REG32(QSPI_FCR) = QSPI_FCR_CTCF;
    REG32(QSPI_CR) = QSPI_CR_EN;

    /*
     * JEDEC Read ID returns three bytes:
     * manufacturer, memory type, capacity.
     *
     * DLR stores N-1, therefore 2 means a 3-byte transfer.
     */
    REG32(QSPI_DLR) = 2U;

    /*
     * 0x9F command on one line, no address phase, three data bytes on one
     * line, indirect-read functional mode.
     *
     * Writing CCR starts the transaction in the STM32H7 QUADSPI model.
     */
    REG32(QSPI_CCR) =
        JEDEC_READ_ID_COMMAND |
        QSPI_CCR_IMODE_1LINE |
        QSPI_CCR_DMODE_1LINE |
        QSPI_CCR_FMODE_INDIRECT_READ;

    uint32_t timeout = QSPI_TIMEOUT_LOOPS;

    while (
        (REG32(QSPI_SR) & QSPI_SR_TCF) == 0U &&
        timeout > 0U
    )
    {
        --timeout;
    }

    result->qspi_status = REG32(QSPI_SR);

    if (timeout == 0U)
    {
        result->status = TEST_FAIL_TIMEOUT;
        stop_forever();
    }

    /*
     * The Renode STM32H7 model supports byte accesses to the QUADSPI data
     * register. Reading exactly three bytes avoids consuming a fourth byte.
     */
    const uint8_t manufacturer = REG8(QSPI_DR);
    const uint8_t memory_type = REG8(QSPI_DR);
    const uint8_t capacity = REG8(QSPI_DR);

    result->jedec_id =
        ((uint32_t)manufacturer) |
        ((uint32_t)memory_type << 8) |
        ((uint32_t)capacity << 16);

    REG32(QSPI_FCR) = QSPI_FCR_CTCF;

    if (
        manufacturer == EXPECTED_MANUFACTURER &&
        memory_type == EXPECTED_MEMORY_TYPE &&
        capacity == EXPECTED_CAPACITY
    )
    {
        result->status = TEST_PASS;
    }
    else
    {
        result->status = TEST_FAIL_ID;
    }

    stop_forever();
}
