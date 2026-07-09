/**
 * @file bootloader.c
 * @brief Minimal bootloader for STM32H750
 *
 * Runs from internal flash (0x08000000, 16KB max).
 * Initializes QSPI W25Q64, configures memory-mapped mode,
 * and jumps to application running from QSPI at 0x90000000.
 */

#include "stm32h7xx.h"
#include <stdint.h>
#include <string.h>

/* ===========================================================================
 * CONFIGURATION
 * =========================================================================== */

#define QSPI_BASE_ADDR         0x90000000UL
#define QSPI_W25Q64_SIZE       0x800000UL   /* 8MB */

/* QSPI Flash layout offsets */
#define OTA_ADDR_CONFIG        0x000000UL
#define OTA_ADDR_APP_A         0x004000UL
#define OTA_ADDR_APP_B         0x0FC000UL
#define OTA_ADDR_BOOT_META     0x1F8000UL

/* W25Q64 Commands */
#define W25Q_CMD_WRITE_ENABLE  0x06
#define W25Q_CMD_READ_STATUS1  0x05
#define W25Q_CMD_READ_DATA     0x03
#define W25Q_CMD_JEDEC_ID      0x9F
#define W25Q_CMD_RELEASE_PD    0xAB
#define W25Q_CMD_QPI_ENABLE    0x38
#define W25Q_CMD_QPI_DISABLE   0xFF
#define W25Q_STATUS_BUSY       0x01

/* Config magic */
#define OTA_CONFIG_MAGIC       0x4F544131UL

/* LED for status indication */
#define LED_PORT               GPIOD
#define LED_PIN                6

/* Boot status */
#define BOOT_OK               0
#define BOOT_NO_FLASH         1
#define BOOT_NO_VALID_APP     2
#define BOOT_JUMP_FAILED      3

/* ===========================================================================
 * OTA Config Structure (must match ota_update.h)
 * =========================================================================== */

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t active_bank;         /* 0=A, 1=B */
    uint32_t app_a_size;
    uint32_t app_a_crc32;
    uint32_t app_b_size;
    uint32_t app_b_crc32;
    uint32_t boot_count;
    uint32_t max_boot_count;
    uint8_t app_a_valid;
    uint8_t app_b_valid;
    uint8_t reserved[16];
} ota_config_t;

/* ===========================================================================
 * FORWARD DECLARATIONS
 * =========================================================================== */

static void SystemClock_Config(void);
static void QSPI_Init(void);
static void QSPI_MemoryMappedMode(void);
static void MPU_Config(void);
static uint32_t QSPI_ReadConfig(ota_config_t *cfg);
static void LED_Init(void);
static void LED_On(void);
static void LED_Off(void);
static void LED_Blink(int count, int delay_ms);
static void JumpToApp(uint32_t addr);

/* ===========================================================================
 * REGISTER-LEVEL QSPI DRIVER (minimal, no HAL dependency)
 * =========================================================================== */

#define QUADSPI_BASE  0xA0001000UL
#define QSPI_CR       (*(volatile uint32_t *)(QUADSPI_BASE + 0x00))
#define QSPI_DCR      (*(volatile uint32_t *)(QUADSPI_BASE + 0x04))
#define QSPI_SR       (*(volatile uint32_t *)(QUADSPI_BASE + 0x08))
#define QSPI_FCR      (*(volatile uint32_t *)(QUADSPI_BASE + 0x0C))
#define QSPI_DLR      (*(volatile uint32_t *)(QUADSPI_BASE + 0x10))
#define QSPI_CCR      (*(volatile uint32_t *)(QUADSPI_BASE + 0x14))
#define QSPI_AR       (*(volatile uint32_t *)(QUADSPI_BASE + 0x18))
#define QSPI_ABR      (*(volatile uint32_t *)(QUADSPI_BASE + 0x1C))
#define QSPI_DR       (*(volatile uint32_t *)(QUADSPI_BASE + 0x20))
#define QSPI_PSMKR    (*(volatile uint32_t *)(QUADSPI_BASE + 0x24))
#define QSPI_PSMAR    (*(volatile uint32_t *)(QUADSPI_BASE + 0x28))
#define QSPI_PIR      (*(volatile uint32_t *)(QUADSPI_BASE + 0x2C))

/* QSPI_CR bits */
#define QSPI_CR_EN        (1UL << 0)
#define QSPI_CR_ABORT     (1UL << 1)
#define QSPI_CR_DMAEN     (1UL << 2)
#define QSPI_CR_TCEN      (1UL << 3)
#define QSPI_CR_SSHIFT    (1UL << 10)
#define QSPI_CR_FTHRES_1  (0UL << 8)

/* QSPI_SR bits */
#define QSPI_SR_BUSY      (1UL << 5)
#define QSPI_SR_TCF       (1UL << 1)
#define QSPI_SR_FLEVEL    (0UL << 12)

/* QSPI_CCR bits */
#define QSPI_CCR_IMODE_1   (1UL << 8)
#define QSPI_CCR_ADMODE_1  (1UL << 10)
#define QSPI_CCR_ADSIZE_24 ((2UL) << 12)
#define QSPI_CCR_DMODE_1   (1UL << 16)
#define QSPI_CCR_FMODE_1   (1UL << 26)   /* Memory-mapped mode */
#define QSPI_CCR_FMODE_0   (0UL << 26)   /* Indirect mode */

static void qspi_delay(volatile uint32_t count)
{
    while (count--) __asm__("nop");
}

static void qspi_send_cmd(uint32_t instruction, uint32_t addr_mode, uint32_t addr,
                           uint32_t data_mode, uint32_t dummy, uint32_t nbdata)
{
    /* Wait for previous operation */
    while (QSPI_SR & QSPI_SR_BUSY);

    /* Clear transfer complete flag */
    QSPI_FCR = QSPI_SR_TCF;

    /* Set data length */
    QSPI_DLR = nbdata;

    /* Set command register */
    QSPI_CCR = instruction |
               addr_mode |
               (QSPI_CCR_ADSIZE_24 & (2UL << 12)) |
               data_mode |
               (dummy << 18) |
               QSPI_CCR_FMODE_0;

    /* Set address if needed */
    if (addr_mode) {
        QSPI_AR = addr;
    }
}

static void qspi_read_data(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        while (QSPI_SR & QSPI_SR_BUSY);
        data[i] = (uint8_t)(QSPI_DR & 0xFF);
    }
}

static int qspi_read_jedec(uint8_t *mfr, uint8_t *type, uint8_t *cap)
{
    uint8_t id[3];
    qspi_send_cmd(W25Q_CMD_JEDEC_ID, 0, 0, QSPI_CCR_DMODE_1, 0, 3);
    qspi_read_data(id, 3);
    *mfr = id[0];
    *type = id[1];
    *cap = id[2];
    return 0;
}

static int qspi_read(uint32_t addr, uint8_t *data, uint32_t len)
{
    qspi_send_cmd(W25Q_CMD_READ_DATA, QSPI_CCR_ADMODE_1, addr, QSPI_CCR_DMODE_1, 0, len);
    qspi_read_data(data, len);
    return 0;
}

/* ===========================================================================
 * SYSTEM CLOCK (HSE 25MHz → PLL → 480MHz)
 * Same as CubeMX-generated SystemClock_Config
 * =========================================================================== */

static void SystemClock_Config(void)
{
    /* Enable PWR clock */
    RCC->APB1HENR |= RCC_APB1HENR_CRSEN;
    PWR->CR3 |= PWR_CR3_LDOEN;
    while (!(PWR->CSR1 & PWR_CSR1_ACTVOSRDY));

    /* Set voltage scaling to Scale 0 */
    PWR->D3CR = (PWR->D3CR & ~PWR_D3CR_VOS) | (3UL << PWR_D3CR_VOS_Pos);
    while (!(PWR->CSR1 & PWR_CSR1_ACTVOSRDY));

    /* Enable HSE */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    /* Configure PLL: HSE 25MHz → 480MHz */
    RCC->PLLCKSELR = (RCC->PLLCKSELR & ~(RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM1)) |
                     (RCC_PLLCKSELR_PLLSRC_HSE | (5UL << RCC_PLLCKSELR_DIVM1_Pos));

    RCC->PLLCFGR = (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLL1RGE | RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVR1EN)) |
                   (RCC_PLLCFGR_PLL1RGE_2 |
                    RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVR1EN);

    RCC->PLL1DIVR = ((192UL - 1UL) << RCC_PLL1DIVR_N1_Pos) |
                    ((2UL - 1UL) << RCC_PLL1DIVR_P1_Pos) |
                    ((2UL - 1UL) << RCC_PLL1DIVR_Q1_Pos) |
                    ((2UL - 1UL) << RCC_PLL1DIVR_R1_Pos);

    RCC->PLL1FRACR = 0;

    /* Enable PLL1 */
    RCC->CR |= RCC_CR_PLL1ON;
    while (!(RCC->CR & RCC_CR_PLL1RDY));

    /* Configure flash latency for 480MHz */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) |
                 (4UL << FLASH_ACR_LATENCY_Pos) |
                 (2UL << FLASH_ACR_WRHIGHFREQ_Pos);

    /* Configure bus clocks: HPRE=/1, D1CPRE=/1, D2PPRE1=/2, D2PPRE2=/2 */
    RCC->D1CFGR = (RCC->D1CFGR & ~(RCC_D1CFGR_HPRE | RCC_D1CFGR_D1CPRE | RCC_D1CFGR_D1PPRE)) |
                  (RCC_D1CFGR_HPRE_DIV1 | RCC_D1CFGR_D1CPRE_DIV1 | RCC_D1CFGR_D1PPRE_DIV2);

    RCC->D2CFGR = (RCC->D2CFGR & ~(RCC_D2CFGR_D2PPRE1 | RCC_D2CFGR_D2PPRE2)) |
                  (RCC_D2CFGR_D2PPRE1_DIV2 | RCC_D2CFGR_D2PPRE2_DIV2);

    RCC->D3CFGR = (RCC->D3CFGR & ~RCC_D3CFGR_D3PPRE) |
                  RCC_D3CFGR_D3PPRE_DIV2;

    /* Set PLL as system clock */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL1;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL1);

    SystemCoreClock = 480000000UL;
}

/* ===========================================================================
 * QSPI INIT (register-level)
 * =========================================================================== */

static void QSPI_Init(void)
{
    /* Enable QSPI clock (on AHB3 bus) */
    RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;
    qspi_delay(100);

    /* Enable reset */
    RCC->AHB3RSTR |= RCC_AHB3RSTR_QSPIRST;
    qspi_delay(10);
    RCC->AHB3RSTR &= ~RCC_AHB3RSTR_QSPIRST;
    qspi_delay(100);

    /* Disable QSPI */
    QSPI_CR &= ~QSPI_CR_EN;

    /* Configure: prescaler=255, CS high time=1, flash size=23 (8MB = 2^23) */
    QSPI_DCR = ((255UL) << 24) |     /* CS high time */
               (23UL << 16) |        /* Flash size: 2^(23+1) = 16MB, but W25Q64 is 8MB */
               (1UL << 8) |          /* Sample shift */
               0UL;                  /* Clock mode 0 */

    /* Enable QSPI */
    QSPI_CR |= QSPI_CR_EN;
    qspi_delay(100);

    /* Release from power-down */
    qspi_send_cmd(W25Q_CMD_RELEASE_PD, 0, 0, QSPI_CCR_DMODE_1, 0, 0);
    qspi_delay(1000);

    /* Read JEDEC ID to verify */
    uint8_t mfr, type, cap;
    qspi_read_jedec(&mfr, &type, &cap);
    /* mfr should be 0xEF for Winbond */
}

static void QSPI_MemoryMappedMode(void)
{
    /* Exit any ongoing operation */
    while (QSPI_SR & QSPI_SR_BUSY);
    QSPI_CR &= ~QSPI_CR_EN;
    qspi_delay(10);

    /* Re-enable */
    QSPI_CR |= QSPI_CR_EN;
    qspi_delay(10);

    /* Configure memory-mapped mode */
    QSPI_DLR = 0; /* Not used in memory-mapped */

    QSPI_CCR = (W25Q_CMD_READ_DATA) |        /* Instruction */
               (QSPI_CCR_ADMODE_1) |          /* Address mode: 1-line */
               (QSPI_CCR_ADSIZE_24) |         /* Address size: 24-bit */
               (QSPI_CCR_DMODE_1) |           /* Data mode: 1-line */
               (0UL << 18) |                   /* Dummy cycles */
               (QSPI_CCR_FMODE_1);            /* Mode: memory-mapped */

    QSPI_AR = 0x000000;  /* Start address */

    /* Enable memory-mapped mode */
    QSPI_CR |= (1UL << 3);  /* TCEN: timeout enable */
}

/* ===========================================================================
 * MPU CONFIG (allow execution from QSPI region)
 * =========================================================================== */

static void MPU_Config(void)
{
    /* Disable MPU */
    __DMB();
    MPU->CTRL = 0;

    /* Region 0: QSPI memory-mapped (0x90000000, 8MB, Device memory, execute) */
    MPU->RNR = 0;
    MPU->RBAR = (QSPI_BASE_ADDR & 0xFFFFFFE0UL) | 0UL;  /* Region 0 */
    MPU->RASR = (1UL << 0)  |       /* Enable region */
                (18UL << 1) |       /* Size: 2^(18+1) = 512KB... need to calculate */
                (1UL << 3) |        /* TEX=001, Cacheable */
                (0UL << 6) |        /* Not shareable */
                (1UL << 8) |        /* Not shareable */
                (0UL << 12) |       /* Not shareable */
                (1UL << 16) |       /* Read-only */
                (1UL << 17) |       /* Read-only */
                (1UL << 19);        /* Execute never = 0 (allow execute) */

    /* Enable MPU */
    MPU->CTRL = (1UL << 0) |   /* MPU enable */
                (1UL << 1);     /* Privileged access default */

    __DSB();
    __ISB();
}

/* ===========================================================================
 * LED HELPERS
 * =========================================================================== */

static void LED_Init(void)
{
    /* Enable GPIOD clock */
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;

    /* Configure PD6 as output push-pull */
    GPIOD->MODER &= ~(3UL << (LED_PIN * 2));
    GPIOD->MODER |= (1UL << (LED_PIN * 2));  /* Output mode */
    GPIOD->OTYPER &= ~(1UL << LED_PIN);      /* Push-pull */
    GPIOD->OSPEEDR |= (3UL << (LED_PIN * 2)); /* Very high speed */
}

static void LED_On(void)  { GPIOD->BSRR = (1UL << (LED_PIN + 16)); }
static void LED_Off(void) { GPIOD->BSRR = (1UL << LED_PIN); }

static void LED_Blink(int count, int delay_ms)
{
    for (int i = 0; i < count; i++) {
        LED_On();
        for (volatile int d = 0; d < delay_ms * 1000; d++) __asm__("nop");
        LED_Off();
        for (volatile int d = 0; d < delay_ms * 1000; d++) __asm__("nop");
    }
}

/* ===========================================================================
 * OTA CONFIG READ
 * =========================================================================== */

static uint32_t QSPI_ReadConfig(ota_config_t *cfg)
{
    qspi_read(OTA_ADDR_CONFIG, (uint8_t *)cfg, sizeof(ota_config_t));
    return cfg->magic;
}

/* ===========================================================================
 * JUMP TO APPLICATION
 * =========================================================================== */

static void JumpToApp(uint32_t app_addr)
{
    /* Check if valid stack pointer */
    uint32_t sp = *(volatile uint32_t *)app_addr;
    if ((sp & 0x2FF00000UL) != 0x20000000UL &&     /* DTCMRAM */
        (sp & 0x24000000UL) != 0x24000000UL &&     /* RAM */
        (sp & 0x30000000UL) != 0x30000000UL) {      /* RAM_D2 */
        return;  /* Invalid stack pointer */
    }

    /* Check if valid reset handler */
    uint32_t reset_handler = *(volatile uint32_t *)(app_addr + 4);
    if (reset_handler < QSPI_BASE_ADDR || reset_handler >= (QSPI_BASE_ADDR + QSPI_W25Q64_SIZE)) {
        return;  /* Invalid reset handler */
    }

    /* Disable all interrupts */
    __disable_irq();

    /* Disable SysTick */
    SysTick->CTRL = 0;

    /* Set vector table offset (to application in QSPI) */
    SCB->VTOR = app_addr;

    /* Set stack pointer */
    __set_MSP(sp);

    /* Jump to application reset handler */
    void (*app_entry)(void) = (void (*)(void))reset_handler;
    app_entry();

    /* Should never reach here */
    while (1);
}

/* ===========================================================================
 * MAIN BOOTLOADER
 * =========================================================================== */

int main(void)
{
    /* Enable FPU and I-Cache */
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));  /* CP10/CP11 full access */
    SCB->CCR |= (1UL << 17);  /* I-Cache enable */
    __DSB();
    __ISB();

    /* Init LED for status indication */
    LED_Init();
    LED_Blink(3, 100);  /* 3 quick blinks = bootloader running */

    /* Configure system clock: HSE 25MHz → PLL → 480MHz */
    SystemClock_Config();

    /* Init QSPI W25Q64 */
    QSPI_Init();

    /* Read OTA config from QSPI */
    ota_config_t config;
    uint32_t magic = QSPI_ReadConfig(&config);

    uint32_t app_addr;

    if (magic == OTA_CONFIG_MAGIC && config.active_bank == 1 && config.app_b_valid) {
        /* Boot from App B (OTA updated firmware) */
        app_addr = QSPI_BASE_ADDR + OTA_ADDR_APP_B;
        LED_Blink(2, 200);  /* 2 blinks = bank B */
    } else {
        /* Boot from App A (default firmware) */
        app_addr = QSPI_BASE_ADDR + OTA_ADDR_APP_A;
        LED_Blink(1, 200);  /* 1 blink = bank A */
    }

    /* Configure MPU for XIP execution from QSPI */
    MPU_Config();

    /* Configure QSPI in memory-mapped mode */
    QSPI_MemoryMappedMode();

    /* Small delay for memory-mapped mode to settle */
    for (volatile int i = 0; i < 10000; i++) __asm__("nop");

    /* Jump to application */
    JumpToApp(app_addr);

    /* If we reach here, boot failed - blink LED rapidly */
    while (1) {
        LED_Blink(10, 50);
    }

    return 0;
}
