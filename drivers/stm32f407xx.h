#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/* -------- Volatile Access Macro -------- */
#ifndef __VO
#define __VO    volatile
#endif

/* -------- FLASH & SRAM Memory Map (STM32F407VG) --------
 *
 * References:
 * - RM0090 (STM32F405/407/415/417 Reference Manual), Rev. 17+
 *   • §2.2 Memory map
 *   • §3.3 Flash memory organization (sectors, sizes)
 *   • §3.4 Option bytes
 *   • §2.3.1 Boot memory mapping (SYSCFG_MEMRMP aliasing)
 * - STM32F407 Datasheet (Doc ID 022152)
 *   • §4 Memory mapping
 *
 * Summary:
 * - FLASH main array: 1 MiB @ 0x0800_0000
 *   Sectors: 0–3 (16 KiB), 4 (64 KiB), 5–11 (128 KiB)
 * - System memory (ST bootloader) @ 0x1FFF_0000
 * - Option bytes region @ 0x1FFF_C000 (OB programming interface via FLASH_IF)
 * - SRAM total 192 KiB split as:
 *   • SRAM1 112 KiB @ 0x2000_0000
 *   • SRAM2  16 KiB @ 0x2001_C000
 *   • CCM    64 KiB @ 0x1000_0000 (core-coupled, not accessible by DMA)
 * - Note: 0x0000_0000 alias may map to FLASH or SRAM per SYSCFG_MEMRMP.
 */

// Generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET          RESET
#define FLAG_SET 			SET

/* ---- Size helpers ---- */
#define BYTES_1K                  (1024U)
#define BYTES_1M                  (1024U * 1024U)
#define ADDR_END(base, size)      ((uint32_t)((base) + (size) - 1U))

/* ---- FLASH (main program memory) ---- */
#define FLASH_BASE_ADDR           (0x08000000U)
#define FLASH_SIZE_BYTES          (1U * BYTES_1M)          /* 1 MiB for STM32F407VG */
#define FLASH_END_ADDR            (ADDR_END(FLASH_BASE_ADDR, FLASH_SIZE_BYTES))

/* Sector base addresses (RM0090 §3.3) */
#define FLASH_SECTOR_0_BASE       (0x08000000U)  /*  16 KiB */
#define FLASH_SECTOR_1_BASE       (0x08004000U)  /*  16 KiB */
#define FLASH_SECTOR_2_BASE       (0x08008000U)  /*  16 KiB */
#define FLASH_SECTOR_3_BASE       (0x0800C000U)  /*  16 KiB */
#define FLASH_SECTOR_4_BASE       (0x08010000U)  /*  64 KiB */
#define FLASH_SECTOR_5_BASE       (0x08020000U)  /* 128 KiB */
#define FLASH_SECTOR_6_BASE       (0x08040000U)  /* 128 KiB */
#define FLASH_SECTOR_7_BASE       (0x08060000U)  /* 128 KiB */
#define FLASH_SECTOR_8_BASE       (0x08080000U)  /* 128 KiB */
#define FLASH_SECTOR_9_BASE       (0x080A0000U)  /* 128 KiB */
#define FLASH_SECTOR_10_BASE      (0x080C0000U)  /* 128 KiB */
#define FLASH_SECTOR_11_BASE      (0x080E0000U)  /* 128 KiB */

/* System/option memory (see RM0090 §3.4, Datasheet §4) */
#define SYSTEM_MEMORY_BASE_ADDR   (0x1FFF0000U)            /* ST ROM bootloader */
#define OPTION_BYTES_BASE_ADDR    (0x1FFFC000U)            /* Option bytes region */

/* ---- SRAM ---- */
/* SRAM1: 112 KiB @ 0x2000_0000 (Datasheet §4; RM0090 §2.2) */
#define SRAM1_BASE_ADDR           (0x20000000U)
#define SRAM1_SIZE_BYTES          (112U * BYTES_1K)
#define SRAM1_END_ADDR            (ADDR_END(SRAM1_BASE_ADDR, SRAM1_SIZE_BYTES))

/* SRAM2: 16 KiB @ 0x2001_C000 */
#define SRAM2_BASE_ADDR           (0x2001C000U)
#define SRAM2_SIZE_BYTES          (16U * BYTES_1K)
#define SRAM2_END_ADDR            (ADDR_END(SRAM2_BASE_ADDR, SRAM2_SIZE_BYTES))

/* CCM RAM: 64 KiB @ 0x1000_0000 (Core-Coupled; not DMA-accessible)
 * RM0090 §2.2 notes CCM is tightly coupled to CPU core.
 */
#define CCM_BASE_ADDR             (0x10000000U)
#define CCM_SIZE_BYTES            (64U * BYTES_1K)
#define CCM_END_ADDR              (ADDR_END(CCM_BASE_ADDR, CCM_SIZE_BYTES))

/* Aggregates */
#define SRAM_TOTAL_SIZE_BYTES     (SRAM1_SIZE_BYTES + SRAM2_SIZE_BYTES + CCM_SIZE_BYTES)

/* Aliasing note (RM0090 §2.3.1):
 * 0x0000_0000U may alias FLASH or SRAM depending on SYSCFG->MEMRMP configuration.
 * Use the explicit region bases above for deterministic addressing.
 */



/* -------- Bus Domain Base Addresses (STM32F407VG) --------
 *
 * Reference:
 * - RM0090, Rev. 17 (STM32F405/407/415/417 Reference Manual)
 *   Section 2.2: Memory map
 *   Section 2.3: Peripheral register boundary addresses
 * - STM32F407 Datasheet (Doc ID 022152), Section 4: Memory mapping
 *
 * Bus domains:
 * - AHB1: GPIO, DMA, RCC, FLASH interface, CRC
 * - AHB2: USB OTG FS, DCMI, RNG
 * - AHB3: FSMC/FMC (external memories)
 * - APB1: Low-speed peripherals (TIM2–7, USART2–5, I2C1–3, PWR, etc.)
 * - APB2: High-speed peripherals (TIM1/8, USART1/6, ADCs, SPI1/4, SYSCFG, EXTI, etc.)
 */

/* ----------------- AHB1 -----------------
 * RM0090 Table 5, AHB1 peripheral base: 0x4002 0000
 */
#define AHB1PERIPH_BASE           (0x40020000U)

/* GPIOs (offsets in 0x400 increments) */
#define GPIOA_BASE_ADDR           (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASE_ADDR           (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE_ADDR           (AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASE_ADDR           (AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASE_ADDR           (AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASE_ADDR           (AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASE_ADDR           (AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASE_ADDR           (AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASE_ADDR           (AHB1PERIPH_BASE + 0x2000U)

/* Core AHB1 peripherals */
#define RCC_BASE_ADDR             (AHB1PERIPH_BASE + 0x3800U)  /* Reset & Clock Control */
#define FLASH_IF_BASE_ADDR        (AHB1PERIPH_BASE + 0x3C00U)  /* Flash Interface */
#define DMA1_BASE_ADDR            (AHB1PERIPH_BASE + 0x6000U)
#define DMA2_BASE_ADDR            (AHB1PERIPH_BASE + 0x6400U)

/* ----------------- AHB2 -----------------
 * RM0090 Table 5, AHB2 peripheral base: 0x5000 0000
 */
#define AHB2PERIPH_BASE           (0x50000000U)

#define USB_OTG_FS_BASE_ADDR      (AHB2PERIPH_BASE + 0x00000U)
#define DCMI_BASE_ADDR            (AHB2PERIPH_BASE + 0x50000U)
#define RNG_BASE_ADDR             (AHB2PERIPH_BASE + 0x60800U)

/* ----------------- AHB3 -----------------
 * RM0090 Table 5, AHB3 peripheral base: 0xA000 0000
 */
#define AHB3PERIPH_BASE           (0xA0000000U)

#define FSMC_BASE_ADDR            (AHB3PERIPH_BASE)   /* Flexible Static Memory Controller */

/* ----------------- APB1 -----------------
 * RM0090 Table 5, APB1 peripheral base: 0x4000 0000
 */
#define APB1PERIPH_BASE           (0x40000000U)

/* Example subset */
#define TIM2_BASE_ADDR            (APB1PERIPH_BASE + 0x0000U)
#define TIM3_BASE_ADDR            (APB1PERIPH_BASE + 0x0400U)
#define TIM4_BASE_ADDR            (APB1PERIPH_BASE + 0x0800U)
#define TIM5_BASE_ADDR            (APB1PERIPH_BASE + 0x0C00U)

#define USART2_BASE_ADDR          (APB1PERIPH_BASE + 0x4400U)
#define USART3_BASE_ADDR          (APB1PERIPH_BASE + 0x4800U)
#define UART4_BASE_ADDR           (APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASE_ADDR           (APB1PERIPH_BASE + 0x5000U)

#define I2C1_BASE_ADDR            (APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASE_ADDR            (APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASE_ADDR            (APB1PERIPH_BASE + 0x5C00U)

#define PWR_BASE_ADDR             (APB1PERIPH_BASE + 0x7000U)

/* ----------------- APB2 -----------------
 * RM0090 Table 5, APB2 peripheral base: 0x4001 0000
 */
#define APB2PERIPH_BASE           (0x40010000U)

#define TIM1_BASE_ADDR            (APB2PERIPH_BASE + 0x0000U)
#define TIM8_BASE_ADDR            (APB2PERIPH_BASE + 0x0400U)

#define USART1_BASE_ADDR          (APB2PERIPH_BASE + 0x1000U)
#define USART6_BASE_ADDR          (APB2PERIPH_BASE + 0x1400U)

#define ADC1_BASE_ADDR            (APB2PERIPH_BASE + 0x2000U)
#define ADC2_BASE_ADDR            (APB2PERIPH_BASE + 0x2100U)
#define ADC3_BASE_ADDR            (APB2PERIPH_BASE + 0x2200U)

#define SPI1_BASE_ADDR            (APB2PERIPH_BASE + 0x3000U)
#define SPI2_BASE_ADDR            (APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASE_ADDR            (APB1PERIPH_BASE + 0x3C00U)

#define SYSCFG_BASE_ADDR          (APB2PERIPH_BASE + 0x3800U)
#define EXTI_BASE_ADDR            (APB2PERIPH_BASE + 0x3C00U)

/* -------- Alias: peripheral regions -------- */
#define PERIPH_BASE               (APB1PERIPH_BASE)  /* legacy alias */


/* -------- GPIO Peripheral Register Definition --------
 *
 * Reference:
 * - RM0090 (STM32F405/407/415/417 Reference Manual), Rev. 17+
 *   §8.4 GPIO registers
 *   Table 41: GPIO register map
 *
 * Each GPIO port (A..I) has the same register layout.
 */

/* GPIO register structure */
typedef struct
{
	__VO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00 */
	__VO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04 */
	__VO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08 */
	__VO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C */
	__VO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10 */
	__VO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14 */
	__VO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18 */
	__VO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C */
	__VO uint32_t AFRL;     /*!< GPIO alternate function low register,  Address offset: 0x20 */
	__VO uint32_t AFRH;     /*!< GPIO alternate function high register, Address offset: 0x24 */
} GPIO_RegDef_t;


/* -------- Peripheral Definitions --------
 * Cast base addresses (from AHB1 domain) to struct pointers.
 */
#define GPIOA    ((GPIO_RegDef_t*) GPIOA_BASE_ADDR)
#define GPIOB    ((GPIO_RegDef_t*) GPIOB_BASE_ADDR)
#define GPIOC    ((GPIO_RegDef_t*) GPIOC_BASE_ADDR)
#define GPIOD    ((GPIO_RegDef_t*) GPIOD_BASE_ADDR)
#define GPIOE    ((GPIO_RegDef_t*) GPIOE_BASE_ADDR)
#define GPIOF    ((GPIO_RegDef_t*) GPIOF_BASE_ADDR)
#define GPIOG    ((GPIO_RegDef_t*) GPIOG_BASE_ADDR)
#define GPIOH    ((GPIO_RegDef_t*) GPIOH_BASE_ADDR)
#define GPIOI    ((GPIO_RegDef_t*) GPIOI_BASE_ADDR)

/* -------- SPI Peripheral Register Definition --------
 * Reference: RM0090 §28 (SPI/I2S)
 * Register map offsets relative to SPIx base address.
 */
typedef struct
{
    __VO uint32_t CR1;      /*!< Control register 1,           Offset: 0x00 */
    __VO uint32_t CR2;      /*!< Control register 2,           Offset: 0x04 */
    __VO uint32_t SR;       /*!< Status register,              Offset: 0x08 */
    __VO uint32_t DR;       /*!< Data register,                Offset: 0x0C */
    __VO uint32_t CRCPR;    /*!< CRC polynomial register,      Offset: 0x10 */
    __VO uint32_t RXCRCR;   /*!< RX CRC register,              Offset: 0x14 */
    __VO uint32_t TXCRCR;   /*!< TX CRC register,              Offset: 0x18 */
    __VO uint32_t I2SCFGR;  /*!< I2S configuration register,   Offset: 0x1C */
    __VO uint32_t I2SPR;    /*!< I2S prescaler register,       Offset: 0x20 */
} SPI_RegDef_t;

#define SPI1    ((SPI_RegDef_t*) SPI1_BASE_ADDR)
#define SPI2    ((SPI_RegDef_t*) SPI2_BASE_ADDR)
#define SPI3    ((SPI_RegDef_t*) SPI3_BASE_ADDR)

/* ---- CR1 ---- */
#define SPI_CR1_CPHA         (1U << 0)
#define SPI_CR1_CPOL         (1U << 1)
#define SPI_CR1_MSTR         (1U << 2)
#define SPI_CR1_BR_Pos       3U
#define SPI_CR1_BR_Msk       (7U << SPI_CR1_BR_Pos)  /* /2..256 */
#define SPI_CR1_SPE          (1U << 6)
#define SPI_CR1_LSBFIRST     (1U << 7)
#define SPI_CR1_SSI          (1U << 8)
#define SPI_CR1_SSM          (1U << 9)
#define SPI_CR1_RXONLY       (1U << 10)
#define SPI_CR1_DFF          (1U << 11)              /* 0=8-bit,1=16-bit */
#define SPI_CR1_CRCNEXT      (1U << 12)
#define SPI_CR1_CRCEN        (1U << 13)
#define SPI_CR1_BIDIOE       (1U << 14)
#define SPI_CR1_BIDIMODE     (1U << 15)

/* ---- CR2 ---- */
#define SPI_CR2_RXDMAEN      (1U << 0)
#define SPI_CR2_TXDMAEN      (1U << 1)
#define SPI_CR2_SSOE         (1U << 2)
#define SPI_CR2_FRF          (1U << 4)               /* 0=Motorola,1=TI */
#define SPI_CR2_ERRIE        (1U << 5)
#define SPI_CR2_RXNEIE       (1U << 6)
#define SPI_CR2_TXEIE        (1U << 7)

/* ---- SR ---- */
#define SPI_SR_RXNE          (1U << 0)
#define SPI_SR_TXE           (1U << 1)
#define SPI_SR_CHSIDE        (1U << 2)
#define SPI_SR_UDR           (1U << 3)
#define SPI_SR_CRCERR        (1U << 4)
#define SPI_SR_MODF          (1U << 5)
#define SPI_SR_OVR           (1U << 6)
#define SPI_SR_BSY           (1U << 7)
#define SPI_SR_FRE           (1U << 8)

/* -------- I2C Peripheral Register Definition --------------------------------
 * Reference: RM0090 §27 (I2C) - Register map (CR1..TRISE)
 * F407 parts: CR1, CR2, OAR1, OAR2, DR, SR1, SR2, CCR, TRISE (no FLTR on F407).
 */
typedef struct
{
    __VO uint32_t CR1;    /*!< Control register 1,            Offset: 0x00 */
    __VO uint32_t CR2;    /*!< Control register 2,            Offset: 0x04 */
    __VO uint32_t OAR1;   /*!< Own address register 1,        Offset: 0x08 */
    __VO uint32_t OAR2;   /*!< Own address register 2,        Offset: 0x0C */
    __VO uint32_t DR;     /*!< Data register,                 Offset: 0x10 */
    __VO uint32_t SR1;    /*!< Status register 1,             Offset: 0x14 */
    __VO uint32_t SR2;    /*!< Status register 2,             Offset: 0x18 */
    __VO uint32_t CCR;    /*!< Clock control register,        Offset: 0x1C */
    __VO uint32_t TRISE;  /*!< TRISE register,                Offset: 0x20 */
} I2C_RegDef_t;

/* -------- I2Cx Peripheral Definitions -------------------------------------- */
#define I2C1   ((I2C_RegDef_t*) I2C1_BASE_ADDR)
#define I2C2   ((I2C_RegDef_t*) I2C2_BASE_ADDR)
#define I2C3   ((I2C_RegDef_t*) I2C3_BASE_ADDR)

/* ===================== I2C Bit Position Definitions ======================== */
/* ---- CR1 ---- */
#define I2C_CR1_PE             (1U << 0)
#define I2C_CR1_SMBUS          (1U << 1)
#define I2C_CR1_SMBTYPE        (1U << 3)
#define I2C_CR1_ENARP          (1U << 4)
#define I2C_CR1_ENPEC          (1U << 5)
#define I2C_CR1_ENGC           (1U << 6)
#define I2C_CR1_NOSTRETCH      (1U << 7)
#define I2C_CR1_START          (1U << 8)
#define I2C_CR1_STOP           (1U << 9)
#define I2C_CR1_ACK            (1U << 10)
#define I2C_CR1_POS            (1U << 11)
#define I2C_CR1_PEC            (1U << 12)
#define I2C_CR1_ALERT          (1U << 13)
#define I2C_CR1_SWRST          (1U << 15)

/* ---- CR2 ---- */
#define I2C_CR2_FREQ_Pos       0U
#define I2C_CR2_FREQ_Msk       (0x3FU << I2C_CR2_FREQ_Pos)   /* PCLK1 freq (MHz) */
#define I2C_CR2_ITERREN        (1U << 8)   /* Error IRQ enable */
#define I2C_CR2_ITEVTEN        (1U << 9)   /* Event IRQ enable */
#define I2C_CR2_ITBUFEN        (1U << 10)  /* Buffer IRQ enable */
#define I2C_CR2_DMAEN          (1U << 11)  /* DMA enable */
#define I2C_CR2_LAST           (1U << 12)  /* DMA last transfer */

/* ---- OAR1 ---- */
#define I2C_OAR1_ADD0          (1U << 0)      /* 7-bit: ignored, 10-bit: interface */
#define I2C_OAR1_ADD_Pos       1U             /* ADD[7:1] or ADD[9:8] in 10-bit */
#define I2C_OAR1_ADDMODE       (1U << 15)     /* 0:7-bit, 1:10-bit */

/* ---- OAR2 ---- */
#define I2C_OAR2_ENDUAL        (1U << 0)
#define I2C_OAR2_ADD2_Pos      1U             /* ADD2[7:1] */
#define I2C_OAR2_ADD2_Msk      (0x7FU << I2C_OAR2_ADD2_Pos)

/* ---- DR ---- */
#define I2C_DR_DR_Pos          0U
#define I2C_DR_DR_Msk          (0xFFU << I2C_DR_DR_Pos)

/* ---- SR1 ---- */
#define I2C_SR1_SB             (1U << 0)
#define I2C_SR1_ADDR           (1U << 1)
#define I2C_SR1_BTF            (1U << 2)
#define I2C_SR1_ADD10          (1U << 3)
#define I2C_SR1_STOPF          (1U << 4)
#define I2C_SR1_RXNE           (1U << 6)
#define I2C_SR1_TXE            (1U << 7)
#define I2C_SR1_BERR           (1U << 8)
#define I2C_SR1_ARLO           (1U << 9)
#define I2C_SR1_AF             (1U << 10)
#define I2C_SR1_OVR            (1U << 11)
#define I2C_SR1_PECERR         (1U << 12)
#define I2C_SR1_TIMEOUT        (1U << 14)
#define I2C_SR1_SMBALERT       (1U << 15)

/* ---- SR2 ---- */
#define I2C_SR2_MSL            (1U << 0)
#define I2C_SR2_BUSY           (1U << 1)
#define I2C_SR2_TRA            (1U << 2)
#define I2C_SR2_GENCALL        (1U << 4)
#define I2C_SR2_SMBDEFAULT     (1U << 5)
#define I2C_SR2_SMBHOST        (1U << 6)
#define I2C_SR2_DUALF          (1U << 7)
#define I2C_SR2_PEC_Pos        8U
#define I2C_SR2_PEC_Msk        (0xFFU << I2C_SR2_PEC_Pos)

/* ---- CCR ---- */
#define I2C_CCR_CCR_Pos        0U
#define I2C_CCR_CCR_Msk        (0x0FFFU << I2C_CCR_CCR_Pos)
#define I2C_CCR_DUTY           (1U << 14)
#define I2C_CCR_FS             (1U << 15)

/* ---- TRISE ---- */
#define I2C_TRISE_TRISE_Pos    0U
#define I2C_TRISE_TRISE_Msk    (0x3FU << I2C_TRISE_TRISE_Pos)


/* -------- EXTI Peripheral Register Definition --------
 * Reference: RM0090 §10 External interrupt/event controller (EXTI)
 * Base: EXTI_BASE_ADDR
 */
typedef struct
{
	__VO uint32_t IMR;    /*!< Interrupt mask register,                   Address offset: 0x00 */
	__VO uint32_t EMR;    /*!< Event mask register,                       Address offset: 0x04 */
	__VO uint32_t RTSR;   /*!< Rising trigger selection register,         Address offset: 0x08 */
	__VO uint32_t FTSR;   /*!< Falling trigger selection register,        Address offset: 0x0C */
	__VO uint32_t SWIER;  /*!< Software interrupt event register,         Address offset: 0x10 */
	__VO uint32_t PR;     /*!< Pending register,                          Address offset: 0x14 */
} EXTI_RegDef_t;

#define EXTI    ((EXTI_RegDef_t*)  EXTI_BASE_ADDR)

/* -------- SYSCFG Peripheral Register Definition --------
 * Reference: RM0090 §9 System configuration controller (SYSCFG)
 * Base: SYSCFG_BASE_ADDR
 */
typedef struct
{
	__VO uint32_t MEMRMP;   /*!< SYSCFG memory remap register,                     Address offset: 0x00 */
	__VO uint32_t PMC;      /*!< SYSCFG peripheral mode configuration register,    Address offset: 0x04 */
	__VO uint32_t EXTICR1;  /*!< External interrupt configuration register 1,      Address offset: 0x08 */
	__VO uint32_t EXTICR2;  /*!< External interrupt configuration register 2,      Address offset: 0x0C */
	__VO uint32_t EXTICR3;  /*!< External interrupt configuration register 3,      Address offset: 0x10 */
	__VO uint32_t EXTICR4;  /*!< External interrupt configuration register 4,      Address offset: 0x14 */
	uint32_t      RESERVED1[2]; /*!< Reserved,                                        Address offset: 0x18–0x1C */
	__VO uint32_t CMPCR;    /*!< Compensation cell control register,               Address offset: 0x20 */
} SYSCFG_RegDef_t;

#define SYSCFG ((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDR)

/* ===== SYSCFG clock (APB2) ===== */
#define SYSCFG_CLK_EN()   (RCC->APB2ENR |=  (1U << 14))
#define SYSCFG_CLK_DIS()  (RCC->APB2ENR &= ~(1U << 14))

/* -------- RCC Peripheral Register Definition --------
 *
 * Reference:
 * - RM0090 (STM32F405/407/415/417 Reference Manual), Rev. 17+
 *   §7 Reset and clock control (RCC)
 *   Table 21: RCC register map and reset values
 *
 * Base address: RCC_BASE_ADDR (from AHB1 domain, e.g., 0x40023800U)
 */

typedef struct
{
	__VO uint32_t CR;            /*!< Clock control register,                       Offset: 0x00 */
	__VO uint32_t PLLCFGR;       /*!< PLL configuration register,                   Offset: 0x04 */
	__VO uint32_t CFGR;          /*!< Clock configuration register,                 Offset: 0x08 */
	__VO uint32_t CIR;           /*!< Clock interrupt register,                     Offset: 0x0C */
	__VO uint32_t AHB1RSTR;      /*!< AHB1 peripheral reset register,               Offset: 0x10 */
	__VO uint32_t AHB2RSTR;      /*!< AHB2 peripheral reset register,               Offset: 0x14 */
	__VO uint32_t AHB3RSTR;      /*!< AHB3 peripheral reset register,               Offset: 0x18 */
	uint32_t      RESERVED0;     /*!< Reserved,                                     Offset: 0x1C */
	__VO uint32_t APB1RSTR;      /*!< APB1 peripheral reset register,               Offset: 0x20 */
	__VO uint32_t APB2RSTR;      /*!< APB2 peripheral reset register,               Offset: 0x24 */
	uint32_t      RESERVED1[2];  /*!< Reserved,                                     Offset: 0x28–0x2C */
	__VO uint32_t AHB1ENR;       /*!< AHB1 peripheral clock enable register,        Offset: 0x30 */
	__VO uint32_t AHB2ENR;       /*!< AHB2 peripheral clock enable register,        Offset: 0x34 */
	__VO uint32_t AHB3ENR;       /*!< AHB3 peripheral clock enable register,        Offset: 0x38 */
	uint32_t      RESERVED2;     /*!< Reserved,                                     Offset: 0x3C */
	__VO uint32_t APB1ENR;       /*!< APB1 peripheral clock enable register,        Offset: 0x40 */
	__VO uint32_t APB2ENR;       /*!< APB2 peripheral clock enable register,        Offset: 0x44 */
	uint32_t      RESERVED3[2];  /*!< Reserved,                                     Offset: 0x48–0x4C */
	__VO uint32_t AHB1LPENR;     /*!< AHB1 clock enable in low power mode,          Offset: 0x50 */
	__VO uint32_t AHB2LPENR;     /*!< AHB2 clock enable in low power mode,          Offset: 0x54 */
	__VO uint32_t AHB3LPENR;     /*!< AHB3 clock enable in low power mode,          Offset: 0x58 */
	uint32_t      RESERVED4;     /*!< Reserved,                                     Offset: 0x5C */
	__VO uint32_t APB1LPENR;     /*!< APB1 clock enable in low power mode,          Offset: 0x60 */
	__VO uint32_t APB2LPENR;     /*!< APB2 clock enable in low power mode,          Offset: 0x64 */
	uint32_t      RESERVED5[2];  /*!< Reserved,                                     Offset: 0x68–0x6C */
	__VO uint32_t BDCR;          /*!< Backup domain control register,               Offset: 0x70 */
	__VO uint32_t CSR;           /*!< Clock control & status register,              Offset: 0x74 */
	uint32_t      RESERVED6[2];  /*!< Reserved,                                     Offset: 0x78–0x7C */
	__VO uint32_t SSCGR;         /*!< Spread spectrum clock generation register,    Offset: 0x80 */
	__VO uint32_t PLLI2SCFGR;    /*!< PLLI2S configuration register,                Offset: 0x84 */
	/* Note: DCKCFGR/PLLSAICFGR exist on 42x/43x, not 407. Leave out for F407. */
} RCC_RegDef_t;

/* -------- Peripheral Definition -------- */
#define RCC     ((RCC_RegDef_t*) RCC_BASE_ADDR)

/* -------- RCC Clock Control Macros (short form: EN/DIS) --------
 * Reference: RM0090 (STM32F405/407/415/417 Reference Manual), §7 RCC
 */

/* ========== AHB1: GPIO ========== */
#define GPIOA_CLK_EN()    (RCC->AHB1ENR |=  (1U << 0))
#define GPIOB_CLK_EN()    (RCC->AHB1ENR |=  (1U << 1))
#define GPIOC_CLK_EN()    (RCC->AHB1ENR |=  (1U << 2))
#define GPIOD_CLK_EN()    (RCC->AHB1ENR |=  (1U << 3))
#define GPIOE_CLK_EN()    (RCC->AHB1ENR |=  (1U << 4))
#define GPIOF_CLK_EN()    (RCC->AHB1ENR |=  (1U << 5))
#define GPIOG_CLK_EN()    (RCC->AHB1ENR |=  (1U << 6))
#define GPIOH_CLK_EN()    (RCC->AHB1ENR |=  (1U << 7))
#define GPIOI_CLK_EN()    (RCC->AHB1ENR |=  (1U << 8))

#define GPIOA_CLK_DIS()   (RCC->AHB1ENR &= ~(1U << 0))
#define GPIOB_CLK_DIS()   (RCC->AHB1ENR &= ~(1U << 1))
#define GPIOC_CLK_DIS()   (RCC->AHB1ENR &= ~(1U << 2))
#define GPIOD_CLK_DIS()   (RCC->AHB1ENR &= ~(1U << 3))
#define GPIOE_CLK_DIS()   (RCC->AHB1ENR &= ~(1U << 4))
#define GPIOF_CLK_DIS()   (RCC->AHB1ENR &= ~(1U << 5))
#define GPIOG_CLK_DIS()   (RCC->AHB1ENR &= ~(1U << 6))
#define GPIOH_CLK_DIS()   (RCC->AHB1ENR &= ~(1U << 7))
#define GPIOI_CLK_DIS()   (RCC->AHB1ENR &= ~(1U << 8))

/* ========== AHB1: GPIO Reset Macros ==========
 * Use these to reset a GPIO port to its default state.
 * Each macro pulses the corresponding bit in RCC->AHB1RSTR:
 *   - Write 1: assert reset
 *   - Write 0: release reset
 * A dummy read-back ensures bus synchronization.
 */
#define GPIOA_REG_RESET()   do { RCC->AHB1RSTR |=  (1U << 0); (void)RCC->AHB1RSTR; RCC->AHB1RSTR &= ~(1U << 0); (void)RCC->AHB1RSTR; } while(0)
#define GPIOB_REG_RESET()   do { RCC->AHB1RSTR |=  (1U << 1); (void)RCC->AHB1RSTR; RCC->AHB1RSTR &= ~(1U << 1); (void)RCC->AHB1RSTR; } while(0)
#define GPIOC_REG_RESET()   do { RCC->AHB1RSTR |=  (1U << 2); (void)RCC->AHB1RSTR; RCC->AHB1RSTR &= ~(1U << 2); (void)RCC->AHB1RSTR; } while(0)
#define GPIOD_REG_RESET()   do { RCC->AHB1RSTR |=  (1U << 3); (void)RCC->AHB1RSTR; RCC->AHB1RSTR &= ~(1U << 3); (void)RCC->AHB1RSTR; } while(0)
#define GPIOE_REG_RESET()   do { RCC->AHB1RSTR |=  (1U << 4); (void)RCC->AHB1RSTR; RCC->AHB1RSTR &= ~(1U << 4); (void)RCC->AHB1RSTR; } while(0)
#define GPIOF_REG_RESET()   do { RCC->AHB1RSTR |=  (1U << 5); (void)RCC->AHB1RSTR; RCC->AHB1RSTR &= ~(1U << 5); (void)RCC->AHB1RSTR; } while(0)
#define GPIOG_REG_RESET()   do { RCC->AHB1RSTR |=  (1U << 6); (void)RCC->AHB1RSTR; RCC->AHB1RSTR &= ~(1U << 6); (void)RCC->AHB1RSTR; } while(0)
#define GPIOH_REG_RESET()   do { RCC->AHB1RSTR |=  (1U << 7); (void)RCC->AHB1RSTR; RCC->AHB1RSTR &= ~(1U << 7); (void)RCC->AHB1RSTR; } while(0)
#define GPIOI_REG_RESET()   do { RCC->AHB1RSTR |=  (1U << 8); (void)RCC->AHB1RSTR; RCC->AHB1RSTR &= ~(1U << 8); (void)RCC->AHB1RSTR; } while(0)

/* ========== APB: SPI Reset Macros ==========
 * Pulse the corresponding RCC reset bit:
 *   - Write 1: assert reset
 *   - Write 0: release reset
 * A dummy read-back ensures bus synchronization.
 */
#define SPI1_REG_RESET()  do { RCC->APB2RSTR |=  (1U << 12); (void)RCC->APB2RSTR; \
                               RCC->APB2RSTR &= ~(1U << 12); (void)RCC->APB2RSTR; } while(0)

#define SPI2_REG_RESET()  do { RCC->APB1RSTR |=  (1U << 14); (void)RCC->APB1RSTR; \
                               RCC->APB1RSTR &= ~(1U << 14); (void)RCC->APB1RSTR; } while(0)

#define SPI3_REG_RESET()  do { RCC->APB1RSTR |=  (1U << 15); (void)RCC->APB1RSTR; \
                               RCC->APB1RSTR &= ~(1U << 15); (void)RCC->APB1RSTR; } while(0)

/* -------- RCC: I2C Peripheral Reset (APB1RSTR) ----------------------------- */
#define I2C1_REG_RESET()  do { RCC->APB1RSTR |=  (1U << 21); (void)RCC->APB1RSTR; \
                               RCC->APB1RSTR &= ~(1U << 21); (void)RCC->APB1RSTR; } while(0)
#define I2C2_REG_RESET()  do { RCC->APB1RSTR |=  (1U << 22); (void)RCC->APB1RSTR; \
                               RCC->APB1RSTR &= ~(1U << 22); (void)RCC->APB1RSTR; } while(0)
#define I2C3_REG_RESET()  do { RCC->APB1RSTR |=  (1U << 23); (void)RCC->APB1RSTR; \
                               RCC->APB1RSTR &= ~(1U << 23); (void)RCC->APB1RSTR; } while(0)

/* ========== AHB1: DMA ========== */
#define DMA1_CLK_EN()     (RCC->AHB1ENR |=  (1U << 21))
#define DMA1_CLK_DIS()    (RCC->AHB1ENR &= ~(1U << 21))
#define DMA2_CLK_EN()     (RCC->AHB1ENR |=  (1U << 22))
#define DMA2_CLK_DIS()    (RCC->AHB1ENR &= ~(1U << 22))

/* ===== SPI clocks ===== */
/* APB2: SPI1 */
#define SPI1_CLK_EN()   (RCC->APB2ENR |=  (1U << 12))
#define SPI1_CLK_DIS()  (RCC->APB2ENR &= ~(1U << 12))

/* APB1: SPI2, SPI3 */
#define SPI2_CLK_EN()   (RCC->APB1ENR |=  (1U << 14))
#define SPI2_CLK_DIS()  (RCC->APB1ENR &= ~(1U << 14))
#define SPI3_CLK_EN()   (RCC->APB1ENR |=  (1U << 15))
#define SPI3_CLK_DIS()  (RCC->APB1ENR &= ~(1U << 15))

/* ===== I2C clocks (APB1) ===== */
#define I2C1_CLK_EN()   (RCC->APB1ENR |=  (1U << 21))
#define I2C1_CLK_DIS()  (RCC->APB1ENR &= ~(1U << 21))
#define I2C2_CLK_EN()   (RCC->APB1ENR |=  (1U << 22))
#define I2C2_CLK_DIS()  (RCC->APB1ENR &= ~(1U << 22))
#define I2C3_CLK_EN()   (RCC->APB1ENR |=  (1U << 23))
#define I2C3_CLK_DIS()  (RCC->APB1ENR &= ~(1U << 23))

/* ===== UART/USART clocks ===== */
/* APB1: USART2/3, UART4/5 */
#define USART2_CLK_EN() (RCC->APB1ENR |=  (1U << 17))
#define USART2_CLK_DIS() (RCC->APB1ENR &= ~(1U << 17))
#define USART3_CLK_EN() (RCC->APB1ENR |=  (1U << 18))
#define USART3_CLK_DIS() (RCC->APB1ENR &= ~(1U << 18))
#define UART4_CLK_EN()  (RCC->APB1ENR |=  (1U << 19))
#define UART4_CLK_DIS() (RCC->APB1ENR &= ~(1U << 19))
#define UART5_CLK_EN()  (RCC->APB1ENR |=  (1U << 20))
#define UART5_CLK_DIS() (RCC->APB1ENR &= ~(1U << 20))

/* APB2: USART1, USART6 */
#define USART1_CLK_EN() (RCC->APB2ENR |=  (1U << 4))
#define USART1_CLK_DIS() (RCC->APB2ENR &= ~(1U << 4))
#define USART6_CLK_EN() (RCC->APB2ENR |=  (1U << 5))
#define USART6_CLK_DIS() (RCC->APB2ENR &= ~(1U << 5))

/* -------- NVIC: Nested Vectored Interrupt Controller (Cortex-M4) --------
 * System Control Space (SCS) base: 0xE000E000
 *
 * We expose the commonly used register groups for enabling/disabling IRQs and
 * setting priorities. Each ISER/ICER/ISPR/ICPR/IABR register controls 32 IRQs.
 *
 * Reference:
 * - ARMv7-M Architecture / Cortex-M4 Devices
 * - STM32F4 (F407) uses external IRQ numbers consistent with CMSIS.
 */

/* --- Interrupt Set-Enable Registers (ISER), 0xE000E100 + 0x00/0x04/0x08 --- */
#define NVIC_ISER0      (*( (__VO uint32_t*)0xE000E100UL )) /* IRQ  0..31  */
#define NVIC_ISER1      (*( (__VO uint32_t*)0xE000E104UL )) /* IRQ 32..63  */
#define NVIC_ISER2      (*( (__VO uint32_t*)0xE000E108UL )) /* IRQ 64..95  */

/* --- Interrupt Clear-Enable Registers (ICER), 0xE000E180 + 0x00/0x04/0x08 --- */
#define NVIC_ICER0      (*( (__VO uint32_t*)0xE000E180UL )) /* IRQ  0..31  */
#define NVIC_ICER1      (*( (__VO uint32_t*)0xE000E184UL )) /* IRQ 32..63  */
#define NVIC_ICER2      (*( (__VO uint32_t*)0xE000E188UL )) /* IRQ 64..95  */

/* --- Interrupt Set-Pending Registers (ISPR), 0xE000E200 + 0x00/0x04/0x08 --- */
#define NVIC_ISPR0      (*( (__VO uint32_t*)0xE000E200UL )) /* IRQ  0..31  */
#define NVIC_ISPR1      (*( (__VO uint32_t*)0xE000E204UL )) /* IRQ 32..63  */
#define NVIC_ISPR2      (*( (__VO uint32_t*)0xE000E208UL )) /* IRQ 64..95  */

/* --- Interrupt Clear-Pending Registers (ICPR), 0xE000E280 + 0x00/0x04/0x08 - */
#define NVIC_ICPR0      (*( (__VO uint32_t*)0xE000E280UL )) /* IRQ  0..31  */
#define NVIC_ICPR1      (*( (__VO uint32_t*)0xE000E284UL )) /* IRQ 32..63  */
#define NVIC_ICPR2      (*( (__VO uint32_t*)0xE000E288UL )) /* IRQ 64..95  */

/* --- Interrupt Active Bit Registers (IABR), 0xE000E300 + 0x00/0x04/0x08 ---- */
#define NVIC_IABR0      (*( (__VO uint32_t*)0xE000E300UL )) /* IRQ  0..31  */
#define NVIC_IABR1      (*( (__VO uint32_t*)0xE000E304UL )) /* IRQ 32..63  */
#define NVIC_IABR2      (*( (__VO uint32_t*)0xE000E308UL )) /* IRQ 64..95  */

/* --- Interrupt Priority Registers (IPR), byte-addressable, 0xE000E400 ------ */
/* 1 byte per IRQ priority; STM32F4 implements the upper 4 bits of each byte. */
#define NVIC_IPR_BASE   (( __VO uint8_t*)0xE000E400UL)
#define NO_PR_BITS_IMPLEMENTED  (4U)   /* top 4 bits in each IPR byte are used */

/* --- Software Trigger Interrupt Register (STIR), 0xE000EF00 ---------------- */
#define NVIC_STIR       (*( (__VO uint32_t*)0xE000EF00UL )) /* write IRQn to pend */

/* -------- Selected IRQ numbers (external interrupts relevant to GPIO/EXTI) --
 */
#define EXTI0_IRQn          (6U)
#define EXTI1_IRQn          (7U)
#define EXTI2_IRQn          (8U)
#define EXTI3_IRQn          (9U)
#define EXTI4_IRQn          (10U)
#define EXTI9_5_IRQn        (23U)
#define EXTI15_10_IRQn      (40U)

/* I2C: separate Event and Error IRQ lines */
#define I2C1_EV_IRQn        (31U)
#define I2C1_ER_IRQn        (32U)
#define I2C2_EV_IRQn        (33U)
#define I2C2_ER_IRQn        (34U)
#define I2C3_EV_IRQn        (72U)
#define I2C3_ER_IRQn        (73U)

/* SPI */
#define SPI1_IRQn           (35U)
#define SPI2_IRQn           (36U)
#define SPI3_IRQn           (51U)

/* USART/UART */
#define USART1_IRQn         (37U)
#define USART2_IRQn         (38U)
#define USART3_IRQn         (39U)
#define UART4_IRQn          (52U)
#define UART5_IRQn          (53U)
#define USART6_IRQn         (71U)


#endif /* INC_STM32F407XX_H_ */
