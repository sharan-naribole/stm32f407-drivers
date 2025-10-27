/*
 * stm32f407xx_spi_driver.h
 *
 *  @file    stm32f407xx_spi_driver.h
 *  @brief   Minimal, allocation-free and callback-free SPI driver API for STM32F407.
 *
 *  ────────────────────────────────────────────────────────────────────────────
 *  OVERVIEW
 *  ────────────────────────────────────────────────────────────────────────────
 *  This header declares a lean SPI driver that mirrors your GPIO driver’s style:
 *  the application pre-populates a handle (device + config), and Init() resolves
 *  registers and programs the hardware. Supports blocking and interrupt I/O,
 *  master/slave, full/half/simplex, 8/16-bit frames, and flexible NSS handling.
 *
 *  Design principles (matching GPIO):
 *   • Allocation-free; no dynamic memory.
 *   • Minimal state in a handle; no callbacks.
 *   • Safe bring-up order: program CR1/CR2 before enabling SPE; manage CS via GPIO.
 *
 *  Dependencies (from stm32f407xx.h):
 *   • SPIx base pointers/typedefs (SPI_RegDef_t), RCC clock macros, NVIC IRQ nums.
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/* ===== Basic types ======================================================== */

typedef enum {
    SPI_DEVICE_SPI1 = 0U,
    SPI_DEVICE_SPI2,
    SPI_DEVICE_SPI3
} SPI_Device_t;

typedef enum { SPI_MODE_MASTER = 0U, SPI_MODE_SLAVE } SPI_Mode_t;

typedef enum {
    SPI_BUS_FULL_DUPLEX = 0U,  /* 2-line */
    SPI_BUS_SIMPLEX_RX,        /* 1-line, receive-only (CR1.RXONLY) */
    SPI_BUS_HALF_DUPLEX        /* 1-line bidirectional (CR1.BIDIMODE) */
} SPI_BusConfig_t;

typedef enum { SPI_DFF_8BIT = 0U, SPI_DFF_16BIT } SPI_DataSize_t;

typedef enum { SPI_CPOL_0 = 0U, SPI_CPOL_1 } SPI_CPOL_t;
typedef enum { SPI_CPHA_0 = 0U, SPI_CPHA_1 } SPI_CPHA_t;

typedef enum { SPI_FIRSTBIT_MSB = 0U, SPI_FIRSTBIT_LSB } SPI_FirstBit_t;

typedef enum {
    SPI_NSS_SOFTWARE = 0U,   /* SSM=1 (use SSI); app may also drive a GPIO CS */
    SPI_NSS_HARD_OUTPUT,     /* Master drives NSS via SSOE (hardware) */
    SPI_NSS_HARD_INPUT       /* Slave samples NSS pin (hardware) */
} SPI_NSS_Mode_t;

typedef enum {
    SPI_BR_DIV2 = 0U, SPI_BR_DIV4, SPI_BR_DIV8,  SPI_BR_DIV16,
    SPI_BR_DIV32,     SPI_BR_DIV64, SPI_BR_DIV128, SPI_BR_DIV256
} SPI_BaudDiv_t;

typedef enum {
    SPI_OK = 0,
    SPI_ERR_BUSY,
    SPI_ERR_INVAL,
    SPI_ERR_OVR,
    SPI_ERR_MODF,
    SPI_ERR_CRC,
    SPI_ERR_FRAME
} SPI_Status_t;

/**
 * @brief SPI runtime configuration (mirrors the “config in handle” pattern used by GPIO).
 * @note  Application populates this struct before calling SPI_Init().
 */
typedef struct {
    SPI_Mode_t       mode;          /* Master/Slave */
    SPI_BusConfig_t  bus;           /* Full, Simplex-RX, Half-duplex */
    SPI_DataSize_t   datasize;      /* 8/16-bit */
    SPI_CPOL_t       cpol;
    SPI_CPHA_t       cpha;
    SPI_FirstBit_t   firstBit;      /* MSB/LSB first */
    SPI_BaudDiv_t    baudDiv;       /* prescaler (master only) */
    SPI_NSS_Mode_t   nss;           /* software/hardware NSS */
    uint8_t          tiMode;        /* 0 = Motorola, 1 = TI frame (CR2.FRF) */
    uint8_t          crcEnable;     /* 0/1 */
    uint16_t         crcPolynomial; /* e.g., 7 */
} SPI_Config_t;

/**
 * @brief SPI handle — app fills .dev and .cfg, then calls SPI_Init(&h).
 *        Init resolves .regs from .dev and programs the hardware.
 *
 * @note  This mirrors your GPIO pattern where the handle owns the config and
 *        Init() consumes it (no redundant “device” parameter). See your GPIO
 *        driver for reference.
 */
typedef struct {
    SPI_RegDef_t   *regs;           /* Resolved SPIx register block (set by Init) */
    SPI_Device_t    dev;            /* Which SPI instance (set by application) */

    SPI_Config_t    cfg;            /* Active configuration (set by application) */

    /* Interrupt (IT) state */
    const uint8_t  *pTx;            /* TX buffer (non-blocking) */
    uint16_t        txRemaining;    /* TX elements remaining (bytes/half-words by DFF) */
    uint8_t        *pRx;            /* RX buffer (non-blocking) */
    uint16_t        rxRemaining;    /* RX elements remaining */

    volatile uint16_t flags;        /* SPI_FLAG_* bitmask */
} SPI_Handle_t;

/* Flags (bitmask) — polled by application in IT mode */
#define SPI_FLAG_TX_BUSY   (1U << 0)
#define SPI_FLAG_RX_BUSY   (1U << 1)
#define SPI_FLAG_TX_DONE   (1U << 2)
#define SPI_FLAG_RX_DONE   (1U << 3)
#define SPI_FLAG_OVR       (1U << 8)   /* Overrun */
#define SPI_FLAG_MODF      (1U << 9)   /* Mode fault */
#define SPI_FLAG_CRCERR    (1U << 10)
#define SPI_FLAG_FRE       (1U << 11)  /* TI frame error */


/* =============================================================================
 * Peripheral Clock setup (RCC)
 * =============================================================================
 */

/**
 * @brief  Enable/Disable APB clock for a SPI instance.
 * @param  pSPIx   Target SPI register block (SPI1..SPI3).
 * @param  EnorDi  ENABLE or DISABLE.
 * @note   Keeps parity with GPIO_PeriClockControl signature.
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/* =============================================================================
 * Init and De-init
 * =============================================================================
 */

/**
 * @brief  Initialize SPI hardware registers for the handle’s .dev/.cfg
 *         (does not set SPE=1).
 * @param  h  Handle with h->dev and h->cfg pre-populated by the application.
 * @return SPI_OK or an error code.
 *
 * Safe order (analogous to GPIO_Init discipline):
 *   1) Enable APB clock for SPIx via SPI_PeriClockControl().
 *   2) Program CR1: MSTR, BR, CPOL/CPHA, DFF, SSM/SSI, RXONLY, BIDIMODE.
 *   3) Program CR2: SSOE (if HW NSS master), FRF, IRQ enables (left off by default).
 *   4) Clear sticky flags by reading SR then DR.
 *   5) Leave SPE=0; enable later via SPI_PeripheralControl() when pins/CS are ready.
 *
 * @note  No redundant “device” argument; the device is taken from h->dev.
 */
SPI_Status_t SPI_Init(SPI_Handle_t *h);

/**
 * @brief  De-initialize (reset) a SPI peripheral to its reset state.
 * @param  pSPIx  Target SPI register block (SPI1..SPI3).
 * @note   Matches GPIO_DeInit() style (port-wide reset via RCC RSTR).
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/* =============================================================================
 * Data transfer (blocking)
 * =============================================================================
 */

/**
 * @brief  Blocking transmit of @p len elements (8/16-bit per DFF).
 */
SPI_Status_t SPI_SendData(SPI_Handle_t *h, const void *pTx, uint16_t len);

/**
 * @brief  Blocking receive of @p len elements.
 */
SPI_Status_t SPI_ReceiveData(SPI_Handle_t *h, void *pRx, uint16_t len);

/**
 * @brief  Blocking full-duplex transfer: send and receive @p len elements.
 */
SPI_Status_t SPI_SendReceiveData(SPI_Handle_t *h, const void *pTx, void *pRx, uint16_t len);


/**
 * @brief  Enable or disable the SPI peripheral (CR1.SPE).
 * @param  h       Handle to the configured SPI instance.
 * @param  EnorDi  ENABLE to activate the peripheral, DISABLE to deactivate.
 *
 * @note   This function controls the SPI Peripheral Enable (SPE) bit in CR1.
 *         SPE must be set to 1 after SPI_Init() to begin SPI communication.
 *
 * @warning When disabling (EnorDi = DISABLE), this function waits for the bus
 *          to become idle (BSY=0) before clearing SPE to avoid truncating any
 *          ongoing transfer. This may block if a transfer is in progress.
 *
 * Safe usage sequence:
 *   1) SPI_Init(h)                      // Configure but leave SPE=0
 *   2) Configure GPIO pins and CS lines // Prepare external connections
 *   3) SPI_PeripheralControl(h, ENABLE) // Activate SPI peripheral
 *   4) Perform SPI transfers            // Send/receive data
 *   5) SPI_PeripheralControl(h, DISABLE)// Deactivate when done (optional)
 *
 * @note   Most configuration changes to CR1/CR2 require SPE=0. Use this function
 *         to safely disable the peripheral before reconfiguration, then re-enable.
 */
void SPI_PeripheralControl(SPI_Handle_t *h, uint8_t EnorDi);


/* =============================================================================
 * Data transfer (interrupt / non-blocking)
 * =============================================================================
 */

/**
 * @brief  Start non-blocking transmit (enables TXEIE).
 * @note   Sets SPI_FLAG_TX_BUSY; ISR drains the buffer then sets SPI_FLAG_TX_DONE.
 */
SPI_Status_t SPI_SendDataIT(SPI_Handle_t *h, const void *pTx, uint16_t len);

/**
 * @brief  Start non-blocking receive (enables RXNEIE).
 * @note   Sets SPI_FLAG_RX_BUSY; ISR fills the buffer then sets SPI_FLAG_RX_DONE.
 */
SPI_Status_t SPI_ReceiveDataIT(SPI_Handle_t *h, void *pRx, uint16_t len);

/**
 * @brief  SPI ISR helper: handles TXE, RXNE, and ERR (OVR/MODF/CRCERR/FRE).
 * @note   Call from SPIx_IRQHandler(). Updates flags and buffer cursors.
 */
void SPI_IRQHandling(SPI_Handle_t *h);


/* =============================================================================
 * IRQ Configuration and ISR registration (NVIC)
 * =============================================================================
 */

/**
 * @brief  Enable/Disable an NVIC IRQ line for SPI.
 * @param  IRQNumber  SPI1_IRQn, SPI2_IRQn, SPI3_IRQn.
 * @param  EnorDi     ENABLE or DISABLE.
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief  Set NVIC priority for a SPI IRQ line.
 * @param  IRQNumber   Target IRQ line.
 * @param  IRQPriority Encoded priority (upper NO_PR_BITS_IMPLEMENTED bits used).
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/* =============================================================================
 * Utilities & runtime control
 * =============================================================================
 */

/** @brief Returns SR.BSY (1 if busy). */
uint8_t SPI_IsBusy(SPI_Handle_t *h);

/** @brief Clear OVR fault condition (read DR then SR when not busy). */
void SPI_ClearOVR(SPI_Handle_t *h);

/**
 * @brief  Reprogram configuration at runtime (role/NSS/mode/etc.).
 * @note   Sequence: wait BSY=0 → SPE=0 → rewrite CR1/CR2 → SPE=1 (if previously enabled).
 * @param  h     Target handle.
 * @param  cfg   New configuration to apply (also copied into h->cfg).
 */
SPI_Status_t SPI_Reconfigure(SPI_Handle_t *h, const SPI_Config_t *cfg);


/* ===== Inline flag helpers =============================================== */
static inline uint16_t SPI_GetFlags(const SPI_Handle_t *h) { return h->flags; }
static inline void     SPI_ClearFlags(SPI_Handle_t *h, uint16_t mask) { h->flags &= (uint16_t)~mask; }

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
