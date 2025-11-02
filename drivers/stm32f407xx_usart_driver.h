/*
 * stm32f407xx_usart_driver.h
 *
 *  @file    stm32f407xx_usart_driver.h
 *  @brief   Minimal, allocation-free and callback-free USART/UART driver API for STM32F407.
 *
 *  ────────────────────────────────────────────────────────────────────────────
 *  OVERVIEW
 *  ────────────────────────────────────────────────────────────────────────────
 *  This header declares a lean USART driver that mirrors your GPIO/SPI driver's style:
 *  the application pre-populates a handle (device + config), and Init() resolves
 *  registers and programs the hardware. Supports blocking and interrupt I/O,
 *  multiple baud rates, parity, stop bits, and hardware flow control.
 *
 *  Design principles (matching GPIO/SPI):
 *   • Allocation-free; no dynamic memory.
 *   • Minimal state in a handle; no callbacks.
 *   • Safe bring-up order: program CR1/CR2/CR3/BRR before enabling UE.
 *
 *  Dependencies (from stm32f407xx.h):
 *   • USARTx base pointers/typedefs (USART_RegDef_t), RCC clock macros, NVIC IRQ nums.
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

/* ===== Basic types ======================================================== */

typedef enum {
    USART_DEVICE_USART1 = 0U,
    USART_DEVICE_USART2,
    USART_DEVICE_USART3,
    USART_DEVICE_UART4,
    USART_DEVICE_UART5,
    USART_DEVICE_USART6
} USART_Device_t;

typedef enum {
    USART_MODE_TX = 0U,        /* Transmit only */
    USART_MODE_RX,             /* Receive only */
    USART_MODE_TXRX            /* Transmit and receive */
} USART_Mode_t;

typedef enum {
    USART_WORDLEN_8BITS = 0U,
    USART_WORDLEN_9BITS
} USART_WordLen_t;

typedef enum {
    USART_PARITY_NONE = 0U,
    USART_PARITY_EVEN,
    USART_PARITY_ODD
} USART_Parity_t;

typedef enum {
    USART_STOPBITS_1 = 0U,     /* 1 stop bit */
    USART_STOPBITS_0_5,        /* 0.5 stop bits */
    USART_STOPBITS_2,          /* 2 stop bits */
    USART_STOPBITS_1_5         /* 1.5 stop bits */
} USART_StopBits_t;

typedef enum {
    USART_HW_FLOW_CTRL_NONE = 0U,
    USART_HW_FLOW_CTRL_RTS,
    USART_HW_FLOW_CTRL_CTS,
    USART_HW_FLOW_CTRL_RTS_CTS
} USART_HWFlowCtrl_t;

typedef enum {
    USART_OVER_16 = 0U,        /* Oversampling by 16 */
    USART_OVER_8               /* Oversampling by 8 */
} USART_Oversampling_t;

typedef enum {
    USART_OK = 0,
    USART_ERR_BUSY,
    USART_ERR_INVAL,
    USART_ERR_TIMEOUT,
    USART_ERR_PARITY,
    USART_ERR_FRAME,
    USART_ERR_NOISE,
    USART_ERR_OVERRUN
} USART_Status_t;

/**
 * @brief USART runtime configuration (mirrors the "config in handle" pattern used by GPIO/SPI).
 * @note  Application populates this struct before calling USART_Init().
 */
typedef struct {
    USART_Mode_t         mode;          /* TX only, RX only, or TX+RX */
    uint32_t             baudRate;      /* Desired baud rate (e.g., 9600, 115200) */
    USART_WordLen_t      wordLen;       /* 8 or 9 bits */
    USART_Parity_t       parity;        /* None, Even, or Odd */
    USART_StopBits_t     stopBits;      /* 1, 0.5, 2, or 1.5 stop bits */
    USART_HWFlowCtrl_t   hwFlowCtrl;    /* Hardware flow control mode */
    USART_Oversampling_t oversampling;  /* Oversampling by 8 or 16 */
} USART_Config_t;

/**
 * @brief USART handle — app fills .dev and .cfg, then calls USART_Init(&h).
 *        Init resolves .regs from .dev and programs the hardware.
 *
 * @note  This mirrors your GPIO/SPI pattern where the handle owns the config and
 *        Init() consumes it (no redundant "device" parameter).
 */
typedef struct {
    USART_RegDef_t   *regs;             /* Resolved USARTx register block (set by Init) */
    USART_Device_t    dev;              /* Which USART instance (set by application) */

    USART_Config_t    cfg;              /* Active configuration (set by application) */

    /* Interrupt (IT) state */
    const uint8_t    *pTx;              /* TX buffer (non-blocking) */
    uint16_t          txRemaining;      /* TX bytes remaining */
    uint8_t          *pRx;              /* RX buffer (non-blocking) */
    uint16_t          rxRemaining;      /* RX bytes remaining */

    volatile uint16_t flags;            /* USART_FLAG_* bitmask */
} USART_Handle_t;

/* Flags (bitmask) — polled by application in IT mode */
#define USART_FLAG_TX_BUSY     (1U << 0)
#define USART_FLAG_RX_BUSY     (1U << 1)
#define USART_FLAG_TX_DONE     (1U << 2)
#define USART_FLAG_RX_DONE     (1U << 3)
#define USART_FLAG_PE          (1U << 8)   /* Parity error */
#define USART_FLAG_FE          (1U << 9)   /* Framing error */
#define USART_FLAG_NE          (1U << 10)  /* Noise error */
#define USART_FLAG_ORE         (1U << 11)  /* Overrun error */
#define USART_FLAG_IDLE        (1U << 12)  /* IDLE line detected */


/* =============================================================================
 * Peripheral Clock setup (RCC)
 * =============================================================================
 */

/**
 * @brief  Enable/Disable APB clock for a USART instance.
 * @param  pUSARTx   Target USART register block (USART1..6, UART4..5).
 * @param  EnorDi    ENABLE or DISABLE.
 * @note   Keeps parity with GPIO_PeriClockControl signature.
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);


/* =============================================================================
 * Init and De-init
 * =============================================================================
 */

/**
 * @brief  Initialize USART hardware registers for the handle's .dev/.cfg
 *         (does not set UE=1).
 * @param  h  Handle with h->dev and h->cfg pre-populated by the application.
 * @return USART_OK or an error code.
 *
 * Safe order (analogous to GPIO/SPI_Init discipline):
 *   1) Enable APB clock for USARTx via USART_PeriClockControl().
 *   2) Program BRR: baud rate (depends on PCLK and OVER8).
 *   3) Program CR1: M, PCE, PS, TE, RE, OVER8.
 *   4) Program CR2: STOP bits.
 *   5) Program CR3: RTSE, CTSE (hardware flow control).
 *   6) Clear sticky flags by reading SR then DR.
 *   7) Leave UE=0; enable later via USART_PeripheralControl() when pins are ready.
 *
 * @note  No redundant "device" argument; the device is taken from h->dev.
 */
USART_Status_t USART_Init(USART_Handle_t *h);

/**
 * @brief  De-initialize (reset) a USART peripheral to its reset state.
 * @param  pUSARTx  Target USART register block (USART1..6, UART4..5).
 * @note   Matches GPIO/SPI_DeInit() style (peripheral-wide reset via RCC RSTR).
 */
void USART_DeInit(USART_RegDef_t *pUSARTx);


/* =============================================================================
 * Data transfer (blocking)
 * =============================================================================
 */

/**
 * @brief  Blocking transmit of @p len bytes.
 * @param  h     Handle to the configured USART instance.
 * @param  pTx   Pointer to transmit buffer.
 * @param  len   Number of bytes to transmit.
 * @return USART_OK or an error code.
 */
USART_Status_t USART_SendData(USART_Handle_t *h, const uint8_t *pTx, uint16_t len);

/**
 * @brief  Blocking receive of @p len bytes.
 * @param  h     Handle to the configured USART instance.
 * @param  pRx   Pointer to receive buffer.
 * @param  len   Number of bytes to receive.
 * @return USART_OK or an error code.
 */
USART_Status_t USART_ReceiveData(USART_Handle_t *h, uint8_t *pRx, uint16_t len);


/**
 * @brief  Enable or disable the USART peripheral (CR1.UE).
 * @param  h       Handle to the configured USART instance.
 * @param  EnorDi  ENABLE to activate the peripheral, DISABLE to deactivate.
 *
 * @note   This function controls the USART Enable (UE) bit in CR1.
 *         UE must be set to 1 after USART_Init() to begin USART communication.
 *
 * @warning When disabling (EnorDi = DISABLE), this function waits for the
 *          transmission to complete (TC=1) before clearing UE to avoid truncating
 *          any ongoing transfer. This may block if a transfer is in progress.
 *
 * Safe usage sequence:
 *   1) USART_Init(h)                      // Configure but leave UE=0
 *   2) Configure GPIO pins (TX/RX/etc.)   // Prepare external connections
 *   3) USART_PeripheralControl(h, ENABLE) // Activate USART peripheral
 *   4) Perform USART transfers            // Send/receive data
 *   5) USART_PeripheralControl(h, DISABLE)// Deactivate when done (optional)
 *
 * @note   Most configuration changes to CR1/CR2/CR3 require UE=0. Use this function
 *         to safely disable the peripheral before reconfiguration, then re-enable.
 */
void USART_PeripheralControl(USART_Handle_t *h, uint8_t EnorDi);


/* =============================================================================
 * Data transfer (interrupt / non-blocking)
 * =============================================================================
 */

/**
 * @brief  Start non-blocking transmit (enables TXEIE).
 * @note   Sets USART_FLAG_TX_BUSY; ISR drains the buffer then sets USART_FLAG_TX_DONE.
 */
USART_Status_t USART_SendDataIT(USART_Handle_t *h, const uint8_t *pTx, uint16_t len);

/**
 * @brief  Start non-blocking receive (enables RXNEIE).
 * @note   Sets USART_FLAG_RX_BUSY; ISR fills the buffer then sets USART_FLAG_RX_DONE.
 */
USART_Status_t USART_ReceiveDataIT(USART_Handle_t *h, uint8_t *pRx, uint16_t len);

/**
 * @brief  USART ISR helper: handles TXE, RXNE, TC, IDLE, and ERR (PE/FE/NE/ORE).
 * @note   Call from USARTx_IRQHandler(). Updates flags and buffer cursors.
 */
void USART_IRQHandling(USART_Handle_t *h);


/* =============================================================================
 * IRQ Configuration and ISR registration (NVIC)
 * =============================================================================
 */

/**
 * @brief  Enable/Disable an NVIC IRQ line for USART.
 * @param  IRQNumber  USART1_IRQn, USART2_IRQn, ..., USART6_IRQn.
 * @param  EnorDi     ENABLE or DISABLE.
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief  Set NVIC priority for a USART IRQ line.
 * @param  IRQNumber   Target IRQ line.
 * @param  IRQPriority Encoded priority (upper NO_PR_BITS_IMPLEMENTED bits used).
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/* =============================================================================
 * Utilities & runtime control
 * =============================================================================
 */

/** @brief Returns SR.TC (1 if transmission complete). */
uint8_t USART_IsTxComplete(USART_Handle_t *h);

/** @brief Returns SR.RXNE (1 if data available). */
uint8_t USART_IsDataAvailable(USART_Handle_t *h);

/** @brief Clear error flags (read SR then DR to clear ORE/PE/FE/NE). */
void USART_ClearErrors(USART_Handle_t *h);

/**
 * @brief  Set baud rate at runtime.
 * @note   Sequence: wait TC=1 → UE=0 → rewrite BRR → UE=1 (if previously enabled).
 * @param  h         Target handle.
 * @param  baudRate  New baud rate (e.g., 115200).
 */
USART_Status_t USART_SetBaudRate(USART_Handle_t *h, uint32_t baudRate);


/* ===== Inline flag helpers =============================================== */
static inline uint16_t USART_GetFlags(const USART_Handle_t *h) { return h->flags; }
static inline void     USART_ClearFlags(USART_Handle_t *h, uint16_t mask) { h->flags &= (uint16_t)~mask; }

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
