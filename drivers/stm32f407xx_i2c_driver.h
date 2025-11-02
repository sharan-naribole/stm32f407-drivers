/*
 * stm32f407xx_i2c_driver.h
 *
 *  @file    stm32f407xx_i2c_driver.h
 *  @brief   Allocation-free I2C driver API for STM32F407 (matches GPIO/SPI style).
 *
 *  ────────────────────────────────────────────────────────────────────────────
 *  OVERVIEW
 *  ────────────────────────────────────────────────────────────────────────────
 *  This header declares an I2C driver that follows the same minimal design as
 *  the GPIO and SPI drivers: the application pre-populates a handle (device +
 *  config), and Init() resolves registers and programs the hardware.
 *
 *  Supports:
 *   • Master/Slave modes
 *   • 7-bit and 10-bit addressing
 *   • Standard mode (Sm, up to 100 kHz) and Fast mode (Fm, up to 400 kHz)
 *   • Blocking and interrupt-driven I/O
 *   • Repeated start, ACK/NACK control
 *   • Clock stretching enable/disable
 *
 *  Design principles:
 *   • Allocation-free; no dynamic memory.
 *   • Minimal state in a handle; no callbacks.
 *   • Safe bring-up: configure before enabling PE bit.
 *
 *  Dependencies (from stm32f407xx.h):
 *   • I2Cx base pointers/typedefs (I2C_RegDef_t), RCC clock macros, NVIC IRQ nums.
 *
 *  Reference: RM0090 Rev 21, §27 (Inter-integrated circuit (I2C) interface)
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/* ===== Configuration Enums ================================================ */

typedef enum {
    I2C_DEVICE_I2C1 = 0U,
    I2C_DEVICE_I2C2,
    I2C_DEVICE_I2C3
} I2C_Device_t;

/**
 * @brief I2C speed modes
 * @note  Sm = Standard mode (up to 100 kHz), Fm = Fast mode (up to 400 kHz)
 */
typedef enum {
    I2C_SPEED_SM   = 100000U,   /* Standard mode: 100 kHz */
    I2C_SPEED_FM2K = 200000U,   /* Fast mode: 200 kHz */
    I2C_SPEED_FM4K = 400000U    /* Fast mode: 400 kHz */
} I2C_Speed_t;

/**
 * @brief Addressing mode: 7-bit or 10-bit
 */
typedef enum {
    I2C_ADDR_7BIT  = 0U,
    I2C_ADDR_10BIT = 1U
} I2C_AddrMode_t;

/**
 * @brief ACK control
 */
typedef enum {
    I2C_ACK_DISABLE = 0U,
    I2C_ACK_ENABLE  = 1U
} I2C_AckControl_t;

/**
 * @brief Fast mode duty cycle (only when speed = FM)
 * @note  Duty 2: Tlow/Thigh = 2
 *        Duty 16_9: Tlow/Thigh = 16/9 (allows higher capacitive loads)
 */
typedef enum {
    I2C_FM_DUTY_2    = 0U,
    I2C_FM_DUTY_16_9 = 1U
} I2C_FmDutyCycle_t;

/**
 * @brief Return status codes
 */
typedef enum {
    I2C_OK = 0,
    I2C_ERR_BUSY,        /* Bus busy or handle busy */
    I2C_ERR_INVAL,       /* Invalid parameter */
    I2C_ERR_BERR,        /* Bus error (misplaced Start/Stop) */
    I2C_ERR_ARLO,        /* Arbitration lost */
    I2C_ERR_AF,          /* Acknowledge failure */
    I2C_ERR_OVR,         /* Overrun/underrun */
    I2C_ERR_TIMEOUT      /* Timeout or Tlow error (SMBus) */
} I2C_Status_t;

/**
 * @brief I2C application states (for interrupt-driven operations)
 */
typedef enum {
    I2C_STATE_READY = 0U,
    I2C_STATE_BUSY_TX,
    I2C_STATE_BUSY_RX
} I2C_State_t;

/**
 * @brief I2C events for application callbacks (optional)
 */
typedef enum {
    I2C_EVENT_TX_CMPLT = 0U,
    I2C_EVENT_RX_CMPLT,
    I2C_EVENT_STOP,
    I2C_ERROR_BERR,
    I2C_ERROR_ARLO,
    I2C_ERROR_AF,
    I2C_ERROR_OVR,
    I2C_ERROR_TIMEOUT
} I2C_Event_t;

/* ===== Configuration Structure ============================================ */

/**
 * @brief I2C runtime configuration
 * @note  Application populates this before calling I2C_Init().
 */
typedef struct {
    uint32_t            sclSpeed;       /* SCL frequency in Hz (I2C_SPEED_*) */
    uint8_t             ownAddress;     /* Own 7-bit slave address (bits [7:1]) */
    I2C_AddrMode_t      addrMode;       /* 7-bit or 10-bit addressing */
    I2C_AckControl_t    ackControl;     /* ACK enable/disable */
    I2C_FmDutyCycle_t   fmDutyCycle;    /* Fast mode duty cycle (2 or 16/9) */
} I2C_Config_t;

/* ===== Handle Structure =================================================== */

/**
 * @brief I2C handle — app fills .dev and .cfg, then calls I2C_Init(&h).
 * @note  Mirrors GPIO/SPI pattern: handle owns config and state.
 */
typedef struct {
    I2C_RegDef_t   *pI2Cx;          /* Resolved I2Cx register block (set by Init) */
    I2C_Device_t    dev;            /* Which I2C instance (set by application) */

    I2C_Config_t    cfg;            /* Active configuration (set by application) */

    /* Interrupt (IT) state for Master mode */
    uint8_t        *pTxBuffer;      /* TX buffer pointer */
    uint8_t        *pRxBuffer;      /* RX buffer pointer */
    uint32_t        txLen;          /* TX length in bytes */
    uint32_t        rxLen;          /* RX length in bytes */
    uint32_t        txCount;        /* TX bytes sent so far */
    uint32_t        rxCount;        /* RX bytes received so far */

    I2C_State_t     state;          /* Current transaction state */
    uint8_t         devAddr;        /* Slave device address (7-bit or 10-bit) */
    uint32_t        rxSize;         /* Total RX size (for N-byte reception logic) */
    uint8_t         repeatedStart;  /* SR = repeated start (1) or Stop (0) */
} I2C_Handle_t;

/* ===== SR1 Flag Bits (for status checking) ================================ */
#define I2C_FLAG_SB         I2C_SR1_SB          /* Start bit (Master) */
#define I2C_FLAG_ADDR       I2C_SR1_ADDR        /* Address sent/matched */
#define I2C_FLAG_BTF        I2C_SR1_BTF         /* Byte transfer finished */
#define I2C_FLAG_ADD10      I2C_SR1_ADD10       /* 10-bit header sent */
#define I2C_FLAG_STOPF      I2C_SR1_STOPF       /* Stop detected (Slave) */
#define I2C_FLAG_RXNE       I2C_SR1_RXNE        /* RX buffer not empty */
#define I2C_FLAG_TXE        I2C_SR1_TXE         /* TX buffer empty */
#define I2C_FLAG_BERR       I2C_SR1_BERR        /* Bus error */
#define I2C_FLAG_ARLO       I2C_SR1_ARLO        /* Arbitration lost */
#define I2C_FLAG_AF         I2C_SR1_AF          /* Acknowledge failure */
#define I2C_FLAG_OVR        I2C_SR1_OVR         /* Overrun/Underrun */
#define I2C_FLAG_TIMEOUT    I2C_SR1_TIMEOUT     /* Timeout error */

/* ===== SR2 Flag Bits ====================================================== */
#define I2C_FLAG_MSL        I2C_SR2_MSL         /* Master/Slave mode */
#define I2C_FLAG_BUSY       I2C_SR2_BUSY        /* Bus busy */
#define I2C_FLAG_TRA        I2C_SR2_TRA         /* Transmitter/Receiver */

/* Repeated Start options */
#define I2C_DISABLE_SR      0U
#define I2C_ENABLE_SR       1U

/* =============================================================================
 * Peripheral Clock Setup (RCC)
 * =============================================================================
 */

/**
 * @brief  Enable/Disable APB1 clock for an I2C instance.
 * @param  pI2Cx   Target I2C register block (I2C1..I2C3).
 * @param  EnorDi  ENABLE or DISABLE.
 * @note   Matches GPIO_PeriClockControl signature.
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/* =============================================================================
 * Init and De-init
 * =============================================================================
 */

/**
 * @brief  Initialize I2C hardware registers for the handle's .dev/.cfg
 *         (does not set PE=1).
 * @param  pHandle  Handle with .dev and .cfg pre-populated by application.
 * @return I2C_OK or an error code.
 *
 * Safe initialization order:
 *   1) Enable APB1 clock via I2C_PeriClockControl().
 *   2) Program CR2.FREQ with APB1 frequency (MHz).
 *   3) Program own address in OAR1 (7-bit or 10-bit).
 *   4) Calculate and program CCR for target SCL speed.
 *   5) Program TRISE register.
 *   6) Enable ACK (CR1.ACK) if configured.
 *   7) Leave PE=0; enable later via I2C_PeripheralControl() when pins/CS are ready.
 *
 * @note  GPIO pins (SDA/SCL) must be configured BEFORE calling this function.
 *        Configure as: AF4, Open-drain, Pull-up, High speed.
 *        After I2C_Init(), call I2C_PeripheralControl(&h, ENABLE) to activate.
 */
I2C_Status_t I2C_Init(I2C_Handle_t *pHandle);

/**
 * @brief  De-initialize (reset) an I2C peripheral to its reset state.
 * @param  pI2Cx  Target I2C register block (I2C1..I2C3).
 * @note   Uses RCC APB1RSTR to pulse the reset bit.
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/* =============================================================================
 * Master Data Transfer (Blocking)
 * =============================================================================
 */

/**
 * @brief  Master transmit (blocking).
 * @param  pHandle      I2C handle (must be initialized).
 * @param  pTxBuffer    Pointer to data to send.
 * @param  len          Number of bytes to send.
 * @param  slaveAddr    7-bit or 10-bit slave address (format depends on cfg).
 * @param  repeatedStart I2C_ENABLE_SR or I2C_DISABLE_SR.
 * @return I2C_OK or error code.
 *
 * Procedure (7-bit addressing, standard):
 *   1) Generate START condition.
 *   2) Send slave address (write mode: R/W=0).
 *   3) Wait for ADDR cleared by reading SR1 then SR2.
 *   4) Transmit len bytes (wait TxE before each write to DR).
 *   5) Wait for BTF (ensures last byte shifted out).
 *   6) Generate STOP (or repeated START if requested).
 */
I2C_Status_t I2C_MasterSendData(I2C_Handle_t *pHandle, uint8_t *pTxBuffer,
                                 uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart);

/**
 * @brief  Master receive (blocking).
 * @param  pHandle      I2C handle.
 * @param  pRxBuffer    Pointer to receive buffer.
 * @param  len          Number of bytes to receive.
 * @param  slaveAddr    Slave address.
 * @param  repeatedStart I2C_ENABLE_SR or I2C_DISABLE_SR.
 * @return I2C_OK or error code.
 *
 * Special cases (RM0090 §27.3.3):
 *   - 1 byte: ACK=0 during EV6, STOP after EV6.
 *   - 2 bytes: ACK=0, POS=1 after ADDR cleared, wait BTF, then STOP, read both.
 *   - N>2 bytes: ACK=1, read N-2 normally, then special handling for last 2 bytes.
 */
I2C_Status_t I2C_MasterReceiveData(I2C_Handle_t *pHandle, uint8_t *pRxBuffer,
                                    uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart);

/* =============================================================================
 * Slave Data Transfer (Blocking)
 * =============================================================================
 */

/**
 * @brief  Slave transmit (blocking) — waits for master to read data.
 * @param  pHandle    I2C handle.
 * @param  pTxBuffer  Data to send.
 * @param  len        Number of bytes.
 * @return I2C_OK or error code.
 * @note   Slave stretches SCL after ADDR until application writes first byte.
 */
I2C_Status_t I2C_SlaveSendData(I2C_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t len);

/**
 * @brief  Slave receive (blocking) — waits for master to send data.
 * @param  pHandle    I2C handle.
 * @param  pRxBuffer  Receive buffer.
 * @param  len        Number of bytes.
 * @return I2C_OK or error code.
 */
I2C_Status_t I2C_SlaveReceiveData(I2C_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t len);

/* =============================================================================
 * Master Data Transfer (Interrupt-driven)
 * =============================================================================
 */

/**
 * @brief  Start non-blocking master transmit (enables event and buffer IRQs).
 * @param  pHandle      I2C handle.
 * @param  pTxBuffer    TX data.
 * @param  len          Bytes to send.
 * @param  slaveAddr    Slave address.
 * @param  repeatedStart I2C_ENABLE_SR or I2C_DISABLE_SR.
 * @return I2C_OK if started, I2C_ERR_BUSY if already busy.
 * @note   Sets state to I2C_STATE_BUSY_TX. ISR handles events and completion.
 */
I2C_Status_t I2C_MasterSendDataIT(I2C_Handle_t *pHandle, uint8_t *pTxBuffer,
                                   uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart);

/**
 * @brief  Start non-blocking master receive (enables event and buffer IRQs).
 * @param  pHandle      I2C handle.
 * @param  pRxBuffer    RX buffer.
 * @param  len          Bytes to receive.
 * @param  slaveAddr    Slave address.
 * @param  repeatedStart I2C_ENABLE_SR or I2C_DISABLE_SR.
 * @return I2C_OK if started, I2C_ERR_BUSY if already busy.
 * @note   Sets state to I2C_STATE_BUSY_RX.
 */
I2C_Status_t I2C_MasterReceiveDataIT(I2C_Handle_t *pHandle, uint8_t *pRxBuffer,
                                      uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart);

/* =============================================================================
 * IRQ Configuration and ISR Handling
 * =============================================================================
 */

/**
 * @brief  Enable/Disable NVIC IRQ for I2C Event or Error line.
 * @param  IRQNumber  I2C1_EV_IRQn, I2C1_ER_IRQn, etc.
 * @param  EnorDi     ENABLE or DISABLE.
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief  Set NVIC priority for an I2C IRQ line.
 * @param  IRQNumber   Target IRQ (Event or Error).
 * @param  IRQPriority Encoded priority (upper 4 bits used on STM32F4).
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief  I2C Event ISR handler (call from I2Cx_EV_IRQHandler).
 * @param  pHandle  I2C handle with active transaction state.
 * @note   Handles SB, ADDR, TxE, RxNE, BTF, STOPF events.
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pHandle);

/**
 * @brief  I2C Error ISR handler (call from I2Cx_ER_IRQHandler).
 * @param  pHandle  I2C handle.
 * @note   Handles BERR, ARLO, AF, OVR, TIMEOUT errors.
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pHandle);

/**
 * @brief  Close ongoing master send/receive operation (clears IRQs and state).
 * @param  pHandle  I2C handle.
 * @note   Useful for aborting a transaction or after completion.
 */
void I2C_CloseSendData(I2C_Handle_t *pHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pHandle);

/* =============================================================================
 * Peripheral Control
 * =============================================================================
 */

/**
 * @brief  Enable or disable the I2C peripheral (CR1.PE).
 * @param  pI2Cx   Target I2C register block.
 * @param  EnorDi  ENABLE or DISABLE.
 *
 * @note   I2C_Init() leaves PE=0; call this function to enable after initialization.
 *         This function can also be used to disable/re-enable the peripheral at runtime.
 *
 * @warning When disabling, ensure no communication is ongoing (check BUSY flag).
 *          Most CR1/CR2 configuration changes require PE=0.
 *
 * Safe usage sequence:
 *   1) I2C_Init(h)                        // Configure but leave PE=0
 *   2) Configure GPIO pins (SDA/SCL)      // AF4, Open-drain, Pull-up
 *   3) I2C_PeripheralControl(h->pI2Cx, ENABLE)  // Activate I2C peripheral
 *   4) Perform I2C transfers              // Master/Slave send/receive
 *   5) I2C_PeripheralControl(h->pI2Cx, DISABLE) // Deactivate when done (optional)
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/**
 * @brief  Control ACK generation (CR1.ACK bit).
 * @param  pI2Cx   Target I2C register block.
 * @param  EnorDi  ENABLE or DISABLE.
 * @note   In master receiver mode, ACK must be disabled before receiving last byte.
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/**
 * @brief  Generate START condition (sets CR1.START bit).
 * @param  pI2Cx  Target I2C register block.
 * @note   Used in master mode to initiate communication.
 */
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);

/**
 * @brief  Generate STOP condition (sets CR1.STOP bit).
 * @param  pI2Cx  Target I2C register block.
 * @note   Releases the bus; cleared by hardware after Stop is sent.
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/* =============================================================================
 * Helper Functions
 * =============================================================================
 */

/**
 * @brief  Get a status flag from SR1 or SR2.
 * @param  pI2Cx    Target I2C register block.
 * @param  flagName Flag bit (I2C_FLAG_*).
 * @return 1 if flag is set, 0 otherwise.
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);

/**
 * @brief  Clear the ADDR flag (sequence: read SR1 then SR2).
 * @param  pHandle  I2C handle.
 * @note   Required after address phase to release SCL stretch.
 */
void I2C_ClearADDRFlag(I2C_Handle_t *pHandle);

/**
 * @brief  Application event callback (weak; override in application code).
 * @param  pHandle  I2C handle.
 * @param  event    Event identifier (I2C_EVENT_* or I2C_ERROR_*).
 * @note   Called by ISR to notify application of completion or errors.
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle, I2C_Event_t event);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
