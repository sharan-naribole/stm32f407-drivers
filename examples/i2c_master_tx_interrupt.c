/* =============================================================================
 * Project  : STM32F407G-DISC1 I2C Driver Exercise - Master Tx Interrupt (Non-blocking)
 * File     : 010i2c_master_tx_interrupt.c
 * Purpose  : Test I2C_MasterSendDataIT API in master mode with interrupt-driven transmission
 *
 * Exercise Requirements:
 *   1. Test I2C_MasterSendDataIT API (non-blocking transmission)
 *   2. I2C-1 Master mode (PB6-PB7 pins)
 *   3. 7-bit addressing mode
 *   4. SCL = 100 kHz (Standard mode)
 *   5. Internal pull-up resistors enabled for SDA and SCL
 *   6. Target slave address: 0x68
 *   7. Interrupt-driven with event and error handlers
 *   8. Comprehensive LED feedback for each stage
 *
 * Hardware Setup:
 *   - Two STM32F407G-DISC1 boards (Master and Slave)
 *   - I2C1 pins: PB6(SCL), PB7(SDA)
 *   - LEDs: PD12(Green), PD13(Orange), PD14(Red), PD15(Blue)
 *   - Cross-connect Master and Slave I2C pins
 *
 * NON-BLOCKING vs BLOCKING:
 *   Blocking (007):
 *     - I2C_MasterSendData() waits until transmission complete
 *     - CPU stuck in polling loop
 *     - Simple but inefficient
 *
 *   Non-Blocking (this):
 *     - I2C_MasterSendDataIT() starts transmission and returns immediately
 *     - ISR handles byte-by-byte transmission
 *     - Application callback notified on completion
 *     - CPU free to do other work
 *
 * INTERRUPT ARCHITECTURE:
 *   1. Application calls I2C_MasterSendDataIT()
 *   2. Driver enables I2C event/buffer interrupts
 *   3. Hardware generates interrupts for each event
 *   4. ISR (I2C1_EV_IRQHandler) calls driver handler
 *   5. Driver sends bytes one by one in ISR
 *   6. On completion, driver calls application callback
 *   7. Application processes result
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │              INTERRUPT-DRIVEN MASTER TX FLOWCHART
 * └──────────────────────────────────────────────────────────────────────────
 *
 *     POWER ON
 *        │
 *        ▼
 *   ┌────────────────     STARTUP PATTERN:
 *   │ INITIALIZE     │     Orange→Green→Red→Blue (2 cycles)
 *   │ SYSTEM         │
 *   └──────┬─────────┘
 *          │
 *          ▼
 *   ┌────────────────     Configure NVIC
 *   │ ENABLE I2C     │     Enable I2C1_EV_IRQn
 *   │ INTERRUPTS     │     Enable I2C1_ER_IRQn
 *   └──────┬─────────┘
 *          │
 *          ▼
 *   ┌────────────────     Orange LED flashing (3x)
 *   │ INITIALIZE I2C │     I2C configured
 *   │ (ONE TIME)     │     Peripheral enabled
 *   └──────┬─────────┘
 *          │
 *          ▼
 *   ┌────────────────     Green LED ON (1 sec)
 *   │ INITIALIZATION │     Ready to transmit
 *   │ COMPLETE       │
 *   └──────┬─────────┘
 *          │
 *          │                    ┌──── EVERY 3 SECONDS ────┐
 *          ▼                    │                          │
 * ╔═══════════════════════════════════════════════════════════════════════
 * ║         I2C MASTER TX INTERRUPT OPERATION (Non-blocking)
 * ╚═══════════════════════════════════════════════════════════════════════
 *          │
 *   Orange LED ON → Transmission starting
 *          │
 *   Main: Call I2C_MasterSendDataIT()
 *          │
 *   Driver: Enable interrupts, return immediately
 *          │
 *   Main: Blue LED ON (transmission in progress)
 *          │
 *   Main: Continue with other work (can blink LED, etc.)
 *          │
 *          ▼
 *   ╔════════════════════════════════════╗
 *   ║    HARDWARE GENERATES EVENTS       ║
 *   ║    ISR HANDLES TRANSMISSION        ║
 *   ╚════════════════════════════════════╝
 *          │
 *   ISR: START event → Address sent
 *   ISR: ADDR event → Address ACKed
 *   ISR: TxE event → Send byte 1
 *   ISR: TxE event → Send byte 2
 *   ...
 *   ISR: TxE event → Send byte N
 *   ISR: BTF event → All bytes sent
 *   ISR: Generate STOP
 *          │
 *   ISR: Call application callback
 *          │
 *          ▼
 *   Callback: I2C_EVENT_TX_CMPLT received
 *          │
 *   Callback: Turn off Blue LED
 *          │
 *   Callback: Show success pattern
 *          │
 *          ▼
 *                 All LEDs OFF
 *                        │
 *                        ▼
 *                 Wait 3 seconds
 *                        │
 *                        └──► REPEAT CYCLE
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                        LED STATUS MAPPING
 * └──────────────────────────────────────────────────────────────────────────
 * Orange (PD13): I2C Initialization / Transmission Starting
 * Green  (PD12): Initialization Complete / Success
 * Blue   (PD15): Transmission in Progress (ISR active)
 * Red    (PD14): Error indication
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                      I2C COMMUNICATION DETAILS
 * └──────────────────────────────────────────────────────────────────────────
 * Transmitted Message: "Hello Slave, I2C interrupt test!"
 * Message Length: 34 characters
 * I2C Mode: Master mode, interrupt-driven
 * Addressing: 7-bit slave address (0x68)
 * Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * Interrupts: I2C1_EV_IRQn (events), I2C1_ER_IRQn (errors)
 *
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include <string.h>

/* Message to transmit to slave */
static const uint8_t tx_message[] = "Hello Slave, I2C interrupt test!";
static const uint16_t tx_length = sizeof(tx_message) - 1;  // 34 bytes

/* Target slave address - 0x68 (7-bit) */
#define SLAVE_ADDR  0x68

/* Global I2C handle */
static I2C_Handle_t g_i2c1_handle = {0};

/* Transmission state flags */
static volatile uint8_t tx_complete = 0;
static volatile uint8_t tx_error = 0;

/* LED Handle Structure for status indication */
typedef struct {
    GPIO_PinHandle_t orange;    // PD13 - Init / transmission starting
    GPIO_PinHandle_t green;     // PD12 - Init complete / success
    GPIO_PinHandle_t red;       // PD14 - Error indication
    GPIO_PinHandle_t blue;      // PD15 - Transmission in progress
} MultiLED_Handle_t;

/* Global LED handle */
static MultiLED_Handle_t status_leds = {0};

/**
 * @brief  Software delay function for timing control.
 */
static void delay_ms(volatile uint32_t ms)
{
    volatile uint32_t cycles = ms * 16800;
    while (cycles--) {
        __asm volatile("nop");
    }
}

/**
 * @brief  Configure GPIO pins for I2C1 alternate function.
 */
static void i2c1_gpio_init(void)
{
    GPIO_PinHandle_t i2c_pin;

    i2c_pin.config.port = GPIO_PORT_B;
    i2c_pin.config.mode = GPIO_MODE_AF;
    i2c_pin.config.otype = GPIO_OTYPE_OD;
    i2c_pin.config.pull = GPIO_PULLUP;
    i2c_pin.config.speed = GPIO_SPEED_HIGH;
    i2c_pin.config.af = 4;

    /* Configure SCL pin (PB6) */
    i2c_pin.config.pin = 6;
    GPIO_Init(&i2c_pin);

    /* Configure SDA pin (PB7) */
    i2c_pin.config.pin = 7;
    GPIO_Init(&i2c_pin);
}

/**
 * @brief  Configure all 4 status LEDs.
 */
static void multi_led_init(MultiLED_Handle_t *leds)
{
    GPIO_PinConfig_t led_config = {
        .port = GPIO_PORT_D,
        .mode = GPIO_MODE_OUTPUT,
        .otype = GPIO_OTYPE_PP,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_LOW,
        .af = 0
    };

    leds->orange.config = led_config;
    leds->orange.config.pin = 13;
    GPIO_Init(&leds->orange);

    leds->green.config = led_config;
    leds->green.config.pin = 12;
    GPIO_Init(&leds->green);

    leds->red.config = led_config;
    leds->red.config.pin = 14;
    GPIO_Init(&leds->red);

    leds->blue.config = led_config;
    leds->blue.config.pin = 15;
    GPIO_Init(&leds->blue);
}

/**
 * @brief  LED control helper functions
 */
static void led_on(GPIO_PinHandle_t *led)
{
    GPIO_WriteToOutputPin(led->port, led->config.pin, GPIO_PIN_SET);
}

static void led_off(GPIO_PinHandle_t *led)
{
    GPIO_WriteToOutputPin(led->port, led->config.pin, GPIO_PIN_RESET);
}

static void all_leds_off(MultiLED_Handle_t *leds)
{
    led_off(&leds->orange);
    led_off(&leds->green);
    led_off(&leds->red);
    led_off(&leds->blue);
}

/**
 * @brief  LED Pattern Functions
 */
static void startup_pattern(MultiLED_Handle_t *leds)
{
    for (int cycle = 0; cycle < 2; cycle++) {
        led_on(&leds->orange); delay_ms(150); led_off(&leds->orange);
        led_on(&leds->green);  delay_ms(150); led_off(&leds->green);
        led_on(&leds->red);    delay_ms(150); led_off(&leds->red);
        led_on(&leds->blue);   delay_ms(150); led_off(&leds->blue);
    }
}

static void success_pattern(MultiLED_Handle_t *leds)
{
    for (int i = 0; i < 3; i++) {
        led_on(&leds->orange);
        led_on(&leds->green);
        led_on(&leds->red);
        led_on(&leds->blue);
        delay_ms(200);
        all_leds_off(leds);
        delay_ms(200);
    }

    for (int i = 0; i < 3; i++) {
        led_on(&leds->green);
        delay_ms(300);
        led_off(&leds->green);
        delay_ms(300);
    }
}

static void error_pattern(MultiLED_Handle_t *leds)
{
    all_leds_off(leds);
    for (int i = 0; i < 5; i++) {
        led_on(&leds->red);
        delay_ms(150);
        led_off(&leds->red);
        delay_ms(150);
    }
}

static void i2c_init_error_pattern(MultiLED_Handle_t *leds)
{
    all_leds_off(leds);
    for (int i = 0; i < 5; i++) {
        led_on(&leds->orange);
        led_on(&leds->red);
        delay_ms(150);
        all_leds_off(leds);
        delay_ms(150);
    }
}

/**
 * @brief  Configure I2C1 in Master mode.
 */
static I2C_Status_t i2c1_master_tx_init(I2C_Handle_t *i2c_handle)
{
    i2c_handle->dev = I2C_DEVICE_I2C1;

    i2c_handle->cfg.sclSpeed = I2C_SPEED_SM;
    i2c_handle->cfg.ownAddress = 0x00;
    i2c_handle->cfg.addrMode = I2C_ADDR_7BIT;
    i2c_handle->cfg.ackControl = I2C_ACK_ENABLE;
    i2c_handle->cfg.fmDutyCycle = I2C_FM_DUTY_2;

    return I2C_Init(i2c_handle);
}

/**
 * @brief  Application event callback (called by ISR)
 * @note   This function is called from interrupt context!
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle, I2C_Event_t event)
{
    if (pHandle->pI2Cx == I2C1) {
        switch (event) {
            case I2C_EVENT_TX_CMPLT:
                /* Transmission complete */
                tx_complete = 1;
                tx_error = 0;
                break;

            case I2C_ERROR_AF:
            case I2C_ERROR_BERR:
            case I2C_ERROR_ARLO:
            case I2C_ERROR_OVR:
            case I2C_ERROR_TIMEOUT:
                /* Error occurred */
                tx_complete = 0;
                tx_error = 1;
                break;

            default:
                break;
        }
    }
}

/**
 * @brief  I2C1 Event Interrupt Handler
 */
void I2C1_EV_IRQHandler(void)
{
    I2C_EV_IRQHandling(&g_i2c1_handle);
}

/**
 * @brief  I2C1 Error Interrupt Handler
 */
void I2C1_ER_IRQHandler(void)
{
    I2C_ER_IRQHandling(&g_i2c1_handle);
}

/**
 * @brief  Perform one I2C master transmission cycle (interrupt-driven)
 */
static void i2c_master_tx_interrupt_cycle(MultiLED_Handle_t *leds)
{
    I2C_Status_t status;

    /* Reset flags */
    tx_complete = 0;
    tx_error = 0;

    /* Signal transmission starting */
    led_on(&leds->orange);
    delay_ms(500);
    led_off(&leds->orange);

    /* Start non-blocking transmission */
    led_on(&leds->blue);
    delay_ms(100);

    status = I2C_MasterSendDataIT(&g_i2c1_handle,
                                  (uint8_t*)tx_message,
                                  tx_length,
                                  SLAVE_ADDR,
                                  I2C_DISABLE_SR);

    if (status != I2C_OK) {
        /* Failed to start transmission */
        led_off(&leds->blue);
        error_pattern(leds);
        all_leds_off(leds);
        return;
    }

    /* Transmission started - ISR will handle it
     * CPU is now free to do other work!
     * We'll blink Blue LED while waiting to show CPU is free */
    uint32_t timeout = 1000;  // 1 second timeout
    while (!tx_complete && !tx_error && timeout > 0) {
        /* Blink blue LED to show we're not blocked */
        led_off(&leds->blue);
        delay_ms(50);
        led_on(&leds->blue);
        delay_ms(50);
        timeout -= 100;
    }

    /* Turn off blue LED */
    led_off(&leds->blue);

    /* Check result */
    if (tx_complete) {
        /* Success */
        success_pattern(leds);
    } else if (tx_error) {
        /* Error */
        error_pattern(leds);
    } else {
        /* Timeout */
        error_pattern(leds);
    }

    /* All LEDs off before next cycle */
    all_leds_off(leds);
}

int main(void)
{
    I2C_Status_t i2c_status;

    /* Initialize LED peripherals */
    multi_led_init(&status_leds);

    /* Signal system startup */
    startup_pattern(&status_leds);
    delay_ms(500);

    /* Configure GPIO pins for I2C1 */
    i2c1_gpio_init();

    /* Configure NVIC for I2C1 interrupts */
    I2C_IRQInterruptConfig(I2C1_EV_IRQn, ENABLE);
    I2C_IRQInterruptConfig(I2C1_ER_IRQn, ENABLE);
    I2C_IRQPriorityConfig(I2C1_EV_IRQn, 5);
    I2C_IRQPriorityConfig(I2C1_ER_IRQn, 5);

    /* Orange LED flashes during I2C initialization */
    for (int i = 0; i < 3; i++) {
        led_on(&status_leds.orange);
        delay_ms(200);
        led_off(&status_leds.orange);
        delay_ms(200);
    }

    /* Initialize I2C for Master Tx */
    i2c_status = i2c1_master_tx_init(&g_i2c1_handle);
    if (i2c_status != I2C_OK) {
        while (1) {
            i2c_init_error_pattern(&status_leds);
            delay_ms(1000);
        }
    }

    /* Enable I2C peripheral */
    I2C_PeripheralControl(g_i2c1_handle.pI2Cx, ENABLE);

    delay_ms(100);

    /* Signal initialization complete */
    led_on(&status_leds.green);
    delay_ms(1000);
    led_off(&status_leds.green);

    /* Main loop - interrupt-driven transmission cycle */
    while (1) {
        i2c_master_tx_interrupt_cycle(&status_leds);
        delay_ms(3000);
    }
}

/* =============================================================================
 * I2C INTERRUPT-DRIVEN OPERATION EXPLANATION:
 *
 * KEY DIFFERENCE FROM BLOCKING:
 * - Blocking: CPU waits in polling loop until transmission complete
 * - Interrupt: CPU free after starting transmission, ISR handles it
 *
 * INTERRUPT FLOW:
 * 1. Application: I2C_MasterSendDataIT() → Returns immediately
 * 2. Driver: Enables I2C interrupts (ITEVTEN, ITBUFEN, ITERREN)
 * 3. Hardware: Generates START → Interrupt
 * 4. ISR: I2C1_EV_IRQHandler() → I2C_EV_IRQHandling()
 * 5. Driver: Sends address
 * 6. Hardware: ADDR event → Interrupt
 * 7. ISR: Clears ADDR flag
 * 8. Hardware: TxE event → Interrupt
 * 9. ISR: Sends next byte
 * 10. Repeat 8-9 for all bytes
 * 11. Hardware: BTF event → Interrupt
 * 12. ISR: Generates STOP
 * 13. Driver: Calls application callback
 * 14. Callback: Sets tx_complete flag
 * 15. Main loop: Detects completion, shows success pattern
 *
 * ADVANTAGES:
 * - CPU not blocked during transmission
 * - Can do other work while transmitting
 * - More efficient for multi-tasking
 * - Better power efficiency (can sleep during transmission)
 *
 * CALLBACK NOTES:
 * - Called from ISR context (keep it short!)
 * - Just set flags, don't do heavy processing
 * - Process results in main loop
 *
 * =============================================================================
 */
