/* =============================================================================
 * Project  : STM32F407G-DISC1 I2C Driver Exercise - Slave Rx Blocking Version
 * File     : 007i2c_slave_rx_blocking.c
 * Purpose  : Test I2C_SlaveReceiveData API in slave mode with LED status feedback
 *
 * Exercise Requirements:
 *   1. Test I2C_SlaveReceiveData API to receive predefined message from Master
 *   2. I2C-1 Slave mode (PB6-PB7 pins)
 *   3. 7-bit addressing mode
 *   4. SCL = 100 kHz (Standard mode)
 *   5. Internal pull-up resistors enabled for SDA and SCL
 *   6. Slave address: 0x68 (common for sensors)
 *   7. LED feedback for comprehensive status indication
 *
 * Hardware Setup:
 *   - Two STM32F407G-DISC1 boards (Master and Slave)
 *   - I2C1 pins: PB6(SCL), PB7(SDA)
 *   - LEDs: PD12(Green), PD13(Orange), PD14(Red), PD15(Blue)
 *   - Cross-connect Master and Slave I2C pins
 *
 * DESIGN PHILOSOPHY:
 *   - I2C peripheral initialized ONCE at startup
 *   - Slave remains in ready state waiting for master
 *   - Blocking receive waits for master to send data
 *   - Data validation with LED status feedback
 *   - Auto-ready for next reception after processing
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                    SLAVE OPERATION FLOWCHART
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
 *   ┌────────────────     Orange LED flashing (3x)
 *   │ INITIALIZE I2C │     I2C configured and enabled ONCE
 *   │ (ONE TIME)     │     Peripheral stays enabled
 *   └──────┬─────────┘
 *          │
 *          ▼
 *   ┌────────────────     Green LED ON (continuous)
 *   │ READY STATE    │     ↕
 *   │ WAITING FOR    │     Waiting for master to send data
 *   │ MASTER         │     Slave address: 0x68
 *   └──────┬─────────┘
 *          │
 *          │                    ┌──── MASTER SENDS DATA ────┐
 *          │                    │ Master addresses slave    │
 *          │                    │ Master transmits data     │
 *          ▼                    └───────────────────────────┘
 * ╔═══════════════════════════════════════════════════════════════════════
 * ║              I2C SLAVE RX OPERATION (Blocking)
 * ╚═══════════════════════════════════════════════════════════════════════
 *          │
 *   Master addresses this slave (0x68)
 *          │
 *   Slave ACKs address match (hardware automatic)
 *          │
 *   ┌─────────── DATA RECEPTION ─────────────┐
 *   │ Receiving data from master             │
 *   │ Blue LED ON during reception           │
 *   │ Blocking call waits for all bytes      │
 *   └────────────────────┬───────────────────┘
 *                        │
 *                        ▼
 *                 Validate received data
 *                        │
 *                        ▼
 *                  Data matches expected?
 *                   ┌────┴────┐
 *                  YES        NO
 *                   │          │
 *                   ▼          ▼
 *            SUCCESS PATTERN  ERROR PATTERN
 *            All LEDs flash   Red blinks 5x
 *                   │          │
 *                   └────┬─────┘
 *                        │
 *                        ▼
 *                 Return to READY state
 *                        │
 *                        └──► READY for next reception
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                        LED STATUS MAPPING
 * └──────────────────────────────────────────────────────────────────────────
 * Orange (PD13): I2C Initialization Status
 * Green  (PD12): Ready State - ON when waiting for master
 * Blue   (PD15): Reception in Progress
 * All    (All):  Success celebration (all flash together)
 * Red    (PD14): Error indication
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                      I2C COMMUNICATION DETAILS
 * └──────────────────────────────────────────────────────────────────────────
 * Expected Message: "Hello Slave, I2C test message from Master!"
 * Message Length: 44 characters
 * I2C Mode: Slave mode responding to master addressing
 * Addressing: 7-bit slave address (0x68)
 * Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * Pull-ups: Internal pull-up resistors enabled on SDA and SCL
 * Reception: Blocking call waits until all data received
 *
 * Connection Requirements:
 * Slave STM32           Master STM32
 * -----------          -----------
 * PB6 (SCL)     <-->   PB6 (SCL)      Bidirectional clock line
 * PB7 (SDA)     <-->   PB7 (SDA)      Bidirectional data line
 * GND           ---    GND            Common ground ESSENTIAL
 *
 * Signal Requirements:
 * - Open-drain with pull-up resistors (internal enabled)
 * - Both master and slave must enable pull-ups OR use external 4.7kΩ
 * - Standard mode: 100 kHz SCL frequency
 * - Start/Stop conditions generated by master
 *
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include <string.h>

/* Expected message from master */
static const uint8_t expected_message[] = "Hello Slave, I2C test message from Master!";
static const uint16_t expected_length = sizeof(expected_message) - 1;

/* Receive buffer */
static uint8_t rx_buffer[64] = {0};

/* Slave address - 0x68 (common for sensors like MPU6050) */
#define SLAVE_ADDR  0x68

/* Global I2C handle */
static I2C_Handle_t g_i2c1_handle = {0};

/* LED Handle Structure for status indication */
typedef struct {
    GPIO_PinHandle_t orange;    // PD13 - I2C initialization status
    GPIO_PinHandle_t green;     // PD12 - Ready state indicator
    GPIO_PinHandle_t red;       // PD14 - Error indication
    GPIO_PinHandle_t blue;      // PD15 - Reception in progress
} MultiLED_Handle_t;

/* Global LED handle */
static MultiLED_Handle_t status_leds = {0};

/**
 * @brief  Software delay function for timing control.
 * @param  ms  Delay time in milliseconds (approximate).
 */
static void delay_ms(volatile uint32_t ms)
{
    // Rough delay assuming 168MHz CPU clock
    // Each iteration takes ~10 cycles, so 168000000/10 = 16800000 per second
    volatile uint32_t cycles = ms * 16800;
    while (cycles--) {
        __asm volatile("nop");
    }
}

/**
 * @brief  Configure GPIO pins for I2C1 alternate function in slave mode.
 *
 * I2C1 Pin Mapping on STM32F407 (Slave Mode):
 * - SCL: PB6 (AF4) - Serial clock line (bidirectional)
 * - SDA: PB7 (AF4) - Serial data line (bidirectional)
 *
 * Configuration:
 * - Mode: Alternate Function (GPIO_MODE_AF)
 * - Output Type: Open-drain (REQUIRED for I2C)
 * - Pull: Internal pull-up enabled (GPIO_PULLUP)
 * - Speed: High for reliable 100kHz operation
 * - Alternate Function: AF4 for I2C1
 *
 * CRITICAL: Open-drain + pull-up is MANDATORY for I2C bus
 */
static void i2c1_gpio_init(void)
{
    GPIO_PinHandle_t i2c_pin;

    /* Common configuration for both I2C pins */
    i2c_pin.config.port = GPIO_PORT_B;
    i2c_pin.config.mode = GPIO_MODE_AF;
    i2c_pin.config.otype = GPIO_OTYPE_OD;       // Open-drain (REQUIRED for I2C)
    i2c_pin.config.pull = GPIO_PULLUP;          // Internal pull-up enabled
    i2c_pin.config.speed = GPIO_SPEED_HIGH;     // High speed for reliable operation
    i2c_pin.config.af = 4;                      // AF4 for I2C1

    /* Configure SCL pin (PB6) */
    i2c_pin.config.pin = 6;
    GPIO_Init(&i2c_pin);

    /* Configure SDA pin (PB7) */
    i2c_pin.config.pin = 7;
    GPIO_Init(&i2c_pin);
}

/**
 * @brief  Configure all 4 status LEDs on STM32F407G-DISC1 board.
 */
static void multi_led_init(MultiLED_Handle_t *leds)
{
    /* Common LED configuration */
    GPIO_PinConfig_t led_config = {
        .port = GPIO_PORT_D,
        .mode = GPIO_MODE_OUTPUT,
        .otype = GPIO_OTYPE_PP,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_LOW,
        .af = 0
    };

    /* Orange LED - PD13 (Initialization Status) */
    leds->orange.config = led_config;
    leds->orange.config.pin = 13;
    GPIO_Init(&leds->orange);

    /* Green LED - PD12 (Ready State) */
    leds->green.config = led_config;
    leds->green.config.pin = 12;
    GPIO_Init(&leds->green);

    /* Red LED - PD14 (Error Indication) */
    leds->red.config = led_config;
    leds->red.config.pin = 14;
    GPIO_Init(&leds->red);

    /* Blue LED - PD15 (Reception in Progress) */
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
    /* Rotating startup pattern - 2 complete cycles */
    for (int cycle = 0; cycle < 2; cycle++) {
        led_on(&leds->orange); delay_ms(150); led_off(&leds->orange);
        led_on(&leds->green);  delay_ms(150); led_off(&leds->green);
        led_on(&leds->red);    delay_ms(150); led_off(&leds->red);
        led_on(&leds->blue);   delay_ms(150); led_off(&leds->blue);
    }
}

static void success_pattern(MultiLED_Handle_t *leds)
{
    /* All LEDs flash together 3 times */
    for (int i = 0; i < 3; i++) {
        led_on(&leds->orange);
        led_on(&leds->green);
        led_on(&leds->red);
        led_on(&leds->blue);
        delay_ms(200);
        all_leds_off(leds);
        delay_ms(200);
    }
}

static void error_pattern(MultiLED_Handle_t *leds)
{
    /* Red LED blinks rapidly */
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
    /* Orange and Red LEDs blink together rapidly */
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
 * @brief  Configure I2C1 in Slave mode for reception.
 *
 * I2C Configuration:
 * - Device: I2C1 (APB1 bus)
 * - Mode: Slave (responds to master addressing)
 * - Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * - Own Address: 0x68 (7-bit addressing)
 * - ACK: Enabled (slave must ACK during reception)
 * - Address Mode: 7-bit addressing
 * - FM Duty: Don't care in Standard mode
 *
 * Slave Mode Behavior:
 * - Responds when master addresses 0x68
 * - Hardware automatically handles address matching and ACK
 * - Receives data bytes synchronized to master's clock
 * - Clock stretching enabled (slave can hold SCL low if needed)
 *
 * @note ACK control is critical - must be enabled before receiving data
 */
static I2C_Status_t i2c1_slave_rx_init(I2C_Handle_t *i2c_handle)
{
    i2c_handle->dev = I2C_DEVICE_I2C1;

    /* I2C Configuration for Slave Rx */
    i2c_handle->cfg.sclSpeed = I2C_SPEED_SM;              // 100 kHz (Standard mode)
    i2c_handle->cfg.ownAddress = SLAVE_ADDR;              // Slave address 0x68
    i2c_handle->cfg.addrMode = I2C_ADDR_7BIT;             // 7-bit addressing
    i2c_handle->cfg.ackControl = I2C_ACK_ENABLE;          // ACK enabled
    i2c_handle->cfg.fmDutyCycle = I2C_FM_DUTY_2;          // Don't care in SM

    return I2C_Init(i2c_handle);
}

/**
 * @brief  Enter ready state - slave is initialized and waiting for master.
 */
static void enter_ready_state(MultiLED_Handle_t *leds)
{
    all_leds_off(leds);
    led_on(&leds->green);  // Green indicates ready for master communication
}

/**
 * @brief  Validate received data against expected message
 * @return 1 if data matches, 0 otherwise
 */
static uint8_t validate_received_data(uint8_t *data, uint16_t length)
{
    if (length != expected_length) {
        return 0;
    }

    return (memcmp(data, expected_message, length) == 0);
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

    /* Orange LED flashes during I2C initialization */
    for (int i = 0; i < 3; i++) {
        led_on(&status_leds.orange);
        delay_ms(200);
        led_off(&status_leds.orange);
        delay_ms(200);
    }

    /* Initialize I2C for Slave Rx */
    i2c_status = i2c1_slave_rx_init(&g_i2c1_handle);
    if (i2c_status != I2C_OK) {
        /* Fatal error - cannot proceed without I2C */
        while (1) {
            i2c_init_error_pattern(&status_leds);
            delay_ms(1000);
        }
    }

    /* Enable I2C peripheral and keep it enabled throughout operation */
    I2C_PeripheralControl(g_i2c1_handle.pI2Cx, ENABLE);

    /* Enable ACK (must be done after peripheral enable) */
    I2C_ManageAcking(g_i2c1_handle.pI2Cx, ENABLE);

    /* Brief pause to ensure I2C peripheral is stable */
    delay_ms(100);

    /* Enter ready state - slave is now fully initialized */
    enter_ready_state(&status_leds);

    /* =========================================================================
     * Main Loop - Slave Reception Cycle
     *
     * The slave continuously:
     * 1. Enters ready state (Green LED ON)
     * 2. Waits for master to address and send data (blocking call)
     * 3. Receives data (Blue LED ON during reception)
     * 4. Validates received data
     * 5. Shows success or error pattern
     * 6. Returns to ready state for next reception
     * =========================================================================
     */
    while (1) {
        /* Ready state - waiting for master */
        enter_ready_state(&status_leds);

        /* Indicate reception is about to start / in progress */
        led_on(&status_leds.blue);
        delay_ms(100);

        /* Clear receive buffer */
        memset(rx_buffer, 0, sizeof(rx_buffer));

        /* Blocking receive - wait for master to send data */
        i2c_status = I2C_SlaveReceiveData(&g_i2c1_handle, rx_buffer, expected_length);

        /* Turn off blue LED after reception */
        led_off(&status_leds.blue);

        /* Check reception status */
        if (i2c_status == I2C_OK) {
            /* Validate received data */
            if (validate_received_data(rx_buffer, expected_length)) {
                /* Data matches - success! */
                success_pattern(&status_leds);
            } else {
                /* Data mismatch - error */
                error_pattern(&status_leds);
            }
        } else {
            /* Reception error */
            error_pattern(&status_leds);
        }

        /* Pause before next reception cycle */
        delay_ms(500);
    }
}

/* =============================================================================
 * I2C SLAVE OPERATION EXPLANATION:
 *
 * HOW IT WORKS:
 * 1. Slave powers on, initializes I2C1, enables peripheral and ACK
 * 2. Slave enters ready state with Green LED ON
 * 3. Slave calls I2C_SlaveReceiveData() - this is a BLOCKING call
 * 4. Slave waits (blocks) until master addresses it and sends data
 * 5. When master addresses slave (0x68):
 *    - Hardware automatically ACKs address match
 *    - Blue LED turns ON to indicate reception in progress
 * 6. For each data byte:
 *    - Master sends byte on SDA
 *    - Slave receives byte
 *    - Slave automatically ACKs (hardware with ACK enabled)
 * 7. After all bytes received:
 *    - Blue LED turns OFF
 *    - Data is validated against expected message
 *    - Success or error pattern displayed
 * 8. Slave returns to ready state for next reception
 *
 * BLOCKING BEHAVIOR:
 * - I2C_SlaveReceiveData() blocks until all data received
 * - Slave cannot do other tasks while waiting
 * - Simple but not suitable for multi-tasking applications
 * - For non-blocking, use interrupt-driven approach
 *
 * ACK CONTROL:
 * - ACK must be enabled for slave reception
 * - Hardware automatically sends ACK after each byte
 * - Last byte is also ACKed (master generates STOP to end)
 *
 * =============================================================================
 * LED STATUS REFERENCE:
 *
 * STARTUP SEQUENCE:
 * - Rotating pattern: Orange→Green→Red→Blue (2 cycles) - System init
 * - Orange flashing (3x): I2C peripheral initialization
 * - Green ON (continuous): Ready state - slave waiting for master
 *
 * RECEPTION CYCLE:
 * - Green ON: Ready state, waiting for master
 * - Blue ON: Reception in progress
 * - All flash together (3x): Data received and validated successfully
 * - Red blinks (5x): Reception error or data mismatch
 *
 * ERROR INDICATION:
 * - Orange + Red rapid blink (5x): Fatal I2C initialization error
 * - Red blinks (5x): Reception error or data validation failed
 *
 * =============================================================================
 * EXPECTED MESSAGE:
 *
 * Message: "Hello Slave, I2C test message from Master!"
 * Length: 44 characters (excluding null terminator)
 *
 * Master must send exactly this message for validation to pass.
 * Any mismatch in length or content will trigger error pattern.
 *
 * =============================================================================
 * I2C CONFIGURATION SUMMARY:
 *
 * - Mode: Slave, 7-bit addressing, ACK enabled
 * - Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * - Own Address: 0x68
 * - Pins: PB6(SCL), PB7(SDA) with AF4
 * - GPIO: Open-drain with internal pull-up resistors
 * - Peripheral: I2C1 on APB1 bus
 * - Initialization: ONE TIME at startup (stays enabled)
 *
 * =============================================================================
 * CONNECTION REQUIREMENTS:
 *
 * Physical Connections:
 * Slave STM32           Master STM32
 * -----------          -----------
 * PB6 (SCL)     <-->   PB6 (SCL)      Clock line (open-drain)
 * PB7 (SDA)     <-->   PB7 (SDA)      Data line (open-drain)
 * GND           ---    GND            Common ground ESSENTIAL
 *
 * Pull-up Resistors:
 * - Internal pull-ups enabled on both master and slave
 * - For better signal integrity, can add external 4.7kΩ resistors
 * - Pull-up value: 4.7kΩ to 10kΩ typical for 100kHz
 *
 * Signal Requirements:
 * - Both SDA and SCL are open-drain bidirectional
 * - Pull-up resistors pull lines HIGH when not driven
 * - Devices pull lines LOW to transmit '0'
 * - Standard mode: 100 kHz SCL frequency
 * - Short wires (<30cm) recommended for reliable operation
 *
 * =============================================================================
 */
