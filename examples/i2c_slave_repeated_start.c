/* =============================================================================
 * Project  : STM32F407G-DISC1 I2C Driver Exercise - Slave Repeated Start Rx
 * File     : 009i2c_slave_repeated_start_rx.c
 * Purpose  : Test receiving TWICE in one transaction (master uses repeated start)
 *
 * Exercise Requirements:
 *   1. Test I2C_SlaveReceiveData with repeated start transaction
 *   2. I2C-1 Slave mode (PB6-PB7 pins)
 *   3. 7-bit addressing mode
 *   4. SCL = 100 kHz (Standard mode)
 *   5. Internal pull-up resistors enabled for SDA and SCL
 *   6. Slave address: 0x68
 *   7. Receive same message TWICE in single transaction
 *   8. Comprehensive LED feedback for each reception
 *
 * Hardware Setup:
 *   - Two STM32F407G-DISC1 boards (Master and Slave)
 *   - I2C1 pins: PB6(SCL), PB7(SDA)
 *   - LEDs: PD12(Green), PD13(Orange), PD14(Red), PD15(Blue)
 *   - Cross-connect Master and Slave I2C pins
 *
 * REPEATED START FROM SLAVE PERSPECTIVE:
 *   The slave sees:
 *     1. Address match → ACK → Receive data1
 *     2. Address match again (Sr) → ACK → Receive data2
 *     3. STOP condition
 *
 *   All in ONE transaction (bus never released)
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │              SLAVE REPEATED START RX FLOWCHART
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
 *   │ WAITING FOR    │     Waiting for master transaction
 *   │ MASTER         │     Slave address: 0x68
 *   └──────┬─────────┘
 *          │
 *          │                    ┌──── MASTER SENDS DATA ────┐
 *          │                    │ (with Repeated Start)     │
 *          ▼                    └───────────────────────────┘
 * ╔═══════════════════════════════════════════════════════════════════════
 * ║         I2C REPEATED START RECEPTION (Slave Perspective)
 * ╚═══════════════════════════════════════════════════════════════════════
 *          │
 *   Master addresses this slave (0x68)
 *          │
 *   Slave ACKs address match (hardware automatic)
 *          │
 *   ┌─────────── FIRST RECEPTION ─────────────┐
 *   │ Receiving from master                   │
 *   │ Blue LED ON during reception            │
 *   │ Receiving: "I2C Test!"                  │
 *   │ 10 bytes received                       │
 *   │ Blue LED blinks once                    │
 *   │ Blocking call waits for all bytes       │
 *   └────────────────────┬────────────────────┘
 *                        │
 *                        ▼
 *         Master generates Repeated Start (Sr)
 *         *** NO STOP - Transaction continues ***
 *                        │
 *         Master addresses slave again (0x68)
 *                        │
 *         Slave ACKs address match again
 *                        │
 *   ┌─────────── SECOND RECEPTION ────────────┐
 *   │ Receiving from master again             │
 *   │ Green LED ON during reception           │
 *   │ Receiving: "I2C Test!" (same message)   │
 *   │ 10 bytes received                       │
 *   │ Green LED blinks once                   │
 *   │ Blocking call waits for all bytes       │
 *   └────────────────────┬────────────────────┘
 *                        │
 *                        ▼
 *                 Master generates STOP
 *                 (Transaction complete)
 *                        │
 *                        ▼
 *                 Validate both messages
 *                   ┌────┴────┐
 *                  OK         ERROR
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
 *                        └──► READY for next transaction
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                        LED STATUS MAPPING
 * └──────────────────────────────────────────────────────────────────────────
 * Orange (PD13): I2C Initialization Status
 * Green  (PD12): Ready State / Second reception (after Sr)
 * Blue   (PD15): First reception in Progress
 * All    (All):  Success celebration (all flash together)
 * Red    (PD14): Error indication
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                      I2C COMMUNICATION DETAILS
 * └──────────────────────────────────────────────────────────────────────────
 * Expected Message: "I2C Test!" (10 characters)
 * Received: TWICE in one transaction from master
 * Total Data: 20 bytes (10 + 10)
 * I2C Mode: Slave mode responding to master
 * Addressing: 7-bit slave address (0x68)
 * Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * Pull-ups: Internal pull-up resistors enabled on SDA and SCL
 *
 * Bus Sequence (Slave Perspective):
 * ADDR match → Rx1 → ADDR match (Sr) → Rx2 → STOP
 *                              ↑
 *                  Repeated START from master
 *
 * Connection Requirements:
 * Slave STM32           Master STM32
 * -----------          -----------
 * PB6 (SCL)     <-->   PB6 (SCL)      Bidirectional clock line
 * PB7 (SDA)     <-->   PB7 (SDA)      Bidirectional data line
 * GND           ---    GND            Common ground ESSENTIAL
 *
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include <string.h>

/* Expected message from master (sent twice) */
static const uint8_t expected_message[] = "I2C Test!";
static const uint16_t expected_length = sizeof(expected_message) - 1;  // 10 bytes

/* Receive buffers for two receptions */
static uint8_t rx_buffer1[32] = {0};
static uint8_t rx_buffer2[32] = {0};

/* Slave address - 0x68 */
#define SLAVE_ADDR  0x68

/* Global I2C handle */
static I2C_Handle_t g_i2c1_handle = {0};

/* LED Handle Structure for status indication */
typedef struct {
    GPIO_PinHandle_t orange;    // PD13 - I2C initialization status
    GPIO_PinHandle_t green;     // PD12 - Ready state / Second reception
    GPIO_PinHandle_t red;       // PD14 - Error indication
    GPIO_PinHandle_t blue;      // PD15 - First reception in progress
} MultiLED_Handle_t;

/* Global LED handle */
static MultiLED_Handle_t status_leds = {0};

/**
 * @brief  Software delay function for timing control.
 * @param  ms  Delay time in milliseconds (approximate).
 */
static void delay_ms(volatile uint32_t ms)
{
    volatile uint32_t cycles = ms * 16800;
    while (cycles--) {
        __asm volatile("nop");
    }
}

/**
 * @brief  Configure GPIO pins for I2C1 alternate function in slave mode.
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

    /* Green LED - PD12 (Ready State / Second Reception) */
    leds->green.config = led_config;
    leds->green.config.pin = 12;
    GPIO_Init(&leds->green);

    /* Red LED - PD14 (Error Indication) */
    leds->red.config = led_config;
    leds->red.config.pin = 14;
    GPIO_Init(&leds->red);

    /* Blue LED - PD15 (First Reception in Progress) */
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
    I2C_Status_t i2c_status1, i2c_status2;

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
    i2c_status1 = i2c1_slave_rx_init(&g_i2c1_handle);
    if (i2c_status1 != I2C_OK) {
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
     * Main Loop - Slave Reception Cycle with Repeated Start
     *
     * The slave continuously:
     * 1. Enters ready state (Green LED ON)
     * 2. Waits for first reception (Blue LED ON)
     * 3. Receives first message (10 bytes)
     * 4. Waits for second reception after repeated start (Green LED ON)
     * 5. Receives second message (10 bytes)
     * 6. Validates both messages
     * 7. Shows success or error pattern
     * 8. Returns to ready state
     * =========================================================================
     */
    while (1) {
        /* Ready state - waiting for master transaction */
        enter_ready_state(&status_leds);

        /* =====================================================================
         * FIRST RECEPTION - Blue LED
         * Master sends first message
         * =====================================================================
         */
        led_on(&status_leds.blue);
        delay_ms(100);

        /* Clear first receive buffer */
        memset(rx_buffer1, 0, sizeof(rx_buffer1));

        /* Blocking receive - wait for master to send first message */
        i2c_status1 = I2C_SlaveReceiveData(&g_i2c1_handle, rx_buffer1, expected_length);

        /* Blue LED blinks to indicate first reception complete */
        led_off(&status_leds.blue);
        delay_ms(100);
        led_on(&status_leds.blue);
        delay_ms(100);
        led_off(&status_leds.blue);
        delay_ms(200);

        /* Check first reception status */
        if (i2c_status1 != I2C_OK) {
            error_pattern(&status_leds);
            delay_ms(500);
            continue;  // Return to ready state
        }

        /* =====================================================================
         * SECOND RECEPTION - Green LED
         * Master sends second message (after repeated start)
         * =====================================================================
         */
        led_on(&status_leds.green);
        delay_ms(100);

        /* Clear second receive buffer */
        memset(rx_buffer2, 0, sizeof(rx_buffer2));

        /* Blocking receive - wait for master to send second message */
        i2c_status2 = I2C_SlaveReceiveData(&g_i2c1_handle, rx_buffer2, expected_length);

        /* Green LED blinks to indicate second reception complete */
        led_off(&status_leds.green);
        delay_ms(100);
        led_on(&status_leds.green);
        delay_ms(100);
        led_off(&status_leds.green);
        delay_ms(200);

        /* Check second reception status */
        if (i2c_status2 != I2C_OK) {
            error_pattern(&status_leds);
            delay_ms(500);
            continue;  // Return to ready state
        }

        /* =====================================================================
         * VALIDATION
         * Check if both messages are correct and identical
         * =====================================================================
         */
        if (validate_received_data(rx_buffer1, expected_length) &&
            validate_received_data(rx_buffer2, expected_length)) {
            /* Both messages received correctly - success! */
            success_pattern(&status_leds);
        } else {
            /* Data mismatch - error */
            error_pattern(&status_leds);
        }

        /* Pause before next reception cycle */
        delay_ms(500);
    }
}

/* =============================================================================
 * I2C SLAVE REPEATED START RECEPTION EXPLANATION:
 *
 * WHAT SLAVE EXPERIENCES:
 * From the slave's perspective, repeated start looks like receiving two
 * separate messages, but the bus is never released between them.
 *
 * DETAILED SEQUENCE:
 * 1. Slave in ready state, ACK enabled, waiting
 * 2. Master generates START condition
 * 3. Master sends slave address (0x68) with Write bit
 * 4. Slave hardware detects address match → ACKs automatically
 * 5. Slave receives first message "I2C Test!" (10 bytes)
 * 6. Each byte is ACKed by slave
 * 7. Master generates REPEATED START (Sr) - NO STOP
 * 8. Master sends slave address (0x68) with Write bit AGAIN
 * 9. Slave hardware detects address match → ACKs automatically (again)
 * 10. Slave receives second message "I2C Test!" (10 bytes)
 * 11. Each byte is ACKed by slave
 * 12. Master generates STOP condition
 * 13. Transaction complete - bus released
 *
 * KEY POINTS:
 * - Slave sees TWO address matches in ONE transaction
 * - No STOP between the two receptions
 * - Bus remains busy throughout
 * - Slave calls I2C_SlaveReceiveData() TWICE
 * - Both calls are blocking (wait for master to send data)
 *
 * BLOCKING BEHAVIOR:
 * - First I2C_SlaveReceiveData() blocks until 10 bytes received
 * - Second I2C_SlaveReceiveData() blocks until next 10 bytes received
 * - Slave cannot do other tasks while waiting
 * - Simple but demonstrates repeated start handling
 *
 * WHY TWO SEPARATE RECEIVE CALLS?
 * - Each address match starts a new receive phase
 * - Slave doesn't know if master will send Sr or STOP
 * - Application must call receive again to get second message
 * - This matches how sensor register reads work
 *
 * REAL-WORLD EXAMPLE:
 * Sensor (slave) with register-based access:
 *   Master → Write register address (1 byte)
 *   Master → Repeated Start
 *   Master → Read register data (N bytes)
 *
 * Our simplified version:
 *   Master → Write message 1 (10 bytes)
 *   Master → Repeated Start
 *   Master → Write message 2 (10 bytes)
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
 * - Green ON: Ready state, waiting for transaction
 * - Blue ON → blink: First reception (10 bytes)
 * - Green ON → blink: Second reception after Sr (10 bytes)
 * - All flash together (3x): Both messages received and validated successfully
 * - Red blinks (5x): Reception error or data mismatch
 *
 * ERROR INDICATION:
 * - Orange + Red rapid blink (5x): Fatal I2C initialization error
 * - Red blinks (5x): Reception error or data validation failed
 *
 * =============================================================================
 * EXPECTED MESSAGES:
 *
 * Message 1: "I2C Test!" (10 characters)
 * Message 2: "I2C Test!" (10 characters - same as message 1)
 * Total: 20 bytes in one transaction
 *
 * Both messages must match the expected message for validation to pass.
 *
 * =============================================================================
 * I2C CONFIGURATION SUMMARY:
 *
 * - Mode: Slave receiver, 7-bit addressing, ACK enabled
 * - Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * - Own Address: 0x68
 * - Pins: PB6(SCL), PB7(SDA) with AF4
 * - GPIO: Open-drain with internal pull-up resistors
 * - Peripheral: I2C1 on APB1 bus
 * - Repeated Start: Slave receives twice without STOP between
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
 *
 * =============================================================================
 * DEBUGGING TIPS:
 *
 * If repeated start fails:
 * 1. Check that master sends Sr (not STOP) between transmissions
 * 2. Verify slave calls receive twice (not once)
 * 3. Check ACK is enabled on slave
 * 4. Monitor bus with logic analyzer to see Sr condition
 * 5. Ensure proper timing between calls
 *
 * Expected timing:
 * - First receive: ~1 ms at 100 kHz
 * - Sr condition: ~100 μs
 * - Second receive: ~1 ms at 100 kHz
 * - Total transaction: ~2.2 ms
 *
 * =============================================================================
 */
