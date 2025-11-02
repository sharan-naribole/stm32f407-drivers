/* =============================================================================
 * Project  : STM32F407G-DISC1 I2C Driver Exercise - Master Repeated Start
 * File     : 009i2c_master_repeated_start.c
 * Purpose  : Test I2C Repeated Start (Sr) by sending same message TWICE in one transaction
 *
 * Exercise Requirements:
 *   1. Test I2C_MasterSendData with Repeated Start (Sr)
 *   2. I2C-1 Master mode (PB6-PB7 pins)
 *   3. 7-bit addressing mode
 *   4. SCL = 100 kHz (Standard mode)
 *   5. Internal pull-up resistors enabled for SDA and SCL
 *   6. Target slave address: 0x68
 *   7. Send predefined message TWICE in single transaction
 *   8. Comprehensive LED feedback for each stage
 *
 * Hardware Setup:
 *   - Two STM32F407G-DISC1 boards (Master and Slave)
 *   - I2C1 pins: PB6(SCL), PB7(SDA)
 *   - LEDs: PD12(Green), PD13(Orange), PD14(Red), PD15(Blue)
 *   - Cross-connect Master and Slave I2C pins
 *
 * REPEATED START CONCEPT:
 *   Normal (2 separate transactions):
 *     START → Addr(W) → Data → STOP
 *     START → Addr(W) → Data → STOP
 *
 *   Repeated Start (1 transaction):
 *     START → Addr(W) → Data → Sr → Addr(W) → Data → STOP
 *                                ↑
 *                    No STOP - Bus kept busy
 *
 * BENEFITS:
 *   - Atomic operation (no other master can interrupt)
 *   - Bus efficiency (no START/STOP overhead between messages)
 *   - Common in sensor read operations (write register → Sr → read data)
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │              REPEATED START OPERATION FLOWCHART
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
 *   ┌────────────────     Green LED ON (1 sec)
 *   │ INITIALIZATION │     System ready to transmit
 *   │ COMPLETE       │
 *   └──────┬─────────┘
 *          │
 *          │                    ┌──── EVERY 3 SECONDS ────┐
 *          ▼                    │                          │
 * ╔═══════════════════════════════════════════════════════════════════════
 * ║         I2C REPEATED START OPERATION (Single Transaction)
 * ╚═══════════════════════════════════════════════════════════════════════
 *          │
 *   Orange LED ON → Transaction starting
 *          │
 *          ▼
 *   Generate START condition
 *          │
 *   ┌─────────── FIRST TRANSMISSION ─────────────┐
 *   │ Blue LED ON                                 │
 *   │ Send slave address (0x68) + Write bit      │
 *   │ Wait for ACK from slave                    │
 *   │ Send message: "I2C Test!"                  │
 *   │ 10 bytes transmitted                       │
 *   │ Blue LED blinks once                       │
 *   └────────────────────┬────────────────────────┘
 *                        │
 *                        ▼
 *                 Generate REPEATED START (Sr)
 *                 *** NO STOP CONDITION ***
 *                 Bus remains busy
 *                        │
 *   ┌─────────── SECOND TRANSMISSION ────────────┐
 *   │ Green LED ON                                │
 *   │ Send slave address (0x68) + Write bit      │
 *   │ Wait for ACK from slave                    │
 *   │ Send SAME message: "I2C Test!"             │
 *   │ 10 bytes transmitted again                 │
 *   │ Green LED blinks once                      │
 *   └────────────────────┬────────────────────────┘
 *                        │
 *                        ▼
 *                 Generate STOP condition
 *                 (Bus released)
 *                        │
 *                        ▼
 *                  Check status
 *                   ┌────┴────┐
 *                  OK         ERROR
 *                   │          │
 *                   ▼          ▼
 *            SUCCESS PATTERN  ERROR PATTERN
 *            All LEDs flash   Red blinks 5x
 *            3 times
 *                   │          │
 *                   └────┬─────┘
 *                        │
 *                        ▼
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
 * Orange (PD13): I2C Initialization / Transaction Start
 * Blue   (PD15): First transmission in progress
 * Green  (PD12): Second transmission in progress (after Sr)
 * All    (All):  Success celebration (all flash together)
 * Red    (PD14): Error indication
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                      I2C COMMUNICATION DETAILS
 * └──────────────────────────────────────────────────────────────────────────
 * Transmitted Message: "I2C Test!" (10 characters)
 * Sent: TWICE in one transaction using Repeated Start
 * Total Data: 20 bytes (10 + 10)
 * I2C Mode: Master mode, initiates all transfers
 * Addressing: 7-bit slave address (0x68)
 * Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * Pull-ups: Internal pull-up resistors enabled on SDA and SCL
 *
 * Bus Sequence:
 * START → 0x68(W) → "I2C Test!" → Sr → 0x68(W) → "I2C Test!" → STOP
 *                                   ↑
 *                         Repeated START (no STOP)
 *
 * Connection Requirements:
 * Master STM32          Slave STM32
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

/* Message to transmit TWICE */
static const uint8_t tx_message[] = "I2C Test!";
static const uint16_t tx_length = sizeof(tx_message) - 1;  // 10 bytes

/* Target slave address - 0x68 (7-bit) */
#define SLAVE_ADDR  0x68

/* Global I2C handle */
static I2C_Handle_t g_i2c1_handle = {0};

/* LED Handle Structure for status indication */
typedef struct {
    GPIO_PinHandle_t orange;    // PD13 - I2C init / transaction start
    GPIO_PinHandle_t green;     // PD12 - Second transmission (after Sr)
    GPIO_PinHandle_t red;       // PD14 - Error indication
    GPIO_PinHandle_t blue;      // PD15 - First transmission
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
    volatile uint32_t cycles = ms * 16800;
    while (cycles--) {
        __asm volatile("nop");
    }
}

/**
 * @brief  Configure GPIO pins for I2C1 alternate function in master mode.
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

    /* Orange LED - PD13 (Init Status / Transaction Start) */
    leds->orange.config = led_config;
    leds->orange.config.pin = 13;
    GPIO_Init(&leds->orange);

    /* Green LED - PD12 (Second Transmission after Sr) */
    leds->green.config = led_config;
    leds->green.config.pin = 12;
    GPIO_Init(&leds->green);

    /* Red LED - PD14 (Error Indication) */
    leds->red.config = led_config;
    leds->red.config.pin = 14;
    GPIO_Init(&leds->red);

    /* Blue LED - PD15 (First Transmission) */
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
 * @brief  Configure I2C1 in Master mode for transmission.
 */
static I2C_Status_t i2c1_master_tx_init(I2C_Handle_t *i2c_handle)
{
    i2c_handle->dev = I2C_DEVICE_I2C1;

    /* I2C Configuration for Master Tx */
    i2c_handle->cfg.sclSpeed = I2C_SPEED_SM;              // 100 kHz (Standard mode)
    i2c_handle->cfg.ownAddress = 0x00;                    // Don't care in master mode
    i2c_handle->cfg.addrMode = I2C_ADDR_7BIT;             // 7-bit addressing
    i2c_handle->cfg.ackControl = I2C_ACK_ENABLE;          // ACK enabled
    i2c_handle->cfg.fmDutyCycle = I2C_FM_DUTY_2;          // Don't care in SM

    return I2C_Init(i2c_handle);
}

/**
 * @brief  Perform one I2C repeated start cycle with LED feedback
 *         Sends the same message TWICE in one transaction
 */
static void i2c_repeated_start_cycle(MultiLED_Handle_t *leds)
{
    I2C_Status_t status1, status2;

    /* Signal transaction start */
    led_on(&leds->orange);
    delay_ms(500);
    led_off(&leds->orange);

    /* =========================================================================
     * FIRST TRANSMISSION - Blue LED
     * Send message with REPEATED START (no STOP)
     * =========================================================================
     */
    led_on(&leds->blue);
    delay_ms(200);

    /* Send first message with REPEATED START (I2C_ENABLE_SR) */
    status1 = I2C_MasterSendData(&g_i2c1_handle,
                                 (uint8_t*)tx_message,
                                 tx_length,
                                 SLAVE_ADDR,
                                 I2C_ENABLE_SR);  // ← REPEATED START (no STOP)

    /* Blue LED blinks to indicate first transmission complete */
    led_off(&leds->blue);
    delay_ms(100);
    led_on(&leds->blue);
    delay_ms(100);
    led_off(&leds->blue);
    delay_ms(200);

    /* Check first transmission status */
    if (status1 != I2C_OK) {
        error_pattern(leds);
        all_leds_off(leds);
        return;
    }

    /* =========================================================================
     * SECOND TRANSMISSION - Green LED
     * Send SAME message with STOP condition
     * =========================================================================
     */
    led_on(&leds->green);
    delay_ms(200);

    /* Send second message with STOP (I2C_DISABLE_SR) */
    status2 = I2C_MasterSendData(&g_i2c1_handle,
                                 (uint8_t*)tx_message,
                                 tx_length,
                                 SLAVE_ADDR,
                                 I2C_DISABLE_SR);  // ← Generate STOP

    /* Green LED blinks to indicate second transmission complete */
    led_off(&leds->green);
    delay_ms(100);
    led_on(&leds->green);
    delay_ms(100);
    led_off(&leds->green);
    delay_ms(200);

    /* Check second transmission status */
    if (status2 == I2C_OK) {
        /* Both transmissions successful */
        success_pattern(leds);
    } else {
        /* Second transmission error */
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
        /* Fatal error - cannot proceed without I2C */
        while (1) {
            i2c_init_error_pattern(&status_leds);
            delay_ms(1000);
        }
    }

    /* Enable I2C peripheral and keep it enabled throughout operation */
    I2C_PeripheralControl(g_i2c1_handle.pI2Cx, ENABLE);

    /* Brief pause to ensure I2C peripheral is stable */
    delay_ms(100);

    /* Signal initialization complete */
    led_on(&status_leds.green);
    delay_ms(1000);
    led_off(&status_leds.green);

    /* =========================================================================
     * Main Loop - Repeated Start Transmission Cycle
     *
     * The master continuously:
     * 1. Signals transaction start (Orange LED)
     * 2. Sends message #1 with REPEATED START (Blue LED)
     * 3. Sends message #2 with STOP (Green LED)
     * 4. Shows success or error pattern
     * 5. Waits 3 seconds
     * 6. Repeats cycle
     *
     * This demonstrates:
     * - Repeated START capability (I2C_ENABLE_SR)
     * - Atomic multi-message transaction
     * - Proper bus control (no STOP between messages)
     * =========================================================================
     */
    while (1) {
        /* Perform repeated start transmission cycle */
        i2c_repeated_start_cycle(&status_leds);

        /* Wait 3 seconds before next cycle */
        delay_ms(3000);
    }
}

/* =============================================================================
 * I2C REPEATED START OPERATION EXPLANATION:
 *
 * WHAT IS REPEATED START?
 * - Repeated Start (Sr) is a START condition generated WITHOUT releasing the bus
 * - Bus remains busy between messages (no STOP condition)
 * - Prevents other masters from interrupting the transaction
 *
 * NORMAL vs REPEATED START:
 *
 * Normal (2 separate transactions):
 *   Transaction 1: START → Addr → Data1 → STOP
 *                                           ↓ Bus released, other masters can use
 *   Transaction 2: START → Addr → Data2 → STOP
 *
 * Repeated Start (1 atomic transaction):
 *   START → Addr → Data1 → Sr → Addr → Data2 → STOP
 *                           ↑
 *                  No STOP - bus stays busy
 *
 * OUR IMPLEMENTATION:
 *   START → 0x68(W) → "I2C Test!" → Sr → 0x68(W) → "I2C Test!" → STOP
 *           ↑                         ↑                             ↑
 *      First send             Repeated START                  Final STOP
 *   (I2C_ENABLE_SR)                                      (I2C_DISABLE_SR)
 *
 * HOW THE DRIVER IMPLEMENTS IT:
 * 1. First call: I2C_MasterSendData(..., I2C_ENABLE_SR)
 *    - Driver sends START, address, data
 *    - Driver generates REPEATED START instead of STOP
 *    - Bus remains busy, master keeps control
 *
 * 2. Second call: I2C_MasterSendData(..., I2C_DISABLE_SR)
 *    - No START needed (bus already busy)
 *    - Driver sends address, data
 *    - Driver generates STOP condition
 *    - Bus released
 *
 * WHEN TO USE REPEATED START:
 * 1. Sensor register read:
 *    - Write register address → Sr → Read register data
 *    - Example: MPU6050, BMP280, EEPROM
 *
 * 2. Multi-byte writes requiring atomicity:
 *    - Write command → Sr → Write parameters
 *    - Prevents other masters from interrupting
 *
 * 3. Bus efficiency:
 *    - Reduces START/STOP overhead
 *    - Faster for consecutive operations
 *
 * ADVANTAGES:
 * - Atomic operation (guaranteed no interruption)
 * - Bus efficiency (less overhead)
 * - Multi-master safety (keeps bus control)
 *
 * =============================================================================
 * LED STATUS REFERENCE:
 *
 * STARTUP SEQUENCE:
 * - Rotating pattern: Orange→Green→Red→Blue (2 cycles) - System init
 * - Orange flashing (3x): I2C peripheral initialization
 * - Green ON (1 sec): Initialization complete, ready to transmit
 *
 * REPEATED START CYCLE (every 3 seconds):
 * - Orange ON (500ms): Transaction starting
 * - Blue ON → blink: First transmission with REPEATED START
 * - Green ON → blink: Second transmission with STOP
 * - All flash together (3x): Both transmissions successful!
 * - Red blinks (5x): Transmission error
 * - All OFF: Waiting for next cycle
 *
 * ERROR INDICATION:
 * - Orange + Red rapid blink (5x): Fatal I2C initialization error
 * - Red blinks (5x): Transmission error during repeated start cycle
 *
 * =============================================================================
 * TRANSMITTED MESSAGE:
 *
 * Message: "I2C Test!"
 * Length: 10 characters (excluding null terminator)
 * Sent: TWICE in one transaction
 * Total: 20 bytes of data
 *
 * The slave must be ready to receive 10 bytes, then receive another 10 bytes
 * in the same transaction (after the repeated start).
 *
 * =============================================================================
 * I2C CONFIGURATION SUMMARY:
 *
 * - Mode: Master transmitter with repeated start
 * - Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * - Target Slave Address: 0x68
 * - Pins: PB6(SCL), PB7(SDA) with AF4
 * - GPIO: Open-drain with internal pull-up resistors
 * - Peripheral: I2C1 on APB1 bus
 * - Repeated Start: I2C_ENABLE_SR (first) → I2C_DISABLE_SR (second)
 *
 * =============================================================================
 * CONNECTION REQUIREMENTS:
 *
 * Physical Connections:
 * Master STM32          Slave STM32
 * -----------          -----------
 * PB6 (SCL)     <-->   PB6 (SCL)      Clock line (open-drain)
 * PB7 (SDA)     <-->   PB7 (SDA)      Data line (open-drain)
 * GND           ---    GND            Common ground ESSENTIAL
 *
 * =============================================================================
 * BUS TIMING ANALYSIS:
 *
 * Normal Operation (2 transactions):
 *   START(1) → 10 bytes → STOP → START(2) → 10 bytes → STOP
 *   Total: 2 START + 2 ADDR + 20 data + 20 ACK + 2 STOP = ~360 bits
 *   Time @ 100kHz: ~3.6 ms
 *
 * Repeated Start (1 transaction):
 *   START → 10 bytes → Sr → 10 bytes → STOP
 *   Total: 1 START + 2 ADDR + 20 data + 20 ACK + 1 Sr + 1 STOP = ~350 bits
 *   Time @ 100kHz: ~3.5 ms
 *
 * Savings: ~100 μs (small but demonstrates atomic operation benefit)
 *
 * =============================================================================
 */
