/* =============================================================================
 * Project  : STM32F407G-DISC1 I2C Driver Exercise - Master Tx Blocking Version
 * File     : 007i2c_master_tx_blocking.c
 * Purpose  : Test I2C_MasterSendData API in master mode with comprehensive LED feedback
 *
 * Exercise Requirements:
 *   1. Test I2C_MasterSendData API to transmit predefined message to Slave
 *   2. I2C-1 Master mode (PB6-PB7 pins)
 *   3. 7-bit addressing mode
 *   4. SCL = 100 kHz (Standard mode)
 *   5. Internal pull-up resistors enabled for SDA and SCL
 *   6. Target slave address: 0x68
 *   7. Blocking transmission with LED status feedback
 *
 * Hardware Setup:
 *   - Two STM32F407G-DISC1 boards (Master and Slave)
 *   - I2C1 pins: PB6(SCL), PB7(SDA)
 *   - LEDs: PD12(Green), PD13(Orange), PD14(Red), PD15(Blue)
 *   - Cross-connect Master and Slave I2C pins
 *
 * DESIGN PHILOSOPHY:
 *   - I2C peripheral initialized ONCE at startup
 *   - Master generates START, sends slave address, transmits data, generates STOP
 *   - Blocking transmission waits until all data sent
 *   - Comprehensive LED feedback at each stage
 *   - Automatic retry cycle every 3 seconds
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                    MASTER OPERATION FLOWCHART
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
 * ║              I2C MASTER TX OPERATION (Blocking)
 * ╚═══════════════════════════════════════════════════════════════════════
 *          │
 *   Orange LED ON → Transmission cycle starting
 *          │
 *          ▼
 *   Generate START condition
 *          │
 *   Send slave address (0x68) + Write bit
 *          │
 *   Wait for ACK from slave
 *          │
 *   ┌─────────── DATA TRANSMISSION ──────────┐
 *   │ Blue LED ON during transmission        │
 *   │ Sending: "Hello Slave, I2C test..."   │
 *   │ 44 bytes transmitted byte-by-byte     │
 *   │ Blocking call waits for completion    │
 *   └────────────────────┬───────────────────┘
 *                        │
 *                        ▼
 *                 Generate STOP condition
 *                        │
 *                        ▼
 *                  Check status
 *                   ┌────┴────┐
 *                  OK         ERROR
 *                   │          │
 *                   ▼          ▼
 *            SUCCESS PATTERN  ERROR PATTERN
 *            All LEDs flash   Red blinks 5x
 *            Green blinks 3x
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
 * Orange (PD13): I2C Initialization Status / Transmission Cycle Start
 * Green  (PD12): Initialization Complete / Transmission Success
 * Blue   (PD15): Transmission in Progress
 * All    (All):  Success celebration (all flash together)
 * Red    (PD14): Error indication
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                      I2C COMMUNICATION DETAILS
 * └──────────────────────────────────────────────────────────────────────────
 * Transmitted Message: "Hello Slave, I2C test message from Master!"
 * Message Length: 44 characters
 * I2C Mode: Master mode, initiates all transfers
 * Addressing: 7-bit slave address (0x68)
 * Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * Pull-ups: Internal pull-up resistors enabled on SDA and SCL
 * Transmission: Blocking call waits until all data transmitted
 * Stop Condition: Generated after transmission complete (no repeated start)
 *
 * Connection Requirements:
 * Master STM32          Slave STM32
 * -----------          -----------
 * PB6 (SCL)     <-->   PB6 (SCL)      Bidirectional clock line
 * PB7 (SDA)     <-->   PB7 (SDA)      Bidirectional data line
 * GND           ---    GND            Common ground ESSENTIAL
 *
 * Signal Requirements:
 * - Open-drain with pull-up resistors (internal enabled)
 * - Both master and slave must enable pull-ups OR use external 4.7kΩ
 * - Standard mode: 100 kHz SCL frequency
 * - Master generates START/STOP conditions
 * - Master provides SCL clock
 *
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include <string.h>

/* Message to transmit to slave */
static const uint8_t tx_message[] = "Hello Slave, I2C test message from Master!";
static const uint16_t tx_length = sizeof(tx_message) - 1;

/* Target slave address - 0x68 (7-bit) */
#define SLAVE_ADDR  0x68

/* Global I2C handle */
static I2C_Handle_t g_i2c1_handle = {0};

/* LED Handle Structure for status indication */
typedef struct {
    GPIO_PinHandle_t orange;    // PD13 - I2C init status / cycle start
    GPIO_PinHandle_t green;     // PD12 - Init complete / success
    GPIO_PinHandle_t red;       // PD14 - Error indication
    GPIO_PinHandle_t blue;      // PD15 - Transmission in progress
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
 * @brief  Configure GPIO pins for I2C1 alternate function in master mode.
 *
 * I2C1 Pin Mapping on STM32F407 (Master Mode):
 * - SCL: PB6 (AF4) - Serial clock line (bidirectional, master generates clock)
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

    /* Orange LED - PD13 (Init Status / Cycle Start) */
    leds->orange.config = led_config;
    leds->orange.config.pin = 13;
    GPIO_Init(&leds->orange);

    /* Green LED - PD12 (Init Complete / Success) */
    leds->green.config = led_config;
    leds->green.config.pin = 12;
    GPIO_Init(&leds->green);

    /* Red LED - PD14 (Error Indication) */
    leds->red.config = led_config;
    leds->red.config.pin = 14;
    GPIO_Init(&leds->red);

    /* Blue LED - PD15 (Transmission in Progress) */
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

    /* Green LED blinks 3 times to confirm success */
    for (int i = 0; i < 3; i++) {
        led_on(&leds->green);
        delay_ms(300);
        led_off(&leds->green);
        delay_ms(300);
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
 *
 * I2C Configuration:
 * - Device: I2C1 (APB1 bus)
 * - Mode: Master (initiates all transfers)
 * - Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * - Own Address: Don't care in master mode (set to 0x00)
 * - ACK: Enabled (needed for potential master receiver mode)
 * - Address Mode: 7-bit addressing
 * - FM Duty: Don't care in Standard mode
 *
 * Master Mode Behavior:
 * - Generates START/STOP conditions
 * - Provides SCL clock at 100 kHz
 * - Addresses slave and transmits data
 * - Monitors ACK/NACK from slave
 * - Can stretch clock or abort on errors
 *
 * @note Own address doesn't matter in master-only mode but set anyway
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
 * @brief  Perform one I2C master transmission cycle with LED feedback
 */
static void i2c_master_tx_cycle(MultiLED_Handle_t *leds)
{
    I2C_Status_t status;

    /* Signal transmission cycle start */
    led_on(&leds->orange);
    delay_ms(500);
    led_off(&leds->orange);

    /* Indicate transmission in progress */
    led_on(&leds->blue);
    delay_ms(100);

    /* Blocking transmission to slave */
    status = I2C_MasterSendData(&g_i2c1_handle,
                                (uint8_t*)tx_message,
                                tx_length,
                                SLAVE_ADDR,
                                I2C_DISABLE_SR);  // Generate STOP after transmission

    /* Turn off blue LED after transmission */
    led_off(&leds->blue);

    /* Check transmission status */
    if (status == I2C_OK) {
        /* Transmission successful */
        success_pattern(leds);
    } else {
        /* Transmission error */
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
     * Main Loop - Master Transmission Cycle
     *
     * The master continuously:
     * 1. Signals cycle start (Orange LED)
     * 2. Transmits message to slave (Blue LED ON during transmission)
     * 3. Checks status and shows success or error pattern
     * 4. Waits 3 seconds
     * 5. Repeats cycle
     *
     * This allows testing of:
     * - Multiple consecutive transmissions
     * - Slave response verification
     * - Error recovery
     * =========================================================================
     */
    while (1) {
        /* Perform transmission cycle */
        i2c_master_tx_cycle(&status_leds);

        /* Wait 3 seconds before next cycle */
        delay_ms(3000);
    }
}

/* =============================================================================
 * I2C MASTER OPERATION EXPLANATION:
 *
 * HOW IT WORKS:
 * 1. Master powers on, initializes I2C1, enables peripheral
 * 2. Master signals ready with Green LED (1 sec)
 * 3. Every 3 seconds, master initiates transmission cycle:
 *    a. Orange LED ON - cycle starting
 *    b. Blue LED ON - transmission in progress
 *    c. I2C_MasterSendData() generates START condition
 *    d. Master sends slave address (0x68) with Write bit (0)
 *    e. Master waits for ACK from slave
 *    f. If ACK received, master sends data bytes one by one
 *    g. After each byte, master waits for ACK from slave
 *    h. After all bytes sent, master generates STOP condition
 *    i. Blue LED OFF
 *    j. Success or error pattern displayed
 *
 * BLOCKING BEHAVIOR:
 * - I2C_MasterSendData() blocks until transmission complete or error
 * - Master cannot do other tasks while transmitting
 * - Simple but not suitable for multi-tasking applications
 * - For non-blocking, use interrupt-driven approach
 *
 * ERROR HANDLING:
 * - If slave doesn't ACK address: I2C_ERR_AF returned
 * - If slave doesn't ACK data byte: I2C_ERR_AF returned
 * - If arbitration lost: I2C_ERR_ARLO returned
 * - If bus error: I2C_ERR_BERR returned
 * - Error patterns help identify issues
 *
 * START AND STOP CONDITIONS:
 * - START: SDA falls while SCL is HIGH (master initiates transfer)
 * - STOP: SDA rises while SCL is HIGH (master ends transfer)
 * - I2C_DISABLE_SR: Generate STOP after transmission (normal operation)
 * - I2C_ENABLE_SR: Generate repeated START (for multi-transfer sequences)
 *
 * =============================================================================
 * LED STATUS REFERENCE:
 *
 * STARTUP SEQUENCE:
 * - Rotating pattern: Orange→Green→Red→Blue (2 cycles) - System init
 * - Orange flashing (3x): I2C peripheral initialization
 * - Green ON (1 sec): Initialization complete, ready to transmit
 *
 * TRANSMISSION CYCLE (every 3 seconds):
 * - Orange ON (500ms): Transmission cycle starting
 * - Blue ON: Transmission in progress
 * - All flash together (3x) + Green blinks (3x): Success!
 * - Red blinks (5x): Transmission error
 * - All OFF: Waiting for next cycle
 *
 * ERROR INDICATION:
 * - Orange + Red rapid blink (5x): Fatal I2C initialization error
 * - Red blinks (5x): Transmission error (no ACK, bus error, etc.)
 *
 * =============================================================================
 * TRANSMITTED MESSAGE:
 *
 * Message: "Hello Slave, I2C test message from Master!"
 * Length: 44 characters (excluding null terminator)
 *
 * This message is sent in its entirety to the slave at address 0x68.
 * Slave must be ready to receive all 44 bytes.
 *
 * =============================================================================
 * I2C CONFIGURATION SUMMARY:
 *
 * - Mode: Master, 7-bit addressing, ACK enabled
 * - Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * - Target Slave Address: 0x68
 * - Pins: PB6(SCL), PB7(SDA) with AF4
 * - GPIO: Open-drain with internal pull-up resistors
 * - Peripheral: I2C1 on APB1 bus
 * - Initialization: ONE TIME at startup (stays enabled)
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
 * Pull-up Resistors:
 * - Internal pull-ups enabled on both master and slave
 * - For better signal integrity, can add external 4.7kΩ resistors
 * - Pull-up value: 4.7kΩ to 10kΩ typical for 100kHz
 * - Only one set of pull-ups needed (internal OR external)
 *
 * Signal Requirements:
 * - Both SDA and SCL are open-drain bidirectional
 * - Pull-up resistors pull lines HIGH when not driven
 * - Devices pull lines LOW to transmit '0'
 * - Master generates clock on SCL at 100 kHz
 * - Standard mode: 100 kHz SCL frequency
 * - Short wires (<30cm) recommended for reliable operation
 *
 * Timing:
 * - Master generates START condition
 * - Clock speed: 100 kHz (10 μs period)
 * - 44 bytes * 8 bits/byte = 352 bits
 * - Plus address, ACKs, START, STOP ≈ 370 bits total
 * - Transmission time: ~3.7 ms at 100 kHz
 *
 * =============================================================================
 */
