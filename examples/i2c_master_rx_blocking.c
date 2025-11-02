/* =============================================================================
 * Project  : STM32F407G-DISC1 I2C Driver Exercise - Master Rx Blocking Version
 * File     : 008i2c_master_rx_blocking.c
 * Purpose  : Test I2C_MasterReceiveData API in master mode with comprehensive LED feedback
 *
 * Exercise Requirements:
 *   1. Test I2C_MasterReceiveData API to receive message from Slave
 *   2. I2C-1 Master mode (PB6-PB7 pins)
 *   3. 7-bit addressing mode
 *   4. SCL = 100 kHz (Standard mode)
 *   5. Internal pull-up resistors enabled for SDA and SCL
 *   6. Target slave address: 0x68
 *   7. Blocking reception with LED status feedback
 *   8. Message length > 2 bytes (avoids 2-byte special case)
 *
 * Hardware Setup:
 *   - Two STM32F407G-DISC1 boards (Master and Slave)
 *   - I2C1 pins: PB6(SCL), PB7(SDA)
 *   - LEDs: PD12(Green), PD13(Orange), PD14(Red), PD15(Blue)
 *   - Cross-connect Master and Slave I2C pins
 *
 * DESIGN PHILOSOPHY:
 *   - I2C peripheral initialized ONCE at startup
 *   - Master generates START, addresses slave in READ mode, receives data, generates STOP
 *   - Blocking reception waits until all data received
 *   - Data validation with comprehensive LED feedback
 *   - Automatic retry cycle every 3 seconds
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                    MASTER RX OPERATION FLOWCHART
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
 *   │ INITIALIZATION │     System ready to receive
 *   │ COMPLETE       │
 *   └──────┬─────────┘
 *          │
 *          │                    ┌──── EVERY 3 SECONDS ────┐
 *          ▼                    │                          │
 * ╔═══════════════════════════════════════════════════════════════════════
 * ║              I2C MASTER RX OPERATION (Blocking)
 * ╚═══════════════════════════════════════════════════════════════════════
 *          │
 *   Orange LED ON → Reception cycle starting
 *          │
 *          ▼
 *   Generate START condition
 *          │
 *   Send slave address (0x68) + Read bit (1)
 *          │
 *   Wait for ACK from slave
 *          │
 *   ┌─────────── DATA RECEPTION ──────────────┐
 *   │ Blue LED ON during reception            │
 *   │ Receiving: "Hello Master, from Slave!" │
 *   │ 28 bytes received byte-by-byte         │
 *   │ Master ACKs each byte except last      │
 *   │ Master NACKs last byte                 │
 *   │ Blocking call waits for completion     │
 *   └────────────────────┬────────────────────┘
 *                        │
 *                        ▼
 *                 Generate STOP condition
 *                        │
 *                        ▼
 *                 Validate received data
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
 * Orange (PD13): I2C Initialization Status / Reception Cycle Start
 * Green  (PD12): Initialization Complete / Reception Success
 * Blue   (PD15): Reception in Progress
 * All    (All):  Success celebration (all flash together)
 * Red    (PD14): Error indication
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                      I2C COMMUNICATION DETAILS
 * └──────────────────────────────────────────────────────────────────────────
 * Expected Message: "Hello Master, from Slave!"
 * Message Length: 28 characters (> 2 bytes, avoids special case)
 * I2C Mode: Master mode, initiates all transfers
 * Addressing: 7-bit slave address (0x68)
 * Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * Pull-ups: Internal pull-up resistors enabled on SDA and SCL
 * Reception: Blocking call waits until all data received
 * Stop Condition: Generated after reception complete (no repeated start)
 *
 * ACK/NACK Behavior:
 * - Master ACKs bytes 1 through N-1 (tells slave to continue)
 * - Master NACKs byte N (last byte - tells slave to stop)
 * - STOP condition releases the bus
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

/* Expected message from slave */
static const uint8_t expected_message[] = "Hello Master, from Slave!";
static const uint16_t expected_length = sizeof(expected_message) - 1;  // 28 bytes

/* Receive buffer */
static uint8_t rx_buffer[64] = {0};

/* Target slave address - 0x68 (7-bit) */
#define SLAVE_ADDR  0x68

/* Global I2C handle */
static I2C_Handle_t g_i2c1_handle = {0};

/* LED Handle Structure for status indication */
typedef struct {
    GPIO_PinHandle_t orange;    // PD13 - I2C init status / cycle start
    GPIO_PinHandle_t green;     // PD12 - Init complete / success
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
 * @brief  Configure I2C1 in Master mode for reception.
 *
 * I2C Configuration:
 * - Device: I2C1 (APB1 bus)
 * - Mode: Master (initiates all transfers)
 * - Speed: 100 kHz (Standard mode - I2C_SPEED_SM)
 * - Own Address: Don't care in master mode (set to 0x00)
 * - ACK: Enabled (master must ACK during reception)
 * - Address Mode: 7-bit addressing
 * - FM Duty: Don't care in Standard mode
 *
 * Master Receiver Mode Behavior:
 * - Generates START/STOP conditions
 * - Provides SCL clock at 100 kHz
 * - Addresses slave with READ bit (1)
 * - ACKs each received byte except the last one
 * - NACKs last byte to signal end of reception
 * - Can stretch clock or abort on errors
 *
 * @note ACK control is critical in master receiver mode
 */
static I2C_Status_t i2c1_master_rx_init(I2C_Handle_t *i2c_handle)
{
    i2c_handle->dev = I2C_DEVICE_I2C1;

    /* I2C Configuration for Master Rx */
    i2c_handle->cfg.sclSpeed = I2C_SPEED_SM;              // 100 kHz (Standard mode)
    i2c_handle->cfg.ownAddress = 0x00;                    // Don't care in master mode
    i2c_handle->cfg.addrMode = I2C_ADDR_7BIT;             // 7-bit addressing
    i2c_handle->cfg.ackControl = I2C_ACK_ENABLE;          // ACK enabled
    i2c_handle->cfg.fmDutyCycle = I2C_FM_DUTY_2;          // Don't care in SM

    return I2C_Init(i2c_handle);
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

/**
 * @brief  Perform one I2C master reception cycle with LED feedback
 */
static void i2c_master_rx_cycle(MultiLED_Handle_t *leds)
{
    I2C_Status_t status;

    /* Signal reception cycle start */
    led_on(&leds->orange);
    delay_ms(500);
    led_off(&leds->orange);

    /* Clear receive buffer */
    memset(rx_buffer, 0, sizeof(rx_buffer));

    /* Indicate reception in progress */
    led_on(&leds->blue);
    delay_ms(100);

    /* Blocking reception from slave */
    status = I2C_MasterReceiveData(&g_i2c1_handle,
                                    rx_buffer,
                                    expected_length,
                                    SLAVE_ADDR,
                                    I2C_DISABLE_SR);  // Generate STOP after reception

    /* Turn off blue LED after reception */
    led_off(&leds->blue);

    /* Check reception status */
    if (status == I2C_OK) {
        /* Validate received data */
        if (validate_received_data(rx_buffer, expected_length)) {
            /* Data matches - success! */
            success_pattern(leds);
        } else {
            /* Data mismatch - error */
            error_pattern(leds);
        }
    } else {
        /* Reception error */
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

    /* Initialize I2C for Master Rx */
    i2c_status = i2c1_master_rx_init(&g_i2c1_handle);
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
     * Main Loop - Master Reception Cycle
     *
     * The master continuously:
     * 1. Signals cycle start (Orange LED)
     * 2. Receives message from slave (Blue LED ON during reception)
     * 3. Validates data and shows success or error pattern
     * 4. Waits 3 seconds
     * 5. Repeats cycle
     *
     * This allows testing of:
     * - Multiple consecutive receptions
     * - Data validation
     * - Error recovery
     * =========================================================================
     */
    while (1) {
        /* Perform reception cycle */
        i2c_master_rx_cycle(&status_leds);

        /* Wait 3 seconds before next cycle */
        delay_ms(3000);
    }
}

/* =============================================================================
 * I2C MASTER RECEIVER OPERATION EXPLANATION:
 *
 * HOW IT WORKS:
 * 1. Master powers on, initializes I2C1, enables peripheral
 * 2. Master signals ready with Green LED (1 sec)
 * 3. Every 3 seconds, master initiates reception cycle:
 *    a. Orange LED ON - cycle starting
 *    b. Blue LED ON - reception in progress
 *    c. I2C_MasterReceiveData() generates START condition
 *    d. Master sends slave address (0x68) with Read bit (1)
 *    e. Master waits for ACK from slave
 *    f. If ACK received, master receives data bytes one by one:
 *       - Byte 1: Master ACKs (tells slave to send more)
 *       - Byte 2: Master ACKs
 *       - ...
 *       - Byte N-1: Master ACKs
 *       - Byte N (last): Master NACKs (tells slave this is the end)
 *    g. After all bytes received, master generates STOP condition
 *    h. Blue LED OFF
 *    i. Data is validated against expected message
 *    j. Success or error pattern displayed
 *
 * BLOCKING BEHAVIOR:
 * - I2C_MasterReceiveData() blocks until reception complete or error
 * - Master cannot do other tasks while receiving
 * - Simple but not suitable for multi-tasking applications
 * - For non-blocking, use interrupt-driven approach
 *
 * ACK/NACK CONTROL (CRITICAL):
 * - Master must ACK all bytes except the last one
 * - Last byte must be NACKed to tell slave to stop transmitting
 * - If master ACKs last byte, slave thinks more data is needed → bus hangs
 * - The driver handles ACK/NACK timing automatically for N>2 bytes
 *
 * N>2 BYTES RECEPTION (Our Case: 28 bytes):
 * - Simpler than 1-byte or 2-byte cases
 * - ACK enabled after ADDR phase
 * - Bytes 1 through N-2: Normal reception with ACK
 * - Byte N-1: After reading, disable ACK
 * - Byte N: Received with NACK, then STOP
 * - Driver handles timing automatically
 *
 * ERROR HANDLING:
 * - If slave doesn't ACK address: I2C_ERR_AF returned
 * - If arbitration lost: I2C_ERR_ARLO returned
 * - If bus error: I2C_ERR_BERR returned
 * - Error patterns help identify issues
 *
 * =============================================================================
 * LED STATUS REFERENCE:
 *
 * STARTUP SEQUENCE:
 * - Rotating pattern: Orange→Green→Red→Blue (2 cycles) - System init
 * - Orange flashing (3x): I2C peripheral initialization
 * - Green ON (1 sec): Initialization complete, ready to receive
 *
 * RECEPTION CYCLE (every 3 seconds):
 * - Orange ON (500ms): Reception cycle starting
 * - Blue ON: Reception in progress
 * - All flash together (3x) + Green blinks (3x): Success + data valid!
 * - Red blinks (5x): Reception error or data mismatch
 * - All OFF: Waiting for next cycle
 *
 * ERROR INDICATION:
 * - Orange + Red rapid blink (5x): Fatal I2C initialization error
 * - Red blinks (5x): Reception error or data validation failed
 *
 * =============================================================================
 * EXPECTED MESSAGE:
 *
 * Message: "Hello Master, from Slave!"
 * Length: 28 characters (excluding null terminator)
 *
 * This message is received in its entirety from the slave at address 0x68.
 * Slave must be ready to transmit all 28 bytes when master requests.
 *
 * =============================================================================
 * I2C CONFIGURATION SUMMARY:
 *
 * - Mode: Master receiver, 7-bit addressing, ACK enabled
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
 * - 28 bytes * 8 bits/byte = 224 bits
 * - Plus address, ACKs, START, STOP ≈ 240 bits total
 * - Reception time: ~2.4 ms at 100 kHz
 *
 * =============================================================================
 */
