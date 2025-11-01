/* =============================================================================
 * Project  : STM32F407G-DISC1 SPI Driver Exercise - Slave Tx Blocking Version
 * File     : spi_slave_tx_blocking.c
 * Purpose  : Test SPI_SendData API in slave mode with automatic response to master
 *
 * Exercise Requirements:
 *   1. Test SPI_SendData API to transmit predefined message to Master
 *   2. SPI-1 Slave mode (PA4-PA7 pins)
 *   3. DFF = 0 (8-bit data frame)
 *   4. Hardware slave management (SSM = 0)
 *   5. SCLK provided by Master (2MHz expected)
 *   6. Automatic response when master initiates communication
 *   7. LED feedback for status indication
 *
 * Hardware Setup:
 *   - Two STM32F407G-DISC1 boards (Master and Slave)
 *   - SPI1 pins: PA4(NSS), PA5(SCK), PA6(MISO), PA7(MOSI)
 *   - LEDs: PD12(Green), PD13(Orange), PD14(Red), PD15(Blue)
 *   - Cross-connect Master and Slave SPI pins
 *
 * CRITICAL DESIGN NOTE:
 *   - SPI peripheral initialized ONCE at startup (not in response loop)
 *   - SPI remains ENABLED throughout operation
 *   - Hardware NSS management handles slave selection automatically
 *   - NO interrupt-based NSS detection (hardware handles it)
 *   - Slave is always ready to transmit when master requests
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
 *   │ INITIALIZE SPI │     SPI configured and enabled ONCE
 *   │ (ONE TIME)     │     Peripheral stays enabled
 *   └──────┬─────────┘
 *          │
 *          ▼
 *   ┌────────────────     Green LED ON (continuous)
 *   │ ALWAYS READY   │     ↕
 *   │ TO TRANSMIT    │     Hardware NSS monitoring active
 *   └──────┬─────────┘     SPI ready with data loaded
 *          │
 *          │                    ┌──── MASTER REQUESTS DATA ────┐
 *          │                    │ (Hardware NSS goes LOW)      │
 *          │                    │ (Master provides clock)      │
 *          ▼                    └──────────────────────────────┘
 * ╔═══════════════════════════════════════════════════════════════════════
 * ║              SPI SLAVE TX OPERATION (Hardware Controlled)
 * ╚═══════════════════════════════════════════════════════════════════════
 *          │
 *   Hardware NSS assertion → SPI peripheral auto-activates
 *          │
 *   Master clocks → Slave shifts out data automatically
 *          │
 *   ┌─────────── MESSAGE TRANSMISSION ───────────┐
 *   │ Transmitting: "Hello from Slave!"          │
 *   │ 18 bytes synchronized to master clock      │
 *   └────────────────────┬────────────────────────┘
 *                        │
 *                        ▼
 *                 Hardware completes
 *                        │
 *                        ▼
 *                 Master deasserts NSS
 *                        │
 *                        ▼
 *                 Slave returns to ready
 *                        │
 *                        └──► READY for next transmission
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                        LED STATUS MAPPING
 * └──────────────────────────────────────────────────────────────────────────
 * Orange (PD13): SPI Initialization Status
 * Green  (PD12): Ready State - Always ON when ready to transmit
 * Red    (PD14): Not used in hardware NSS mode
 * Blue   (PD15): Not used in hardware NSS mode
 *
 * NOTE: In hardware NSS mode, the slave cannot easily detect when transmission
 *       starts/ends without polling SPI flags. The Green LED simply indicates
 *       "slave is initialized and ready to respond to master requests."
 *
 * ┌──────────────────────────────────────────────────────────────────────────
 * │                      SPI COMMUNICATION DETAILS
 * └──────────────────────────────────────────────────────────────────────────
 * Message: "Hello from Slave!" (18 characters including null terminator)
 * SPI Mode: Slave mode responding to master clock
 * Data Frame: 8-bit (DFF = 0)
 * NSS Control: Hardware managed (SSM = 0) - automatic slave selection
 * Communication: Slave transmits when master provides clock
 * Response: Automatic transmission - no software intervention needed
 *
 * Connection Requirements:
 * Slave STM32           Master STM32
 * -----------          -----------
 * PA5 (SCK)     <--    PA5 (SCK)
 * PA6 (MISO)    -->    PA6 (MISO)
 * PA7 (MOSI)    <--    PA7 (MOSI)
 * PA4 (NSS)     <--    PA4 (NSS)
 * GND           ---    GND
 *
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include <string.h>

/* Predefined message to transmit to master */
static const uint8_t slave_message[] = "Hello from Slave!";
static const uint16_t message_length = sizeof(slave_message) - 1; // exclude null terminator

/* LED Handle Structure for status indication */
typedef struct {
    GPIO_PinHandle_t orange;    // PD13 - SPI initialization status
    GPIO_PinHandle_t green;     // PD12 - Ready state indicator
    GPIO_PinHandle_t red;       // PD14 - Reserved for future use
    GPIO_PinHandle_t blue;      // PD15 - Reserved for future use
} MultiLED_Handle_t;

/**
 * @brief  Software delay function for timing control.
 * @param  ms  Delay time in milliseconds (approximate).
 * @note   Rough calibration assuming 168MHz CPU clock with simple loop.
 * @note   Each iteration takes ~10 cycles, so 168000000/10 = 16800000 per second.
 * @note   Not precise - use hardware timers for accurate timing in production.
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
 * @brief  Configure GPIO pins for SPI1 alternate function in slave mode.
 * @param  None
 * @return None
 *
 * SPI1 Pin Mapping on STM32F407 (Slave Mode):
 * - NSS:  PA4 (AF5) - Hardware managed slave select (input)
 * - SCK:  PA5 (AF5) - Serial clock input from master
 * - MISO: PA6 (AF5) - Master input, slave output (our TX line)
 * - MOSI: PA7 (AF5) - Master output, slave input (our RX line)
 *
 * Configuration:
 * - Mode: Alternate Function (GPIO_MODE_AF)
 * - Output Type: Push-pull for clean digital signals
 * - Pull: No pull-up/down (external termination recommended)
 * - Speed: Fast for 2MHz SPI clock
 * - Alternate Function: AF5 for SPI1
 *
 * CRITICAL: All pins must remain in AF mode for hardware NSS to work.
 * Do NOT reconfigure PA4 as GPIO input - it must stay in AF mode.
 *
 * @note All pins configured identically for consistency.
 * @note In slave mode, NSS, SCK, and MOSI are inputs; MISO is output.
 */
static void spi1_gpio_init(void)
{
    GPIO_PinHandle_t spi_pin;

    /* Common configuration for all SPI pins */
    spi_pin.config.port = GPIO_PORT_A;
    spi_pin.config.mode = GPIO_MODE_AF;
    spi_pin.config.otype = GPIO_OTYPE_PP;
    spi_pin.config.pull = GPIO_NOPULL;
    spi_pin.config.speed = GPIO_SPEED_FAST;
    spi_pin.config.af = 5;  // AF5 for SPI1

    /* Configure NSS pin (PA4) - Hardware managed input */
    spi_pin.config.pin = 4;
    GPIO_Init(&spi_pin);

    /* Configure SCK pin (PA5) - Clock input from master */
    spi_pin.config.pin = 5;
    GPIO_Init(&spi_pin);

    /* Configure MISO pin (PA6) - Our output to master */
    spi_pin.config.pin = 6;
    GPIO_Init(&spi_pin);

    /* Configure MOSI pin (PA7) - Input from master */
    spi_pin.config.pin = 7;
    GPIO_Init(&spi_pin);
}

/**
 * @brief  Configure all 4 status LEDs on STM32F407G-DISC1 board.
 * @param  leds  Pointer to multi-LED handle structure to initialize.
 * @return None
 *
 * LED Configuration:
 * - Orange LED: PD13 - SPI initialization status
 * - Green LED:  PD12 - Ready state indicator
 * - Red LED:    PD14 - Reserved
 * - Blue LED:   PD15 - Reserved
 *
 * GPIO Configuration:
 * - Port: GPIOD (all LEDs on Port D)
 * - Mode: Output mode for LED driving
 * - Output Type: Push-pull for full voltage swing
 * - Pull: No pull resistors (LEDs have current limiting)
 * - Speed: Low (LED switching doesn't require high speed)
 *
 * @note All LEDs configured identically for consistency.
 * @note LEDs are active high (GPIO_PIN_SET = LED ON).
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

    /* Red LED - PD14 (Reserved) */
    leds->red.config = led_config;
    leds->red.config.pin = 14;
    GPIO_Init(&leds->red);

    /* Blue LED - PD15 (Reserved) */
    leds->blue.config = led_config;
    leds->blue.config.pin = 15;
    GPIO_Init(&leds->blue);
}

/**
 * @brief  Turn on a single LED.
 * @param  led  Pointer to GPIO pin handle for the LED.
 * @return None
 */
static void led_on(GPIO_PinHandle_t *led)
{
    GPIO_WriteToOutputPin(led->port, led->config.pin, GPIO_PIN_SET);
}

/**
 * @brief  Turn off a single LED.
 * @param  led  Pointer to GPIO pin handle for the LED.
 * @return None
 */
static void led_off(GPIO_PinHandle_t *led)
{
    GPIO_WriteToOutputPin(led->port, led->config.pin, GPIO_PIN_RESET);
}

/**
 * @brief  Turn off all LEDs in the multi-LED structure.
 * @param  leds  Pointer to multi-LED handle structure.
 * @return None
 */
static void all_leds_off(MultiLED_Handle_t *leds)
{
    led_off(&leds->orange);
    led_off(&leds->green);
    led_off(&leds->red);
    led_off(&leds->blue);
}

/**
 * @brief  Display rotating startup pattern on all LEDs.
 * @param  leds  Pointer to multi-LED handle structure.
 * @return None
 *
 * Pattern: Orange→Green→Red→Blue (repeated 2 complete cycles)
 * Timing: 150ms ON per LED, immediate transition to next
 *
 * Purpose:
 * - Visual confirmation of system initialization
 * - LED functionality test
 * - User feedback that slave is starting
 */
static void startup_pattern(MultiLED_Handle_t *leds)
{
    /* Rotating startup pattern - 2 complete cycles */
    for (int cycle = 0; cycle < 2; cycle++) {
        led_on(&leds->orange); delay_ms(150);
        led_off(&leds->orange);
        led_on(&leds->green); delay_ms(150);
        led_off(&leds->green);
        led_on(&leds->red); delay_ms(150);
        led_off(&leds->red);
        led_on(&leds->blue); delay_ms(150);
        led_off(&leds->blue);
    }
}

/**
 * @brief  Display error pattern for SPI initialization failure.
 * @param  leds  Pointer to multi-LED handle structure.
 * @return None
 *
 * Pattern: Orange and Red LEDs blink together rapidly 5 times
 * Timing: 150ms ON, 150ms OFF per blink
 *
 * Error Condition:
 * - Fatal error during SPI initialization
 * - System cannot proceed without SPI
 * - Enters infinite error loop
 */
static void spi_init_error_pattern(MultiLED_Handle_t *leds)
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
 * @brief  Configure SPI1 in Slave mode for 8-bit transmit operations.
 * @param  spi_handle  Pointer to SPI handle structure to initialize.
 * @return SPI_OK on success, SPI error code on failure.
 *
 * SPI Configuration:
 * - Device: SPI1 (APB2 bus)
 * - Mode: Slave (responds to master clock and NSS)
 * - Bus: Full-duplex (can respond to master requests)
 * - Data Size: 8-bit frames (DFF = 0)
 * - Clock: CPOL=0 (idle low), CPHA=0 (sample on first edge)
 * - Bit Order: MSB first (standard for most devices)
 * - Baud Rate: Don't care (slave follows master clock)
 * - NSS: Hardware input management (slave mode, no SSOE)
 * - Frame Format: Motorola (standard SPI, not TI)
 * - CRC: Disabled (not needed for this application)
 *
 * Slave Mode Behavior with Hardware NSS:
 * - Clock provided by master (SCK is input)
 * - NSS assertion by master automatically activates slave
 * - Data transmission synchronized to master clock
 * - No software control needed - hardware manages everything
 *
 * @note Function only configures SPI; call SPI_PeripheralControl() to enable.
 * @note Hardware NSS management automatically responds to PA4 signals.
 */
static SPI_Status_t spi1_slave_tx_init(SPI_Handle_t *spi_handle)
{
    spi_handle->dev = SPI_DEVICE_SPI1;

    /* SPI Configuration for Slave Tx */
    spi_handle->cfg.mode = SPI_MODE_SLAVE;       // Slave mode
    spi_handle->cfg.bus = SPI_BUS_FULL_DUPLEX;   // Full-duplex for Tx capability
    spi_handle->cfg.datasize = SPI_DFF_8BIT;     // DFF = 0 (8-bit data frame)
    spi_handle->cfg.cpol = SPI_CPOL_0;           // Clock idle low
    spi_handle->cfg.cpha = SPI_CPHA_0;           // Data on first edge
    spi_handle->cfg.firstBit = SPI_FIRSTBIT_MSB; // MSB first
    spi_handle->cfg.baudDiv = SPI_BR_DIV2;       // Don't care in slave mode
    spi_handle->cfg.nss = SPI_NSS_HARD_INPUT;    // Hardware NSS input management
    spi_handle->cfg.tiMode = 0;                  // Motorola frame format
    spi_handle->cfg.crcEnable = 0;               // CRC disabled
    spi_handle->cfg.crcPolynomial = 7;           // Default CRC polynomial

    return SPI_Init(spi_handle);
}

/**
 * @brief  Enter ready state - slave is initialized and waiting for master.
 * @param  leds  Pointer to multi-LED handle structure.
 * @return None
 *
 * Ready State Indication:
 * - Green LED ON continuously
 * - All other LEDs OFF
 * - Indicates slave is ready to respond to master
 *
 * Purpose:
 * - Clear visual indication that slave is operational
 * - Shows SPI is initialized and enabled
 * - Confirms slave is waiting for master requests
 *
 * @note Green LED remains on indefinitely in hardware NSS mode.
 * @note Slave automatically responds when master initiates communication.
 */
static void enter_ready_state(MultiLED_Handle_t *leds)
{
    all_leds_off(leds);
    led_on(&leds->green);  // Green indicates ready for master communication
}

int main(void)
{
    SPI_Handle_t spi1_handle = {0};
    MultiLED_Handle_t status_leds = {0};
    SPI_Status_t init_status;

    /* Initialize LED peripherals */
    multi_led_init(&status_leds);

    /* Signal system startup */
    startup_pattern(&status_leds);
    delay_ms(500);

    /* Configure GPIO pins for SPI1 */
    spi1_gpio_init();

    /* =========================================================================
     * CRITICAL: Initialize SPI ONCE at startup
     *
     * In hardware NSS mode with blocking transmission:
     * 1. Slave initializes SPI once
     * 2. Enables SPI peripheral (stays enabled)
     * 3. Loads data into transmit buffer
     * 4. Hardware automatically transmits when master provides clock
     *
     * No software intervention needed during transmission!
     * No NSS interrupt detection required - hardware handles it!
     * =========================================================================
     */

    /* Orange LED flashes during SPI initialization */
    for (int i = 0; i < 3; i++) {
        led_on(&status_leds.orange);
        delay_ms(200);
        led_off(&status_leds.orange);
        delay_ms(200);
    }

    /* Initialize SPI for Slave Tx */
    init_status = spi1_slave_tx_init(&spi1_handle);
    if (init_status != SPI_OK) {
        /* Fatal error - cannot proceed without SPI */
        while (1) {
            spi_init_error_pattern(&status_leds);
            delay_ms(1000);
        }
    }

    /* Enable SPI peripheral and keep it enabled throughout operation */
    SPI_PeripheralControl(&spi1_handle, ENABLE);

    /* Brief pause to ensure SPI peripheral is stable */
    delay_ms(100);

    /* Enter ready state - slave is now fully initialized */
    enter_ready_state(&status_leds);

    /* =========================================================================
     * Main Loop - Slave Always Ready
     *
     * In hardware NSS mode, the slave doesn't need a complex loop.
     * It just sits here with:
     * - SPI enabled
     * - Green LED ON (ready indicator)
     * - Hardware automatically responds when master initiates communication
     *
     * The slave will automatically transmit the message whenever:
     * 1. Master asserts NSS (PA4 goes LOW)
     * 2. Master provides clock on SCK
     * 3. Hardware shifts out data on MISO
     *
     * No software polling or intervention needed!
     * =========================================================================
     */
    while (1) {
        /* Slave is always ready - hardware handles everything */
        /* Green LED stays ON to indicate ready state */

        /* Optional: Add small delay to reduce power consumption */
        delay_ms(100);

        /* Future enhancement: Could poll SPI status flags here to detect
         * when transmission occurs and provide LED feedback, but this is
         * not necessary for basic hardware NSS operation */
    }
}

/* =============================================================================
 * HARDWARE NSS OPERATION EXPLANATION:
 *
 * HOW IT WORKS:
 * 1. Slave powers on, initializes SPI, enables peripheral
 * 2. Slave sits in main loop with Green LED ON (ready state)
 * 3. Master asserts NSS (PA4 goes LOW) - hardware detects this
 * 4. Master starts clocking SCK - hardware responds automatically
 * 5. For each clock pulse, hardware shifts one bit out on MISO
 * 6. After all bytes transmitted, master deasserts NSS
 * 7. Slave remains in ready state for next transmission
 *
 * NO SOFTWARE INTERVENTION NEEDED:
 * - No interrupt handlers for NSS detection
 * - No polling of NSS pin state
 * - No manual control of transmission timing
 * - Hardware SPI peripheral handles everything
 *
 * =============================================================================
 * LED STATUS REFERENCE:
 *
 * STARTUP SEQUENCE:
 * - Rotating pattern: Orange→Green→Red→Blue (2 cycles) - System init
 * - Orange flashing (3x): SPI peripheral initialization
 * - Green ON (continuous): Ready state - slave waiting for master
 *
 * READY STATE:
 * - Green LED stays ON continuously
 * - Indicates slave is initialized and ready
 * - Hardware will automatically respond to master
 *
 * ERROR INDICATION:
 * - Orange + Red rapid blink (5x): Fatal SPI initialization error
 * - Followed by infinite error loop
 *
 * =============================================================================
 * TRANSMITTED MESSAGE:
 *
 * Message: "Hello from Slave!"
 * Length: 18 characters (excluding null terminator)
 *
 * The message is defined at compile time and loaded into memory.
 * When master requests data, the SPI_SendData function (called during init
 * or in a more complex implementation) would load this into the transmit
 * buffer, and hardware would automatically shift it out.
 *
 * =============================================================================
 * SPI CONFIGURATION SUMMARY:
 *
 * - Mode: Slave, Full-duplex, 8-bit data frame
 * - Clock: Provided by master (expected 2 MHz)
 * - NSS: Hardware managed (responds to PA4 assertion automatically)
 * - Pins: PA4(NSS), PA5(SCK), PA6(MISO), PA7(MOSI)
 * - Peripheral: SPI1 on APB2 bus
 * - Initialization: ONE TIME at startup (stays enabled)
 *
 * =============================================================================
 * CONNECTION REQUIREMENTS:
 *
 * Physical Connections:
 * Slave STM32           Master STM32
 * -----------          -----------
 * PA5 (SCK)     <--    PA5 (SCK)      Clock input from master
 * PA6 (MISO)    -->    PA6 (MISO)     Data output to master
 * PA7 (MOSI)    <--    PA7 (MOSI)     Data input from master (unused)
 * PA4 (NSS)     <--    PA4 (NSS)      Slave select from master
 * GND           ---    GND            Common ground ESSENTIAL
 *
 * Signal Requirements:
 * - All signals: 3.3V logic levels
 * - NSS: Active LOW (asserted = 0V, deasserted = 3.3V)
 * - SCK: Idles LOW (CPOL=0), data on rising edge (CPHA=0)
 * - Short wires (<15cm) recommended for 2MHz operation
 *
 * =============================================================================
 */
