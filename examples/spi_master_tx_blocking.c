/* =============================================================================
 * Project  : STM32F407G-DISC1 SPI Driver Exercise - Enhanced Multi-LED Version
 * File     : 004spi_master_tx_blocking.c
 * Purpose  : Test SPI_SendData API with extended test message using multiple LEDs
 *
 * Exercise Requirements:
 *   1. Test SPI_SendData API to send extended test message
 *   2. SPI-2 Master mode
 *   3. SCLK = max possible (APB1_CLK / 2 = ~21MHz)
 *   4. Test both DFF = 0 (8-bit) and DFF = 1 (16-bit) modes
 *   5. Enhanced visual feedback using all 4 LEDs
 *
 * Hardware Setup:
 *   - STM32F407G-DISC1 board
 *   - SPI2 pins: PB12(NSS), PB13(SCK), PB14(MISO), PB15(MOSI)
 *   - LEDs: PD12(Green), PD13(Orange), PD14(Red), PD15(Blue)
 *   - Logic analyzer or oscilloscope on SPI pins to verify transmission
 *
 * ┌─────────────────────────────────────────────────────────────────────────────┐
 * │                    EXERCISE OPERATION FLOWCHART                             │
 * └─────────────────────────────────────────────────────────────────────────────┘
 *
 *     POWER ON
 *        │
 *        ▼
 *   ┌─────────────┐     STARTUP PATTERN:
 *   │ INITIALIZE  │     Orange→Green→Red→Blue (2 cycles)
 *   │ SYSTEM      │
 *   └──────┬──────┘
 *          │
 *          ▼
 *   ┌─────────────┐     Orange LED ON (1 sec)
 *   │ READY FOR   │     ↓
 *   │ TESTING     │     System ready, enter main loop
 *   └──────┬──────┘
 *          │
 *          ▼                    ┌──── EVERY 3 SECONDS ────┐
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    TEST CYCLE                               │
 * └─────────────────────────────────────────────────────────────┘
 *          │
 *   Orange LED ON → Test cycle starting
 *          │
 *          ▼
 * ╔═══════════════════════════════════════════════════════════════╗
 * ║                    8-BIT TEST PHASE                           ║
 * ╚═══════════════════════════════════════════════════════════════╝
 *          │
 *   Green LED flashing (3x) → SPI init & preparation
 *          │
 *   Green LED SOLID → Transmitting test message (104 bytes)
 *          │
 *   ┌─────────┐ SUCCESS? ┌─────────┐
 *   │ Green   │ ────YES──▶│ Green   │
 *   │ blinks  │          │ slow    │
 *   │ 3x slow │          │ blinks  │
 *   └─────────┘          │ 3x      │
 *       │                └─────────┘
 *       │ FAIL                │
 *       ▼                     │ 500ms PAUSE
 *   ERROR PATTERN             ▼
 *   Red blinks 5x     ╔═══════════════════════════════════════════════╗
 *   → EXIT            ║            TRANSITION PHASE                   ║
 *                     ╚═══════════════════════════════════════════════╝
 *                             │
 *                      Orange LED pulses (2x) → Test mode switching
 *                             │
 *                             ▼
 *                     ╔═══════════════════════════════════════════════════════════════╗
 *                     ║                   16-BIT TEST PHASE                          ║
 *                     ╚═══════════════════════════════════════════════════════════════╝
 *                             │
 *                      Red LED flashing (4x) → SPI reconfigure & preparation
 *                             │
 *                      Red LED SOLID → Transmitting test message (52 words)
 *                             │
 *                      ┌─────────┐ SUCCESS? ┌─────────┐
 *                      │ Red     │ ────YES──▶│ Red     │
 *                      │ blinks  │          │ slow    │
 *                      │ 5x slow │          │ blinks  │
 *                      └─────────┘          │ 4x      │
 *                          │                └─────────┘
 *                          │ FAIL               │
 *                          ▼                   │ PAUSE (500ms)
 *                      ERROR PATTERN           ▼
 *                      → EXIT           ╔═══════════════════════════════════════════════╗
 *                                      ║           SUCCESS CELEBRATION               ║
 *                                      ╚═══════════════════════════════════════════════╝
 *                                                     │
 *                                              All LEDs flash together (3x)
 *                                                     │
 *                                              Blue LED ON (1 sec)
 *                                                     │
 *                                              All LEDs OFF
 *                                                     │
 *                                              WAIT 3 seconds
 *                                                     │
 *                                              ┌──────┴──────┐
 *                                              │ REPEAT LOOP │
 *                                              └─────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────────┐
 * │                        LED STATUS MAPPING                                   │
 * └─────────────────────────────────────────────────────────────────────────────┘
 * Orange (PD13): System Status / Initialization / Test Start
 * Green  (PD12): 8-bit Test Progress and Results
 * Red    (PD14): 16-bit Test Progress and Results
 * Blue   (PD15): Overall Test Success Indicator
 *
 * ┌─────────────────────────────────────────────────────────────────────────────┐
 * │                      SPI TRANSMISSION DETAILS                               │
 * └─────────────────────────────────────────────────────────────────────────────┘
 * Test Message: "SPI Master blocking transmission test using STM32F407 microcontroller with multiple LED status indicators"
 * Message Length: 104 characters
 * 8-bit Mode:   104 bytes transmitted sequentially
 * 16-bit Mode:  52 words transmitted (104 bytes total)
 * Clock Speed:  21 MHz (maximum for APB1 domain)
 * NSS Control:  Software managed (PB12 GPIO)
 * Transmission Time: ~50μs for 8-bit mode, ~25μs for 16-bit mode at 21MHz
 *
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

/* Test data as required - longer message for extended transmission visibility */
static const uint8_t test_message[] = "SPI Master blocking transmission test using STM32F407 microcontroller with multiple LED status indicators";
static const uint16_t test_message_len = sizeof(test_message) - 1; // exclude null terminator

/* LED Handle Structure for multiple LEDs */
typedef struct {
    GPIO_PinHandle_t orange;    // PD13 - System status
    GPIO_PinHandle_t green;     // PD12 - 8-bit test
    GPIO_PinHandle_t red;       // PD14 - 16-bit test
    GPIO_PinHandle_t blue;      // PD15 - Overall success
} MultiLED_Handle_t;

/* -----------------------------------------------------------------------------
 * Delay function for timing control
 * ---------------------------------------------------------------------------*/
static void delay_ms(volatile uint32_t ms)
{
    // Rough delay assuming 168MHz CPU clock
    // Each iteration takes ~10 cycles, so 168000000/10 = 16800000 per second
    volatile uint32_t cycles = ms * 16800;
    while (cycles--) {
        __asm volatile("nop");
    }
}

/* -----------------------------------------------------------------------------
 * Configure GPIO pins for SPI2 alternate function
 * SPI2 pins on STM32F407:
 *   - NSS:  PB12 (AF5)
 *   - SCK:  PB13 (AF5)
 *   - MISO: PB14 (AF5)
 *   - MOSI: PB15 (AF5)
 * ---------------------------------------------------------------------------*/
static void spi2_gpio_init(void)
{
    GPIO_PinHandle_t spi_pin;

    /* Common configuration for all SPI pins */
    spi_pin.config.port = GPIO_PORT_B;
    spi_pin.config.mode = GPIO_MODE_AF;
    spi_pin.config.otype = GPIO_OTYPE_PP;
    spi_pin.config.pull = GPIO_NOPULL;
    spi_pin.config.speed = GPIO_SPEED_FAST;
    spi_pin.config.af = 5;  // AF5 for SPI2

    /* Configure SCK pin (PB13) */
    spi_pin.config.pin = 13;
    GPIO_Init(&spi_pin);

    /* Configure MISO pin (PB14) */
    spi_pin.config.pin = 14;
    GPIO_Init(&spi_pin);

    /* Configure MOSI pin (PB15) */
    spi_pin.config.pin = 15;
    GPIO_Init(&spi_pin);

    /* Configure NSS pin (PB12) - Software managed */
    spi_pin.config.pin = 12;
    GPIO_Init(&spi_pin);
}

/* -----------------------------------------------------------------------------
 * Configure all 4 status LEDs on STM32F407G-DISC1
 * Orange (PD13), Green (PD12), Red (PD14), Blue (PD15)
 * ---------------------------------------------------------------------------*/
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

    /* Orange LED - PD13 (System Status) */
    leds->orange.config = led_config;
    leds->orange.config.pin = 13;
    GPIO_Init(&leds->orange);

    /* Green LED - PD12 (8-bit Test) */
    leds->green.config = led_config;
    leds->green.config.pin = 12;
    GPIO_Init(&leds->green);

    /* Red LED - PD14 (16-bit Test) */
    leds->red.config = led_config;
    leds->red.config.pin = 14;
    GPIO_Init(&leds->red);

    /* Blue LED - PD15 (Overall Success) */
    leds->blue.config = led_config;
    leds->blue.config.pin = 15;
    GPIO_Init(&leds->blue);
}

/* -----------------------------------------------------------------------------
 * LED Control Helper Functions
 * ---------------------------------------------------------------------------*/
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

/* -----------------------------------------------------------------------------
 * LED Pattern Functions
 * ---------------------------------------------------------------------------*/
static void startup_pattern(MultiLED_Handle_t *leds)
{
    /* Rotating startup pattern */
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

static void success_celebration(MultiLED_Handle_t *leds)
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

static void transition_pattern(MultiLED_Handle_t *leds)
{
    /* Orange LED pulses to indicate transition between tests */
    all_leds_off(leds);
    for (int i = 0; i < 2; i++) {
        led_on(&leds->orange);
        delay_ms(400);
        led_off(&leds->orange);
        delay_ms(300);
    }
}

static void error_pattern(MultiLED_Handle_t *leds)
{
    /* Red LED blinks slowly for visibility */
    all_leds_off(leds);
    for (int i = 0; i < 5; i++) {
        led_on(&leds->red);
        delay_ms(250);
        led_off(&leds->red);
        delay_ms(250);
    }
}

/* -----------------------------------------------------------------------------
 * Common SPI2 Master Configuration
 * Sets up all common parameters for maximum speed operation
 * SCLK = max possible (APB1_CLK/2 = ~21MHz)
 * ---------------------------------------------------------------------------*/
static void spi2_master_init_common(SPI_Handle_t *spi_handle)
{
    spi_handle->dev = SPI_DEVICE_SPI2;

    /* Common SPI Configuration */
    spi_handle->cfg.mode = SPI_MODE_MASTER;      // Master mode
    spi_handle->cfg.bus = SPI_BUS_FULL_DUPLEX;   // Full-duplex
    spi_handle->cfg.cpol = SPI_CPOL_0;           // Clock idle low
    spi_handle->cfg.cpha = SPI_CPHA_0;           // Data on first edge
    spi_handle->cfg.firstBit = SPI_FIRSTBIT_MSB; // MSB first
    spi_handle->cfg.baudDiv = SPI_BR_DIV2;       // Maximum speed: APB1_CLK/2
    spi_handle->cfg.nss = SPI_NSS_SOFTWARE;      // Manual CS control
    spi_handle->cfg.tiMode = 0;                  // Motorola frame format
    spi_handle->cfg.crcEnable = 0;               // CRC disabled
    spi_handle->cfg.crcPolynomial = 7;           // Default CRC polynomial
}

/* -----------------------------------------------------------------------------
 * Configure SPI2 Master for 8-bit mode (DFF = 0)
 * ---------------------------------------------------------------------------*/
static SPI_Status_t spi2_master_init_8bit(SPI_Handle_t *spi_handle)
{
    spi2_master_init_common(spi_handle);
    spi_handle->cfg.datasize = SPI_DFF_8BIT;    // DFF = 0
    return SPI_Init(spi_handle);
}

/* -----------------------------------------------------------------------------
 * Configure SPI2 Master for 16-bit mode (DFF = 1)
 * ---------------------------------------------------------------------------*/
static SPI_Status_t spi2_master_init_16bit(SPI_Handle_t *spi_handle)
{
    spi2_master_init_common(spi_handle);
    spi_handle->cfg.datasize = SPI_DFF_16BIT;   // DFF = 1
    return SPI_Init(spi_handle);
}

/* -----------------------------------------------------------------------------
 * Manual Chip Select control (since we're using software NSS)
 * ---------------------------------------------------------------------------*/
static void spi_cs_enable(void)
{
    GPIO_WriteToOutputPin(GPIOB, 12, GPIO_PIN_RESET);  // Pull NSS low
}

static void spi_cs_disable(void)
{
    GPIO_WriteToOutputPin(GPIOB, 12, GPIO_PIN_SET);    // Pull NSS high
}

/* -----------------------------------------------------------------------------
 * Enhanced SPI Exercise Test Function with Multi-LED feedback
 * Tests both DFF=0 and DFF=1 modes with clear visual indicators
 * ---------------------------------------------------------------------------*/
static void spi_exercise_test(SPI_Handle_t *spi_handle, MultiLED_Handle_t *leds)
{
    SPI_Status_t status;

    /* Signal test cycle start */
    led_on(&leds->orange);
    delay_ms(500);
    led_off(&leds->orange);

    /* =========================================================================
     * Test 1: DFF = 0 (8-bit mode) - Send test message
     * =========================================================================
     */

    /* Green LED indicates 8-bit test starting */
    led_on(&leds->green);

    /* Configure SPI for 8-bit mode */
    status = spi2_master_init_8bit(spi_handle);
    if (status != SPI_OK) {
        error_pattern(leds);
        return;
    }

    SPI_PeripheralControl(spi_handle, ENABLE);

    /* Green LED flashes during 8-bit test preparation */
    for (int i = 0; i < 3; i++) {
        led_off(&leds->green);
        delay_ms(200);
        led_on(&leds->green);
        delay_ms(200);
    }
    led_on(&leds->green);  // Solid during transmission

    /* Send test message in 8-bit mode */
    spi_cs_enable();
    delay_ms(1);

    status = SPI_SendData(spi_handle, test_message, test_message_len);

    delay_ms(1);
    spi_cs_disable();

    /* Check 8-bit test result */
    if (status == SPI_OK) {
        /* Success: Green LED slow blinks (3 complete cycles) */
        led_off(&leds->green);
        delay_ms(200);
        for (int i = 0; i < 3; i++) {
            led_on(&leds->green);
            delay_ms(300);
            led_off(&leds->green);
            delay_ms(300);
        }
    } else {
        error_pattern(leds);
        return;
    }

    SPI_PeripheralControl(spi_handle, DISABLE);
    delay_ms(500);  // Pause after 8-bit test

    /* Transition pattern between tests */
    transition_pattern(leds);

    /* =========================================================================
     * Test 2: DFF = 1 (16-bit mode) - Send test message
     * =========================================================================
     */

    /* Red LED indicates 16-bit test starting */
    led_on(&leds->red);

    /* Configure SPI for 16-bit mode */
    status = spi2_master_init_16bit(spi_handle);
    if (status != SPI_OK) {
        error_pattern(leds);
        return;
    }

    SPI_PeripheralControl(spi_handle, ENABLE);

    /* Red LED flashes during 16-bit test preparation */
    for (int i = 0; i < 4; i++) {
        led_off(&leds->red);
        delay_ms(200);
        led_on(&leds->red);
        delay_ms(200);
    }
    led_on(&leds->red);  // Solid during transmission

    /* Send test message in 16-bit mode */
    spi_cs_enable();
    delay_ms(1);

    uint16_t len_16bit = (test_message_len + 1) / 2;  // Round up for 16-bit words
    status = SPI_SendData(spi_handle, test_message, len_16bit);

    delay_ms(1);
    spi_cs_disable();

    /* Check 16-bit test result */
    if (status == SPI_OK) {
        /* Success: Red LED slow blinks (4 complete cycles) */
        led_off(&leds->red);
        delay_ms(200);
        for (int i = 0; i < 4; i++) {
            led_on(&leds->red);
            delay_ms(300);
            led_off(&leds->red);
            delay_ms(300);
        }

        /* Add delay before celebration pattern */
        delay_ms(500);

        /* Both tests passed - celebration pattern */
        success_celebration(leds);

        /* Blue LED indicates overall success */
        led_on(&leds->blue);
        delay_ms(1000);
        led_off(&leds->blue);

    } else {
        error_pattern(leds);
        return;
    }

    SPI_PeripheralControl(spi_handle, DISABLE);
    all_leds_off(leds);
}

int main(void)
{
    SPI_Handle_t spi2_handle = {0};
    MultiLED_Handle_t status_leds = {0};

    /* Initialize all LEDs */
    multi_led_init(&status_leds);

    /* Signal system startup */
    startup_pattern(&status_leds);
    delay_ms(500);

    /* Configure GPIO pins for SPI2 */
    spi2_gpio_init();

    /* Initialize CS pin high (deselected) */
    spi_cs_disable();

    /* Signal initialization complete */
    led_on(&status_leds.orange);
    delay_ms(1000);
    led_off(&status_leds.orange);

    /*
     * Main Test Loop
     * Enhanced visual feedback with 4 LEDs:
     * - Orange: System status and test transitions
     * - Green:  8-bit test progress
     * - Red:    16-bit test progress
     * - Blue:   Overall success indicator
     */
    while (1) {
        spi_exercise_test(&spi2_handle, &status_leds);
        delay_ms(3000);  // Wait 3 seconds between test cycles
    }
}

/* =============================================================================
 * LED Status Reference Guide:
 *
 * STARTUP:
 * - Rotating pattern: Orange→Green→Red→Blue (2 cycles)
 *
 * TEST CYCLE:
 * - Orange ON: Test cycle starting
 * - Green flashing: 8-bit test preparation
 * - Green solid: 8-bit transmission in progress
 * - Green slow blinks (3x): 8-bit test success
 * - Orange pulses (2x): Transition between tests
 * - Red flashing: 16-bit test preparation
 * - Red solid: 16-bit transmission in progress
 * - Red slow blinks (4x): 16-bit test success
 * - All LEDs flash together: Complete success celebration
 * - Blue ON (1 sec): Overall test cycle success
 *
 * ERRORS:
 * - Red slow blinks (5x): Test failure or initialization error
 *
 * Clock Speed Analysis:
 * STM32F407 typical configuration:
 * - SYSCLK = 168 MHz, AHB_CLK = 168 MHz
 * - APB1_CLK = 42 MHz, APB2_CLK = 84 MHz
 * - SPI2 on APB1: SPI_BR_DIV2 → 42MHz/2 = 21 MHz (MAXIMUM)
 * =============================================================================
 */
