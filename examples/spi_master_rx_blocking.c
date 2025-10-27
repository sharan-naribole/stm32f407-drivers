/* =============================================================================
 * Project  : STM32F407G-DISC1 SPI Driver Exercise - Master Rx with Software NSS
 * File     : 005spi_master_rx_blocking_software_nss.c
 * Purpose  : Test SPI_ReceiveData API with manual CS control via PB6
 *
 * Exercise Requirements:
 *   1. Test SPI_ReceiveData API to receive predefined message from Slave
 *   2. SPI-1 Master mode (PA5-PA7 for SPI signals)
 *   3. DFF = 0 (8-bit data frame)
 *   4. Software slave management (SSM = 1) - Manual CS control via PB6
 *   5. SCLK = 2MHz (from 16MHz APB2 clock)
 *   6. Button trigger and comprehensive LED feedback
 *   7. Master triggers Slave and validates received message
 *
 * Hardware Setup:
 *   - Two STM32F407G-DISC1 boards (Master and Slave)
 *   - SPI1 pins: PA5(SCK), PA6(MISO), PA7(MOSI)
 *   - CS Control: PB6 (Master output → Slave input)
 *   - Button: PA0 (User button on Discovery board)
 *   - LEDs: PD12(Green), PD13(Orange), PD14(Red), PD15(Blue)
 *   - Cross-connect Master and Slave SPI pins + CS line
 *
 * KEY CHANGES FROM HARDWARE NSS VERSION:
 *   - PA4 NOT used (avoids hardware NSS conflicts)
 *   - PB6 used for manual CS control (GPIO output on master)
 *   - SPI configured with SPI_NSS_SOFTWARE mode
 *   - CS manually asserted/deasserted around transfers
 *   - Shorter delays (10ms) sufficient for slave detection
 *
 * ┌────────────────────────────────────────────────────────────────────────────
 * │                    MASTER OPERATION FLOWCHART
 * └────────────────────────────────────────────────────────────────────────────
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
 *   ┌────────────────     Orange LED ON (continuous)
 *   │ READY FOR      │     ↕
 *   │ BUTTON PRESS   │     Waiting for user button press
 *   └──────┬─────────┘
 *          │
 *          ▼                    ┌──── ON BUTTON PRESS ────┐
 * ┌───────────────────────────────────────────────────────────────────────────
 * │                    MASTER RX TEST CYCLE
 * └───────────────────────────────────────────────────────────────────────────
 *          │
 *   Blue LED ON → Button press detected (hardware interrupt)
 *          │
 *          ▼
 * ╔═══════════════════════════════════════════════════════════════════════════
 * ║                    SPI MASTER RX OPERATION
 * ╚═══════════════════════════════════════════════════════════════════════════
 *          │
 *   Orange LED flashing (3x) → SPI initialization
 *          │
 *   PB6 = LOW → Assert CS to trigger slave
 *          │
 *   Green LED SOLID → Receiving message from slave
 *          │
 *   PB6 = HIGH → Deassert CS after reception
 *          │
 *   ┌─────────── MESSAGE RECEPTION  ───────────┐
 *   │ Receiving: "Hello from Slave!" (18 bytes)│
 *   └────────────────────┬──────────────────────┘
 *                        │
 *        ┌─────── RECEPTION STATUS   ────────┐
 *        │ Success │ ──────YES──► │ Validate │
 *        │ (SPI_OK)│              │ Message  │
 *        └─────────┘              └──────────┘
 *            │                          │
 *            │ FAIL              ┌──────┴──────┐
 *            ▼                   │   CORRECT?  │
 *    ┌─────────────────          └──────┬──────┘
 *    │ ERROR HANDLING  │                │
 *    └─────────────────┘         YES │  │ NO
 *            │                       │  │
 *            │                       │  └──► Wrong Message Error
 *            ▼                       │       (Orange + Red alternate)
 *     SPI ERROR:                     │
 *     Orange + Red together (5x)     ▼
 *            │                   SUCCESS INDICATION
 *            │                       │
 *            ▼                Green slow blinks (5x)
 *     NO MESSAGE:                    │
 *     Red blinks (3x)         All LEDs flash (3x)
 *            │                       │
 *            ▼                Blue ON (2 sec)
 *     WRONG MESSAGE:                 │
 *     Orange + Red alternate (3x)    │
 *            │                       │
 *            ▼                       ▼
 *     All LEDs OFF           All LEDs OFF
 *            │                       │
 *            │                       │
 *            └───────────┬───────────┘
 *                        │
 *                        ▼
 *                 ┌─────────────┐
 *                 │ READY LOOP  │
 *                 │ Orange LED  │
 *                 │ ON          │
 *                 └─────────────┘
 *
 * ┌────────────────────────────────────────────────────────────────────────────
 * │                        LED STATUS MAPPING
 * └────────────────────────────────────────────────────────────────────────────
 * Orange (PD13): System Ready State / SPI Operation / Error Indicator
 * Green  (PD12): Message Reception / Success Indicator
 * Red    (PD14): Error States
 * Blue   (PD15): Button Press Detection / Overall Success
 *
 * ┌────────────────────────────────────────────────────────────────────────────
 * │                      SPI COMMUNICATION DETAILS
 * └────────────────────────────────────────────────────────────────────────────
 * Expected Message: "Hello from Slave!" (18 characters)
 * SPI Mode: Master mode initiating communication
 * Data Frame: 8-bit (DFF = 0)
 * NSS Control: Software management - Manual CS via PB6 GPIO
 * Clock: 2MHz (APB2/8 = 16MHz/8)
 * Communication: Master receives data from slave
 * Validation: Compares received message with expected message
 *
 * Connection Requirements:
 * Master STM32          Slave STM32
 * -----------          -----------
 * PA5 (SCK)     -->    PA5 (SCK)      Serial clock
 * PA6 (MISO)    <--    PA6 (MISO)     Data from slave to master
 * PA7 (MOSI)    -->    PA7 (MOSI)     Data to slave (unused for RX)
 * PB6 (CS)      -->    PB6 (CS)       Chip select (manual control)
 * GND           ---    GND            Common ground ESSENTIAL
 *
 * CRITICAL NOTES:
 *   - PA4 is NOT used in this version (avoids hardware NSS issues)
 *   - PB6 controls slave selection manually
 *   - CS assertion (LOW) triggers slave to arm transmission
 *   - 10ms delay after CS assertion gives slave time to detect and prepare
 *   - CS deassertion (HIGH) after transfer completes the transaction
 *
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include <string.h>

/* Expected message from slave - matches slave's predefined message */
static const uint8_t expected_message[] = "Hello from Slave!";
static const uint16_t expected_message_len = sizeof(expected_message) - 1; // exclude null terminator
static uint8_t received_message[32]; // Buffer for received message with some extra space

/* LED Handle Structure for multiple LEDs */
typedef struct {
	GPIO_PinHandle_t orange;    // PD13 - System status / SPI operation
	GPIO_PinHandle_t green;     // PD12 - Message reception / Success
	GPIO_PinHandle_t red;       // PD14 - Error indicators
	GPIO_PinHandle_t blue;      // PD15 - Button press / Overall success
} MultiLED_Handle_t;

/* Button handle with interrupt support */
typedef struct {
	GPIO_PinHandle_t button;    // PA0 - User button
	volatile uint8_t pressed;   // Flag set by interrupt
} Button_Handle_t;

/* Global pointer used by ISR - declared before any function that uses it */
static Button_Handle_t *g_button_handle = NULL;

/**
 * @brief  Software delay function for timing control.
 * @param  ms  Delay time in milliseconds (approximate).
 * @note   Rough calibration assuming 168MHz CPU clock with simple loop.
 * @note   Each iteration takes ~10 cycles, so 168000000/10 = 16800000 per second.
 * @note   Not precise - use hardware timers for accurate timing in production.
 */
static void delay_ms(volatile uint32_t ms)
{
	// For 16MHz: 16,000,000 / 10 cycles per loop = 1,600,000 loops/sec
	// = 1,600 loops per millisecond
	volatile uint32_t cycles = ms * 1600;
	while (cycles--) {
		__asm volatile("nop");
	}
}


/**
 * @brief  Configure GPIO pins for SPI1 with SOFTWARE NSS (CS on PB6).
 * @param  None
 * @return None
 *
 * SPI1 Pin Mapping on STM32F407 (Software NSS):
 * - SCK:  PA5 (AF5) - Serial clock output (master mode)
 * - MISO: PA6 (AF5) - Master input, slave output
 * - MOSI: PA7 (AF5) - Master output, slave input
 * - CS:   PB6 (GPIO OUTPUT) - Manual chip select control
 *
 * IMPORTANT: PA4 is NOT configured/used in this version
 */
static void spi1_gpio_init(void)
{
	GPIO_PinHandle_t spi_pin;

	/* =========================================================================
	 * Configure SPI signal pins on Port A (SCK, MISO, MOSI)
	 * =========================================================================
	 */
	spi_pin.config.port = GPIO_PORT_A;
	spi_pin.config.mode = GPIO_MODE_AF;
	spi_pin.config.otype = GPIO_OTYPE_PP;
	spi_pin.config.pull = GPIO_NOPULL;
	spi_pin.config.speed = GPIO_SPEED_FAST;
	spi_pin.config.af = 5;  // AF5 for SPI1

	/* Configure SCK pin (PA5) */
	spi_pin.config.pin = 5;
	GPIO_Init(&spi_pin);

	/* Configure MISO pin (PA6) */
	spi_pin.config.pin = 6;
	GPIO_Init(&spi_pin);

	/* Configure MOSI pin (PA7) */
	spi_pin.config.pin = 7;
	GPIO_Init(&spi_pin);

	/* =========================================================================
	 * Configure CS (Chip Select) pin on PB6 as GPIO OUTPUT
	 * =========================================================================
	 */
	spi_pin.config.port = GPIO_PORT_B;
	spi_pin.config.mode = GPIO_MODE_OUTPUT;
	spi_pin.config.otype = GPIO_OTYPE_PP;
	spi_pin.config.speed = GPIO_SPEED_FAST;
	spi_pin.config.pull = GPIO_NOPULL;
	spi_pin.config.af = 0;  // Not used for GPIO mode
	spi_pin.config.pin = 6;
	GPIO_Init(&spi_pin);

	/* Initialize CS HIGH (deasserted/inactive) - slave not selected */
	GPIO_WriteToOutputPin(GPIOB, 6, GPIO_PIN_SET);
}

/**
 * @brief  Configure user button (PA0) for interrupt-based input detection.
 * @param  button_handle  Pointer to button handle structure to initialize.
 * @return None
 */
static void button_init(Button_Handle_t *button_handle)
{
	/* Initialize button pressed flag */
	button_handle->pressed = 0;

	/* Button configuration - PA0 as normal input */
	button_handle->button.config.port = GPIO_PORT_A;
	button_handle->button.config.pin  = 0;
	button_handle->button.config.mode = GPIO_MODE_INPUT;
	button_handle->button.config.otype = GPIO_OTYPE_PP;   // don't-care for input
	button_handle->button.config.pull = GPIO_NOPULL;      // external pull-down on Discovery
	button_handle->button.config.speed = GPIO_SPEED_LOW;  // don't-care for input
	button_handle->button.config.af = 0;                  // not used for input

	GPIO_Init(&button_handle->button);

	/* Configure EXTI line for PA0 -> EXTI0, rising edge trigger */
	GPIO_ConfigEXTI(&button_handle->button, GPIO_EXTI_TRIGGER_RISING, ENABLE);

	/* Enable NVIC for EXTI0 and set a medium priority (10) */
	GPIO_IRQInterruptConfig(EXTI0_IRQn, ENABLE);
	GPIO_IRQPriorityConfig(EXTI0_IRQn, 10U);

	/* Set global pointer for ISR */
	g_button_handle = button_handle;
}

/**
 * @brief  Configure all 4 status LEDs on STM32F407G-DISC1 board.
 * @param  leds  Pointer to multi-LED handle structure to initialize.
 * @return None
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

	/* Orange LED - PD13 (System Status) */
	leds->orange.config = led_config;
	leds->orange.config.pin = 13;
	GPIO_Init(&leds->orange);

	/* Green LED - PD12 (Message Reception/Success) */
	leds->green.config = led_config;
	leds->green.config.pin = 12;
	GPIO_Init(&leds->green);

	/* Red LED - PD14 (Error Indicators) */
	leds->red.config = led_config;
	leds->red.config.pin = 14;
	GPIO_Init(&leds->red);

	/* Blue LED - PD15 (Button Press/Overall Success) */
	leds->blue.config = led_config;
	leds->blue.config.pin = 15;
	GPIO_Init(&leds->blue);
}

/**
 * @brief  Turn on a single LED.
 */
static void led_on(GPIO_PinHandle_t *led)
{
	GPIO_WriteToOutputPin(led->port, led->config.pin, GPIO_PIN_SET);
}

/**
 * @brief  Turn off a single LED.
 */
static void led_off(GPIO_PinHandle_t *led)
{
	GPIO_WriteToOutputPin(led->port, led->config.pin, GPIO_PIN_RESET);
}

/**
 * @brief  Turn off all LEDs in the multi-LED structure.
 */
static void all_leds_off(MultiLED_Handle_t *leds)
{
	led_off(&leds->orange);
	led_off(&leds->green);
	led_off(&leds->red);
	led_off(&leds->blue);
}

/**
 * @brief  Wait for button press interrupt with improved debouncing.
 * @param  button_handle  Pointer to button handle structure.
 * @return None
 */
static void wait_for_button_press(Button_Handle_t *button_handle)
{
	/* Clear any pending flag first */
	button_handle->pressed = 0;

	/* Wait for interrupt to set the pressed flag */
	while (!button_handle->pressed) {
		delay_ms(10);  // Small delay to prevent busy waiting
	}

	/* Software debouncing - wait for button to stabilize */
	delay_ms(100);

	/* Clear flag for next use */
	button_handle->pressed = 0;
}

/**
 * @brief  Display rotating startup pattern on all LEDs.
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
 * @brief  Display success celebration pattern.
 */
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

/**
 * @brief  Display error pattern for no message received.
 */
static void no_message_error(MultiLED_Handle_t *leds)
{
	/* Red LED blinks 3 times for no message received */
	all_leds_off(leds);
	for (int i = 0; i < 3; i++) {
		led_on(&leds->red);
		delay_ms(300);
		led_off(&leds->red);
		delay_ms(300);
	}
}

/**
 * @brief  Display error pattern for wrong message received.
 */
static void wrong_message_error(MultiLED_Handle_t *leds)
{
	/* Orange and Red LEDs alternate 3 times for wrong message */
	all_leds_off(leds);
	for (int i = 0; i < 3; i++) {
		led_on(&leds->orange);
		delay_ms(250);
		led_off(&leds->orange);
		led_on(&leds->red);
		delay_ms(250);
		led_off(&leds->red);
	}
}

/**
 * @brief  Display error pattern for SPI initialization failure.
 */
static void spi_error_pattern(MultiLED_Handle_t *leds)
{
	/* Orange and Red LEDs blink rapidly for SPI initialization error */
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
 * @brief  Configure SPI1 in Master mode with SOFTWARE NSS management.
 * @param  spi_handle  Pointer to SPI handle structure to initialize.
 * @return SPI_OK on success, SPI error code on failure.
 *
 * SPI Configuration:
 * - Device: SPI1 (APB2 bus)
 * - Mode: Master mode
 * - Bus: Full-duplex (can receive data)
 * - Data Size: 8-bit frames (DFF = 0)
 * - Clock: CPOL=0 (idle low), CPHA=0 (sample on first edge)
 * - Bit Order: MSB first
 * - Baud Rate: APB2/8 = 16MHz/8 = 2MHz
 * - NSS: SOFTWARE management (manual CS control via PB6)
 * - Frame Format: Motorola (standard SPI)
 * - CRC: Disabled
 */
static SPI_Status_t spi1_master_rx_init(SPI_Handle_t *spi_handle)
{
	spi_handle->dev = SPI_DEVICE_SPI1;

	/* SPI Configuration for Master Rx with Software NSS */
	spi_handle->cfg.mode = SPI_MODE_MASTER;
	spi_handle->cfg.bus = SPI_BUS_FULL_DUPLEX;
	spi_handle->cfg.datasize = SPI_DFF_8BIT;
	spi_handle->cfg.cpol = SPI_CPOL_0;
	spi_handle->cfg.cpha = SPI_CPHA_0;
	spi_handle->cfg.firstBit = SPI_FIRSTBIT_MSB;
	spi_handle->cfg.baudDiv = SPI_BR_DIV8;  // 2MHz
	spi_handle->cfg.nss = SPI_NSS_SOFTWARE;  // Software NSS management
	spi_handle->cfg.tiMode = 0;
	spi_handle->cfg.crcEnable = 0;
	spi_handle->cfg.crcPolynomial = 7;

	return SPI_Init(spi_handle);
}

/**
 * @brief  Master SPI receive operation with manual CS control on PB6.
 * @param  spi_handle  Pointer to initialized SPI handle.
 * @param  rx_buffer   Pointer to receive buffer.
 * @param  len         Number of bytes to receive.
 * @return SPI_OK on success, SPI error code on failure.
 *
 * CS Control Sequence:
 * 1. Assert CS (PB6 = LOW) to select slave
 * 2. Wait 10ms for slave to detect CS and arm transmission
 * 3. Perform SPI receive operation (master clocks, slave sends data)
 * 4. Deassert CS (PB6 = HIGH) to deselect slave
 *
 * The 10ms delay in step 2 gives the slave's main loop enough time to:
 * - Detect the CS falling edge
 * - Call SPI_SendDataIT() to arm transmission
 * - Have the first byte pre-loaded and ready
 */
static SPI_Status_t master_receive_from_slave(SPI_Handle_t *spi_handle,
		uint8_t *rx_buffer,
		uint16_t len)
{
	SPI_Status_t status;

	/* Clear receive buffer */
	memset(rx_buffer, 0, len + 1);

	/* STEP 1: Assert CS (drive PB6 LOW) - select slave */
	GPIO_WriteToOutputPin(GPIOB, 6, GPIO_PIN_RESET);

	/* STEP 2: Give slave time to detect CS and arm transmission */
	delay_ms(10);  // 10ms is sufficient for slave polling loop

	/* STEP 3: Receive data from slave */
	/* SPI_ReceiveData automatically handles dummy transmission in master mode */
	status = SPI_ReceiveData(spi_handle, rx_buffer, len);

	/* STEP 4: Deassert CS (drive PB6 HIGH) - deselect slave */
	GPIO_WriteToOutputPin(GPIOB, 6, GPIO_PIN_SET);

	/* Null terminate received string for easy display/comparison */
	if (status == SPI_OK) {
		rx_buffer[len] = '\0';
	}

	return status;
}

/**
 * @brief  Validate received message against expected content.
 * @param  received  Pointer to received message buffer.
 * @param  expected  Pointer to expected message buffer.
 * @return 1 if messages match, 0 if they don't match.
 */
static uint8_t validate_received_message(const uint8_t *received, const uint8_t *expected)
{
	return (strcmp((const char*)received, (const char*)expected) == 0);
}

/**
 * @brief  Complete SPI Master Rx test cycle with comprehensive LED feedback.
 * @param  spi_handle  Pointer to SPI handle structure.
 * @param  leds        Pointer to LED handle structure.
 * @param  button      Pointer to button handle structure.
 * @return None
 *
 * Test Sequence:
 * 1. Wait for button press
 * 2. Initialize SPI peripheral
 * 3. Assert CS and receive data from slave
 * 4. Validate received message
 * 5. Display result via LED patterns
 * 6. Return to ready state
 */
static void spi_master_rx_test(SPI_Handle_t *spi_handle,
		MultiLED_Handle_t *leds,
		Button_Handle_t *button)
{
	SPI_Status_t status;
	uint8_t message_valid = 0;

	/* =========================================================================
	 * STEP 1: Wait for Button Press
	 * =========================================================================
	 */
	wait_for_button_press(button);

	/* Turn OFF Orange LED immediately on button detection */
	led_off(&leds->orange);

	/* Temporarily disable button interrupt during test */
	GPIO_IRQInterruptConfig(EXTI0_IRQn, DISABLE);

	/* Signal button press detected */
	led_on(&leds->blue);
	delay_ms(300);
	led_off(&leds->blue);

	/* =========================================================================
	 * STEP 2: Initialize SPI Peripheral
	 * =========================================================================
	 */
	all_leds_off(leds);

	/* Orange LED flashes during SPI initialization */
	for (int i = 0; i < 3; i++) {
		led_on(&leds->orange);
		delay_ms(200);
		led_off(&leds->orange);
		delay_ms(200);
	}

	/* Initialize SPI for Master Rx */
	status = spi1_master_rx_init(spi_handle);
	if (status != SPI_OK) {
		spi_error_pattern(leds);
		GPIO_IRQInterruptConfig(EXTI0_IRQn, ENABLE);
		return;
	}

	/* Enable SPI peripheral */
	SPI_PeripheralControl(spi_handle, ENABLE);

	/* Brief stabilization delay */
	delay_ms(50);

	/* =========================================================================
	 * STEP 3: Communicate with Slave
	 * =========================================================================
	 */

	/* Blue LED during CS assertion */
	led_on(&leds->blue);
	delay_ms(200);

	/* Green LED during message reception */
	led_off(&leds->blue);
	led_on(&leds->green);

	/* Receive message from slave (CS control is inside this function) */
	status = master_receive_from_slave(spi_handle, received_message, expected_message_len);

	led_off(&leds->green);

	/* Disable SPI peripheral */
	SPI_PeripheralControl(spi_handle, DISABLE);

	/* ========== DEBUG: Inspect received data ========== */
	volatile uint8_t rx_byte_0 = received_message[0];   // Should be 'H'
	volatile uint8_t rx_byte_1 = received_message[1];   // Should be 'e'
	volatile uint8_t rx_byte_6 = received_message[6];   // Should be 'f'
	volatile uint8_t rx_byte_17 = received_message[17]; // Should be '!'

	/* Set breakpoint here to inspect in debugger */
	volatile int check_data = 0;  // ← BREAKPOINT

	/* =========================================================================
	 * STEP 4: Validate Results and Display Status
	 * =========================================================================
	 */

	if (status != SPI_OK) {
		/* SPI communication error */
		spi_error_pattern(leds);
		GPIO_IRQInterruptConfig(EXTI0_IRQn, ENABLE);
		return;
	}

	/* Check if any message was received */
	if (received_message[0] == 0x00 || strlen((char*)received_message) == 0) {
		/* No message received */
		no_message_error(leds);
		GPIO_IRQInterruptConfig(EXTI0_IRQn, ENABLE);
		return;
	}

	/* Validate received message */
	message_valid = validate_received_message(received_message, expected_message);

	if (message_valid) {
		/* =====================================================================
		 * SUCCESS: Correct message received
		 * =====================================================================
		 */

		/* Green LED slow blinks (5 complete cycles) to indicate success */
		for (int i = 0; i < 5; i++) {
			led_on(&leds->green);
			delay_ms(400);
			led_off(&leds->green);
			delay_ms(400);
		}

		delay_ms(500);  // Pause before celebration

		/* Success celebration pattern */
		success_celebration(leds);

		/* Blue LED indicates overall success */
		led_on(&leds->blue);
		delay_ms(2000);  // Hold success indicator for 2 seconds
		led_off(&leds->blue);

	} else {
		/* =====================================================================
		 * FAILURE: Wrong message received
		 * =====================================================================
		 */
		wrong_message_error(leds);
	}

	/* Turn off all LEDs */
	all_leds_off(leds);

	/* Re-enable button interrupt for next test */
	GPIO_IRQInterruptConfig(EXTI0_IRQn, ENABLE);
}

int main(void)
{
	SPI_Handle_t spi1_handle = {0};
	MultiLED_Handle_t status_leds = {0};
	Button_Handle_t user_button = {0};

	/* Initialize all peripherals */
	multi_led_init(&status_leds);
	button_init(&user_button);

	/* Signal system startup */
	startup_pattern(&status_leds);
	delay_ms(500);

	/* Configure GPIO pins for SPI1 with software NSS (CS on PB6) */
	spi1_gpio_init();

	/* =========================================================================
	 * Main Master Rx Loop
	 * Orange LED stays on continuously until button is pressed
	 * =========================================================================
	 */
	while (1) {
		/* Signal ready for button press - persistent indication */
		led_on(&status_leds.orange);

		/* Run test cycle - this will turn off Orange LED when button pressed */
		spi_master_rx_test(&spi1_handle, &status_leds, &user_button);

		/* Brief pause before showing ready state again */
		delay_ms(1000);
	}
}

/* =============================================================================
 * EXTI0 Interrupt Handler
 * =============================================================================
 */
void EXTI0_IRQHandler(void)
{
	/* Clear EXTI pending register first */
	GPIO_IRQHandling(0);

	/* Minimal ISR - just set flag if button is pressed */
	if (GPIO_ReadFromInputPin(GPIOA, 0) == GPIO_PIN_SET) {
		if (g_button_handle != NULL) {
			g_button_handle->pressed = 1;  // Set flag for main loop
		}
	}
}

/* =============================================================================
 * SOFTWARE NSS IMPLEMENTATION NOTES
 * =============================================================================
 *
 * ADVANTAGES OVER HARDWARE NSS:
 * ✓ No PA4 hardware conflicts or timing issues
 * ✓ Complete control over CS timing
 * ✓ Can use any GPIO pin for CS
 * ✓ Easier to debug with logic analyzer
 * ✓ More predictable behavior
 * ✓ Works identically on all STM32 variants
 *
 * TIMING SEQUENCE:
 * 1. Master: CS LOW (assert) → PB6 = 0
 * 2. Master: Wait 10ms (slave detection time)
 * 3. Slave: Detects CS LOW, arms SPI_SendDataIT()
 * 4. Slave: First byte pre-loaded into DR
 * 5. Master: Starts clocking (SPI_ReceiveData)
 * 6. Transfer: 18 bytes exchanged
 * 7. Master: CS HIGH (deassert) → PB6 = 1
 *
 * WHY 10MS DELAY WORKS:
 * - Slave polls CS every 5ms in main loop
 * - Worst case: 5ms to detect + 1ms to arm = 6ms
 * - 10ms provides comfortable margin
 * - Can be reduced to 5ms if needed (test first)
 *
 * =============================================================================
 * HARDWARE CONNECTIONS
 * =============================================================================
 *
 * Physical Wiring (Master to Slave):
 *
 *   Master Board              Slave Board
 *   ────────────              ───────────
 *   PA5 (SCK)  ────────────► PA5 (SCK)     2MHz clock
 *   PA6 (MISO) ◄──────────── PA6 (MISO)    Data: slave → master
 *   PA7 (MOSI) ────────────► PA7 (MOSI)    (unused in RX-only)
 *   PB6 (CS)   ────────────► PB6 (CS)      Manual chip select
 *   GND        ──────────────GND            Common ground
 *
 * CRITICAL: All 5 connections must be made for proper operation!
 *
 * Signal Levels:
 * - All signals: 3.3V CMOS logic
 * - CS idle: HIGH (3.3V) = slave not selected
 * - CS active: LOW (0V) = slave selected
 * - Keep wires short (<20cm) for 2MHz operation
 *
 * =============================================================================
 * LED STATUS REFERENCE GUIDE
 * =============================================================================
 *
 * STARTUP SEQUENCE:
 * - Rotating pattern: Orange→Green→Red→Blue (2 cycles)
 * - Orange ON (continuous): Ready state, waiting for button press
 *
 * BUTTON PRESS CYCLE:
 * - Blue ON (300ms): Button press interrupt detected
 * - Orange flashing (3x): SPI initialization in progress
 * - Blue ON (200ms): Asserting CS to trigger slave
 * - Green ON (solid): Receiving message from slave
 * - (Green OFF when reception complete)
 *
 * SUCCESS RESULTS:
 * - Green slow blinks (5x): Message received and validated successfully
 * - All LEDs flash together (3x): Success celebration
 * - Blue ON (2 sec): Overall reception success
 * - Orange ON (continuous): Return to ready state
 *
 * ERROR INDICATIONS:
 * - Orange + Red together (5x rapid): SPI initialization or communication error
 * - Red blinks (3x slow): No message received (empty buffer)
 * - Orange + Red alternate (3x): Wrong message received (validation failed)
 *
 * =============================================================================
 * SPI CONFIGURATION SUMMARY
 * =============================================================================
 *
 * Peripheral: SPI1
 * Bus: APB2 (up to 84 MHz)
 * Mode: Master, Full-duplex
 * Data Frame: 8-bit (DFF = 0)
 * Clock: 2 MHz (APB2_CLK/8 = 16MHz/8)
 * Clock Polarity: CPOL=0 (idle low)
 * Clock Phase: CPHA=0 (sample on first/rising edge)
 * Bit Order: MSB first
 * NSS Management: Software (manual CS control via PB6)
 * Frame Format: Motorola (standard SPI)
 * CRC: Disabled
 *
 * Expected Message: "Hello from Slave!"
 * Message Length: 18 characters (excluding null terminator)
 *
 * =============================================================================
 * TROUBLESHOOTING GUIDE
 * =============================================================================
 *
 * SYMPTOM: Orange LED never turns ON at startup
 * CAUSE: LED initialization failed or hardware issue
 * FIX: Check LED connections, verify GPIO clock enabled
 *
 * SYMPTOM: Button press not detected
 * CAUSE: EXTI interrupt not configured or button wiring issue
 * FIX: Verify PA0 connection, check EXTI/NVIC configuration
 *
 * SYMPTOM: Orange + Red error pattern after button press
 * CAUSE: SPI initialization failed
 * FIX: Check SPI clock enabled, verify SPI_Init() return value in debugger
 *
 * SYMPTOM: No message received (Red blinks 3x)
 * CAUSE: Slave not responding or wiring issue
 * FIX:
 *   1. Verify all 5 wires connected (SCK, MISO, MOSI, CS, GND)
 *   2. Check slave is powered and running (Green LED ON on slave)
 *   3. Probe CS line with multimeter (should pulse LOW during transfer)
 *   4. Use logic analyzer to verify clock and data signals
 *
 * SYMPTOM: Wrong message received (Orange + Red alternate)
 * CAUSE: Timing issue, noise, or slave not synchronized
 * FIX:
 *   1. Check wiring quality (short wires, good connections)
 *   2. Verify slave detects CS assertion (slave Blue LED should blink)
 *   3. Increase CS assertion delay from 10ms to 20ms
 *   4. Check ground connection quality
 *   5. Inspect received_message[] in debugger at breakpoint
 *
 * SYMPTOM: First byte wrong, rest OK (e.g., "Xello from Slave!")
 * CAUSE: Slave not ready when master starts clocking
 * FIX: Increase delay after CS assertion from 10ms to 20ms
 *
 * SYMPTOM: Random garbage received
 * CAUSE: Slave SPI not initialized or wrong SPI configuration
 * FIX:
 *   1. Verify slave SPI initialized (Orange flashes on slave)
 *   2. Check CPOL/CPHA match on both sides (both CPOL=0, CPHA=0)
 *   3. Verify slave is in correct SPI mode (slave, not master)
 *
 * SYMPTOM: All bytes are 0xFF
 * CAUSE: Slave not transmitting (MISO not driven)
 * FIX:
 *   1. Verify slave calls SPI_SendDataIT() when CS detected
 *   2. Check slave's SPI1 interrupt enabled in NVIC
 *   3. Verify SPI1_IRQHandler() is defined on slave
 *   4. Check MISO wire connection
 *
 * =============================================================================
 * DEBUGGING WITH BREAKPOINTS
 * =============================================================================
 *
 * Key breakpoint location marked in code with comment: // ← BREAKPOINT
 *
 * At this breakpoint, inspect:
 * - received_message[] array: Should contain "Hello from Slave!"
 * - rx_byte_0 through rx_byte_17: Individual character values
 * - status: Should be SPI_OK (0)
 * - spi_handle.regs->SR: SPI status register (check for errors)
 *
 * Common values to check:
 * - rx_byte_0 should be 'H' (0x48)
 * - rx_byte_1 should be 'e' (0x65)
 * - rx_byte_17 should be '!' (0x21)
 *
 * If seeing shifted data (e.g., "HHello..."):
 * - Timing issue with slave arming
 * - Increase CS assertion delay
 *
 * =============================================================================
 * PERFORMANCE CHARACTERISTICS
 * =============================================================================
 *
 * Transfer time for 18 bytes at 2 MHz:
 * - Bit time: 0.5 microseconds
 * - Byte time: 4 microseconds (8 bits)
 * - Total data transfer: 72 microseconds
 * - CS assertion delay: 10 milliseconds
 * - Total transaction time: ~10.1 milliseconds
 *
 * CPU usage:
 * - Master: Blocks during transfer (blocking mode)
 * - Slave: Minimal (interrupt-driven)
 *
 * Maximum transfer rate:
 * - Limited by 10ms CS assertion delay
 * - ~100 transactions per second maximum
 * - Can be improved by reducing delay (test first)
 *
 * =============================================================================
 * NEXT STEPS / ENHANCEMENTS
 * =============================================================================
 *
 * Once working reliably:
 * 1. Reduce CS assertion delay from 10ms to 5ms (test thoroughly)
 * 2. Implement master interrupt-driven RX (non-blocking)
 * 3. Add DMA support for faster/more efficient transfers
 * 4. Implement bidirectional communication (master TX + RX)
 * 5. Add CRC checking for data integrity
 * 6. Increase baud rate to 4 MHz or higher
 * 7. Support variable-length messages
 * 8. Add timeout handling for slave non-response
 *
 * =============================================================================
 */
