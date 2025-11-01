/* =============================================================================
 * Project  : STM32F407G-DISC1 SPI Driver Exercise - Slave Tx EXTI-Based
 * File     : spi_slave_tx_interrupt.c
 * Purpose  : Test SPI interrupt-driven transmission with EXTI CS detection
 *
 * Exercise Requirements:
 *   1. Test SPI_SendDataIT API to transmit predefined message to Master
 *   2. SPI-1 Slave mode (PA5-PA7 for SPI signals)
 *   3. DFF = 0 (8-bit data frame)
 *   4. Software slave management (SSM = 1)
 *   5. SCLK provided by Master (2MHz expected)
 *   6. EXTI interrupt on PB6 for zero-latency CS detection
 *   7. LED feedback for status indication
 *
 * Hardware Setup:
 *   - Two STM32F407G-DISC1 boards (Master and Slave)
 *   - SPI1 pins: PA5(SCK), PA6(MISO), PA7(MOSI)
 *   - CS Control: PB6 (Master output → Slave EXTI input)
 *   - LEDs: PD12(Green), PD13(Orange), PD14(Red), PD15(Blue)
 *
 * ARCHITECTURE - EXTI Interrupt-Driven CS Detection:
 *   - PB6 configured as EXTI interrupt (falling edge)
 *   - Master asserts CS (PB6 LOW) → EXTI interrupt fires immediately
 *   - EXTI ISR calls SPI_SendDataIT() to arm transmission
 *   - Master starts clocking → SPI TXE interrupts handle byte transmission
 *   - Main loop detects completion and shows status
 *   - Zero latency, perfect synchronization
 *
 * ┌────────────────────────────────────────────────────────────────────────────
 * │            EXTI-DRIVEN SLAVE OPERATION FLOWCHART
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
 *   ┌────────────────     Orange LED flashing (3x)
 *   │ INITIALIZE SPI │     SPI configured and enabled
 *   └──────┬─────────┘
 *          │
 *          ▼
 *   ┌────────────────     Configure NVIC
 *   │ SETUP NVIC     │     Enable SPI1_IRQn
 *   │ FOR SPI1       │     Enable EXTI9_5_IRQn
 *   └──────┬─────────┘
 *          │
 *          ▼
 *   ┌────────────────     Configure EXTI
 *   │ SETUP PB6      │     Falling edge trigger
 *   │ EXTI           │     High priority (3)
 *   └──────┬─────────┘
 *          │
 *          ▼
 *   ┌────────────────     Green LED ON (continuous)
 *   │ READY STATE    │     ↕
 *   │ WAITING FOR CS │     Waiting for EXTI interrupt
 *   └──────┬─────────┘
 *          │
 *          ▼                    ┌──── MASTER ASSERTS CS ────┐
 * ╔═══════════════════════════════════════════════════════════════════════════
 * ║              EXTI INTERRUPT-DRIVEN TRANSMISSION
 * ╚═══════════════════════════════════════════════════════════════════════════
 *          │
 *   PB6 goes LOW → EXTI6 interrupt fires (EXTI9_5_IRQHandler)
 *          │
 *          ▼
 *   ┌────────────────────────────────────────────────────────────────────┐
 *   │            EXTI9_5_IRQHandler() Called                             │
 *   │                                                                     │
 *   │  1. Clear EXTI pending bit for line 6                              │
 *   │  2. Call SPI_SendDataIT(&g_spi1_handle, msg, len)                  │
 *   │  3. Driver arms transmission, enables TXEIE                        │
 *   │  4. Return immediately (ISR complete in <1μs)                      │
 *   └─────────────────────┬──────────────────────────────────────────────┘
 *                         │
 *   Master starts clocking → TXE interrupts fire
 *                         │
 *                         ▼
 *   ┌────────────────────────────────────────────────────────────────────┐
 *   │            SPI1_IRQHandler() Called (Repeatedly)                   │
 *   │                                                                     │
 *   │  Hardware generates TXE interrupt for each byte                    │
 *   │  ISR sends bytes one-by-one synchronized to master clock           │
 *   │  Repeats for all 18 bytes                                          │
 *   └─────────────────────┬──────────────────────────────────────────────┘
 *                         │
 *                         ▼
 *   ┌────────────────────────────────────────────────────────────────────┐
 *   │         After Last Byte Transmitted                                │
 *   │                                                                     │
 *   │  ISR detects: txRemaining == 0                                     │
 *   │  Actions:                                                          │
 *   │    • Disables TXEIE                                                │
 *   │    • Sets TX_DONE flag                                             │
 *   │    • Returns to main loop                                          │
 *   └─────────────────────┬──────────────────────────────────────────────┘
 *                         │
 *                         ▼
 *   ┌────────────────────────────────────────────────────────────────────┐
 *   │         Main Loop Detects Completion                               │
 *   │                                                                     │
 *   │  Checks: SPI_FLAG_TX_DONE set?                                     │
 *   │  Actions:                                                          │
 *   │    • Clears TX_DONE flag                                           │
 *   │    • Shows success LED pattern                                     │
 *   │    • Returns to ready state                                        │
 *   └─────────────────────┬──────────────────────────────────────────────┘
 *                         │
 *                         ▼
 *             Back to READY STATE (Green LED ON)
 *                         │
 *                         └──► Wait for next EXTI interrupt
 *
 * ┌────────────────────────────────────────────────────────────────────────────
 * │                        LED STATUS MAPPING
 * └────────────────────────────────────────────────────────────────────────────
 * Orange (PD13): SPI Initialization Status
 * Green  (PD12): Ready State - ON when waiting for CS
 * Red    (PD14): Error indication
 * Blue   (PD15): Transmission success
 *
 * ┌────────────────────────────────────────────────────────────────────────────
 * │                      SPI COMMUNICATION DETAILS
 * └────────────────────────────────────────────────────────────────────────────
 * Message: "Hello from Slave!" (18 characters)
 * SPI Mode: Slave mode responding to master clock
 * Data Frame: 8-bit (DFF = 0)
 * NSS Control: Software management (SSM = 1) - CS via PB6 EXTI
 * Communication: Interrupt-driven transmission synchronized to master
 * Response: EXTI arms transmission, SPI ISR sends bytes
 *
 * Connection Requirements:
 * Slave STM32           Master STM32
 * -----------          -----------
 * PA5 (SCK)     <--    PA5 (SCK)      Serial clock from master
 * PA6 (MISO)    -->    PA6 (MISO)     Data from slave to master
 * PA7 (MOSI)    <--    PA7 (MOSI)     Data to slave (unused)
 * PB6 (CS)      <--    PB6 (CS)       Chip select via EXTI
 * GND           ---    GND            Common ground ESSENTIAL
 *
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include <string.h>

/* Predefined message to transmit to master */
static const uint8_t slave_message[] = "Hello from Slave!";
static const uint16_t message_length = sizeof(slave_message) - 1;

/* Global SPI handle for interrupt handler access */
static SPI_Handle_t g_spi1_handle = {0};

/* LED Handle Structure */
typedef struct {
	GPIO_PinHandle_t orange;    // PD13 - SPI initialization status
	GPIO_PinHandle_t green;     // PD12 - Ready state indicator
	GPIO_PinHandle_t red;       // PD14 - Error indication
	GPIO_PinHandle_t blue;      // PD15 - Transmission success
} MultiLED_Handle_t;

/* Global LED handle */
static MultiLED_Handle_t status_leds = {0};

/**
 * @brief  Software delay function
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
 * @brief  Configure GPIO pins for SPI1 (Software NSS)
 */
static void spi1_gpio_init(void)
{
	GPIO_PinHandle_t spi_pin;

	/* Configure SPI signal pins (SCK, MISO, MOSI) */
	spi_pin.config.port = GPIO_PORT_A;
	spi_pin.config.mode = GPIO_MODE_AF;
	spi_pin.config.otype = GPIO_OTYPE_PP;
	spi_pin.config.pull = GPIO_NOPULL;
	spi_pin.config.speed = GPIO_SPEED_FAST;
	spi_pin.config.af = 5;

	for (uint8_t pin = 5; pin <= 7; pin++) {
		spi_pin.config.pin = pin;
		GPIO_Init(&spi_pin);
	}
}

/**
 * @brief  Configure PB6 as EXTI interrupt for CS detection
 */
static void cs_exti_init(void)
{
	GPIO_PinHandle_t cs_pin;

	/* Configure PB6 as input with pull-up */
	cs_pin.config.port = GPIO_PORT_B;
	cs_pin.config.pin = 6;
	cs_pin.config.mode = GPIO_MODE_INPUT;
	cs_pin.config.pull = GPIO_PULLUP;  // CS idle HIGH
	cs_pin.config.speed = GPIO_SPEED_FAST;
	cs_pin.config.af = 0;

	GPIO_Init(&cs_pin);

	/* Configure EXTI on PB6 - falling edge trigger (CS assertion) */
	GPIO_ConfigEXTI(&cs_pin, GPIO_EXTI_TRIGGER_FALLING, ENABLE);

	/* Enable EXTI9_5 interrupt (PB6 = EXTI line 6) */
	GPIO_IRQInterruptConfig(EXTI9_5_IRQn, ENABLE);

	/* Set high priority for immediate CS response */
	GPIO_IRQPriorityConfig(EXTI9_5_IRQn, 3);
}

/**
 * @brief  Configure all LEDs
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

	uint8_t pins[] = {13, 12, 14, 15};
	GPIO_PinHandle_t *led_handles[] = {
		&leds->orange, &leds->green, &leds->red, &leds->blue
	};

	for (int i = 0; i < 4; i++) {
		led_handles[i]->config = led_config;
		led_handles[i]->config.pin = pins[i];
		GPIO_Init(led_handles[i]);
	}
}

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
 * @brief  Startup pattern
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

/**
 * @brief  Success pattern
 */
static void transmission_success_pattern(MultiLED_Handle_t *leds)
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
}

/**
 * @brief  Error pattern
 */
static void spi_init_error_pattern(MultiLED_Handle_t *leds)
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
 * @brief  Configure SPI1 in Slave mode (Software NSS)
 */
static SPI_Status_t spi1_slave_tx_interrupt_init(SPI_Handle_t *spi_handle)
{
	spi_handle->dev = SPI_DEVICE_SPI1;

	spi_handle->cfg.mode = SPI_MODE_SLAVE;
	spi_handle->cfg.bus = SPI_BUS_FULL_DUPLEX;
	spi_handle->cfg.datasize = SPI_DFF_8BIT;
	spi_handle->cfg.cpol = SPI_CPOL_0;
	spi_handle->cfg.cpha = SPI_CPHA_0;
	spi_handle->cfg.firstBit = SPI_FIRSTBIT_MSB;
	spi_handle->cfg.baudDiv = SPI_BR_DIV2;
	spi_handle->cfg.nss = SPI_NSS_SOFTWARE;
	spi_handle->cfg.tiMode = 0;
	spi_handle->cfg.crcEnable = 0;
	spi_handle->cfg.crcPolynomial = 7;

	return SPI_Init(spi_handle);
}

/**
 * @brief  Enter ready state
 */
static void enter_ready_state(MultiLED_Handle_t *leds)
{
	all_leds_off(leds);
	led_on(&leds->green);
}

int main(void)
{
	SPI_Status_t init_status;

	/* Initialize LEDs */
	multi_led_init(&status_leds);
	startup_pattern(&status_leds);
	delay_ms(500);

	/* Configure SPI GPIO */
	spi1_gpio_init();

	/* Configure CS EXTI */
	cs_exti_init();

	/* Initialize SPI */
	for (int i = 0; i < 3; i++) {
		led_on(&status_leds.orange);
		delay_ms(200);
		led_off(&status_leds.orange);
		delay_ms(200);
	}

	init_status = spi1_slave_tx_interrupt_init(&g_spi1_handle);
	if (init_status != SPI_OK) {
		while (1) {
			spi_init_error_pattern(&status_leds);
			delay_ms(1000);
		}
	}

	/* Enable SPI peripheral */
	SPI_PeripheralControl(&g_spi1_handle, ENABLE);

	/* Enable SPI1 interrupt */
	SPI_IRQInterruptConfig(SPI1_IRQn, ENABLE);
	SPI_IRQPriorityConfig(SPI1_IRQn, 5);

	delay_ms(100);

	/* Enter ready state */
	enter_ready_state(&status_leds);

	/* Main loop - detect completion */
	while (1) {
		if (SPI_GetFlags(&g_spi1_handle) & SPI_FLAG_TX_DONE) {
			SPI_ClearFlags(&g_spi1_handle, SPI_FLAG_TX_DONE);

			led_off(&status_leds.green);
			transmission_success_pattern(&status_leds);
			enter_ready_state(&status_leds);
		}

		delay_ms(10);
	}
}

/**
 * @brief  SPI1 Interrupt Handler
 */
void SPI1_IRQHandler(void)
{
	SPI_IRQHandling(&g_spi1_handle);
}

/**
 * @brief  EXTI9_5 Interrupt Handler (handles EXTI lines 5-9, including PB6)
 */
void EXTI9_5_IRQHandler(void)
{
	/* Check if EXTI line 6 (PB6) triggered the interrupt */
	if (EXTI->PR & (1 << 6)) {
		/* Clear EXTI6 pending bit */
		EXTI->PR = (1 << 6);

		/* Arm SPI transmission immediately */
		SPI_SendDataIT(&g_spi1_handle,
		               (uint8_t*)slave_message,
		               message_length);
	}
}
