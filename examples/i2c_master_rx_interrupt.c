/* =============================================================================
 * Project  : STM32F407G-DISC1 I2C Driver Exercise - Master Rx Interrupt (Non-blocking)
 * File     : 011i2c_master_rx_interrupt.c
 * Purpose  : Test I2C_MasterReceiveDataIT API with interrupt-driven reception
 *
 * Exercise Requirements:
 *   1. Test I2C_MasterReceiveDataIT API (non-blocking reception)
 *   2. I2C-1 Master mode (PB6-PB7 pins)
 *   3. SCL = 100 kHz, 7-bit addressing, slave 0x68
 *   4. Interrupt-driven with event and error handlers
 *   5. Message > 2 bytes (28 bytes)
 *
 * NON-BLOCKING RECEPTION:
 *   - I2C_MasterReceiveDataIT() starts reception and returns
 *   - ISR handles ACK/NACK and byte reception
 *   - Callback notifies completion
 *   - CPU free during reception
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

#define SLAVE_ADDR  0x68

static I2C_Handle_t g_i2c1_handle = {0};
static volatile uint8_t rx_complete = 0;
static volatile uint8_t rx_error = 0;

typedef struct {
    GPIO_PinHandle_t orange, green, red, blue;
} MultiLED_Handle_t;

static MultiLED_Handle_t status_leds = {0};

static void delay_ms(volatile uint32_t ms)
{
    volatile uint32_t cycles = ms * 16800;
    while (cycles--) {
        __asm volatile("nop");
    }
}

static void i2c1_gpio_init(void)
{
    GPIO_PinHandle_t i2c_pin;
    i2c_pin.config.port = GPIO_PORT_B;
    i2c_pin.config.mode = GPIO_MODE_AF;
    i2c_pin.config.otype = GPIO_OTYPE_OD;
    i2c_pin.config.pull = GPIO_PULLUP;
    i2c_pin.config.speed = GPIO_SPEED_HIGH;
    i2c_pin.config.af = 4;

    i2c_pin.config.pin = 6;  // SCL
    GPIO_Init(&i2c_pin);
    i2c_pin.config.pin = 7;  // SDA
    GPIO_Init(&i2c_pin);
}

static void multi_led_init(MultiLED_Handle_t *leds)
{
    GPIO_PinConfig_t led_config = {
        .port = GPIO_PORT_D, .mode = GPIO_MODE_OUTPUT,
        .otype = GPIO_OTYPE_PP, .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_LOW, .af = 0
    };

    leds->orange.config = led_config; leds->orange.config.pin = 13; GPIO_Init(&leds->orange);
    leds->green.config = led_config; leds->green.config.pin = 12; GPIO_Init(&leds->green);
    leds->red.config = led_config; leds->red.config.pin = 14; GPIO_Init(&leds->red);
    leds->blue.config = led_config; leds->blue.config.pin = 15; GPIO_Init(&leds->blue);
}

static void led_on(GPIO_PinHandle_t *led) { GPIO_WriteToOutputPin(led->port, led->config.pin, GPIO_PIN_SET); }
static void led_off(GPIO_PinHandle_t *led) { GPIO_WriteToOutputPin(led->port, led->config.pin, GPIO_PIN_RESET); }
static void all_leds_off(MultiLED_Handle_t *leds) { led_off(&leds->orange); led_off(&leds->green); led_off(&leds->red); led_off(&leds->blue); }

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
        led_on(&leds->orange); led_on(&leds->green); led_on(&leds->red); led_on(&leds->blue);
        delay_ms(200);
        all_leds_off(leds);
        delay_ms(200);
    }
    for (int i = 0; i < 3; i++) {
        led_on(&leds->green); delay_ms(300);
        led_off(&leds->green); delay_ms(300);
    }
}

static void error_pattern(MultiLED_Handle_t *leds)
{
    all_leds_off(leds);
    for (int i = 0; i < 5; i++) {
        led_on(&leds->red); delay_ms(150);
        led_off(&leds->red); delay_ms(150);
    }
}

static I2C_Status_t i2c1_master_rx_init(I2C_Handle_t *i2c_handle)
{
    i2c_handle->dev = I2C_DEVICE_I2C1;
    i2c_handle->cfg.sclSpeed = I2C_SPEED_SM;
    i2c_handle->cfg.ownAddress = 0x00;
    i2c_handle->cfg.addrMode = I2C_ADDR_7BIT;
    i2c_handle->cfg.ackControl = I2C_ACK_ENABLE;
    i2c_handle->cfg.fmDutyCycle = I2C_FM_DUTY_2;
    return I2C_Init(i2c_handle);
}

static uint8_t validate_received_data(uint8_t *data, uint16_t length)
{
    if (length != expected_length) return 0;
    return (memcmp(data, expected_message, length) == 0);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle, I2C_Event_t event)
{
    if (pHandle->pI2Cx == I2C1) {
        switch (event) {
            case I2C_EVENT_RX_CMPLT:
                rx_complete = 1;
                rx_error = 0;
                break;
            case I2C_ERROR_AF:
            case I2C_ERROR_BERR:
            case I2C_ERROR_ARLO:
            case I2C_ERROR_OVR:
            case I2C_ERROR_TIMEOUT:
                rx_complete = 0;
                rx_error = 1;
                break;
            default:
                break;
        }
    }
}

void I2C1_EV_IRQHandler(void) { I2C_EV_IRQHandling(&g_i2c1_handle); }
void I2C1_ER_IRQHandler(void) { I2C_ER_IRQHandling(&g_i2c1_handle); }

static void i2c_master_rx_interrupt_cycle(MultiLED_Handle_t *leds)
{
    I2C_Status_t status;

    rx_complete = 0;
    rx_error = 0;

    led_on(&leds->orange); delay_ms(500); led_off(&leds->orange);

    memset(rx_buffer, 0, sizeof(rx_buffer));

    led_on(&leds->blue); delay_ms(100);

    status = I2C_MasterReceiveDataIT(&g_i2c1_handle, rx_buffer, expected_length,
                                     SLAVE_ADDR, I2C_DISABLE_SR);

    if (status != I2C_OK) {
        led_off(&leds->blue);
        error_pattern(leds);
        all_leds_off(leds);
        return;
    }

    /* Wait for completion while blinking blue */
    uint32_t timeout = 1000;
    while (!rx_complete && !rx_error && timeout > 0) {
        led_off(&leds->blue); delay_ms(50);
        led_on(&leds->blue); delay_ms(50);
        timeout -= 100;
    }

    led_off(&leds->blue);

    if (rx_complete) {
        if (validate_received_data(rx_buffer, expected_length)) {
            success_pattern(leds);
        } else {
            error_pattern(leds);
        }
    } else {
        error_pattern(leds);
    }

    all_leds_off(leds);
}

int main(void)
{
    multi_led_init(&status_leds);
    startup_pattern(&status_leds);
    delay_ms(500);

    i2c1_gpio_init();

    I2C_IRQInterruptConfig(I2C1_EV_IRQn, ENABLE);
    I2C_IRQInterruptConfig(I2C1_ER_IRQn, ENABLE);
    I2C_IRQPriorityConfig(I2C1_EV_IRQn, 5);
    I2C_IRQPriorityConfig(I2C1_ER_IRQn, 5);

    for (int i = 0; i < 3; i++) {
        led_on(&status_leds.orange); delay_ms(200);
        led_off(&status_leds.orange); delay_ms(200);
    }

    if (i2c1_master_rx_init(&g_i2c1_handle) != I2C_OK) {
        while (1) { error_pattern(&status_leds); delay_ms(1000); }
    }

    I2C_PeripheralControl(g_i2c1_handle.pI2Cx, ENABLE);
    delay_ms(100);

    led_on(&status_leds.green); delay_ms(1000); led_off(&status_leds.green);

    while (1) {
        i2c_master_rx_interrupt_cycle(&status_leds);
        delay_ms(3000);
    }
}

/* =============================================================================
 * NON-BLOCKING MASTER RX OPERATION:
 *
 * 1. I2C_MasterReceiveDataIT() starts reception, enables interrupts, returns
 * 2. ISR handles START, address, ACK/NACK control, byte reception
 * 3. For N>2 bytes: ISR manages ACK until last byte, then NACK
 * 4. ISR generates STOP after last byte
 * 5. Callback sets rx_complete flag
 * 6. Main loop detects completion and validates data
 *
 * BENEFITS:
 * - CPU free during reception
 * - Can perform other tasks
 * - More efficient than blocking
 * =============================================================================
 */
