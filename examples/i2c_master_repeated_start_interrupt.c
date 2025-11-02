/* =============================================================================
 * Project  : STM32F407G-DISC1 I2C Driver Exercise - Master Repeated Start Interrupt
 * File     : 012i2c_master_repeated_start_interrupt.c
 * Purpose  : Test I2C Repeated Start with interrupts - send message TWICE
 *
 * Exercise Requirements:
 *   1. Test I2C_MasterSendDataIT with Repeated Start
 *   2. Send same message TWICE in one transaction (interrupt-driven)
 *   3. I2C-1 Master mode (PB6-PB7 pins), SCL = 100kHz, slave 0x68
 *   4. Interrupt-driven with comprehensive LED feedback
 *
 * INTERRUPT + REPEATED START:
 *   - First: I2C_MasterSendDataIT(..., I2C_ENABLE_SR) → Returns, ISR handles
 *   - Callback: I2C_EVENT_TX_CMPLT for first transmission
 *   - Second: I2C_MasterSendDataIT(..., I2C_DISABLE_SR) → Returns, ISR handles
 *   - Callback: I2C_EVENT_TX_CMPLT for second transmission
 *   - All in ONE bus transaction (no STOP between)
 *
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include <string.h>

static const uint8_t tx_message[] = "I2C Test!";
static const uint16_t tx_length = sizeof(tx_message) - 1;  // 10 bytes

#define SLAVE_ADDR  0x68

static I2C_Handle_t g_i2c1_handle = {0};
static volatile uint8_t tx1_complete = 0;
static volatile uint8_t tx2_complete = 0;
static volatile uint8_t tx_error = 0;

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

    i2c_pin.config.pin = 6; GPIO_Init(&i2c_pin);
    i2c_pin.config.pin = 7; GPIO_Init(&i2c_pin);
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
}

static void error_pattern(MultiLED_Handle_t *leds)
{
    all_leds_off(leds);
    for (int i = 0; i < 5; i++) {
        led_on(&leds->red); delay_ms(150);
        led_off(&leds->red); delay_ms(150);
    }
}

static I2C_Status_t i2c1_master_tx_init(I2C_Handle_t *i2c_handle)
{
    i2c_handle->dev = I2C_DEVICE_I2C1;
    i2c_handle->cfg.sclSpeed = I2C_SPEED_SM;
    i2c_handle->cfg.ownAddress = 0x00;
    i2c_handle->cfg.addrMode = I2C_ADDR_7BIT;
    i2c_handle->cfg.ackControl = I2C_ACK_ENABLE;
    i2c_handle->cfg.fmDutyCycle = I2C_FM_DUTY_2;
    return I2C_Init(i2c_handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle, I2C_Event_t event)
{
    if (pHandle->pI2Cx == I2C1) {
        switch (event) {
            case I2C_EVENT_TX_CMPLT:
                if (!tx1_complete) {
                    tx1_complete = 1;  // First transmission done
                } else {
                    tx2_complete = 1;  // Second transmission done
                }
                break;
            case I2C_ERROR_AF:
            case I2C_ERROR_BERR:
            case I2C_ERROR_ARLO:
            case I2C_ERROR_OVR:
            case I2C_ERROR_TIMEOUT:
                tx_error = 1;
                break;
            default:
                break;
        }
    }
}

void I2C1_EV_IRQHandler(void) { I2C_EV_IRQHandling(&g_i2c1_handle); }
void I2C1_ER_IRQHandler(void) { I2C_ER_IRQHandling(&g_i2c1_handle); }

static void i2c_repeated_start_interrupt_cycle(MultiLED_Handle_t *leds)
{
    I2C_Status_t status1, status2;

    tx1_complete = 0;
    tx2_complete = 0;
    tx_error = 0;

    led_on(&leds->orange); delay_ms(500); led_off(&leds->orange);

    /* ========== FIRST TRANSMISSION (with Repeated Start) ========== */
    led_on(&leds->blue); delay_ms(200);

    status1 = I2C_MasterSendDataIT(&g_i2c1_handle, (uint8_t*)tx_message, tx_length,
                                   SLAVE_ADDR, I2C_ENABLE_SR);

    if (status1 != I2C_OK) {
        led_off(&leds->blue);
        error_pattern(leds);
        all_leds_off(leds);
        return;
    }

    /* Wait for first transmission to complete */
    uint32_t timeout1 = 500;
    while (!tx1_complete && !tx_error && timeout1 > 0) {
        led_off(&leds->blue); delay_ms(25);
        led_on(&leds->blue); delay_ms(25);
        timeout1 -= 50;
    }

    led_off(&leds->blue);
    delay_ms(100);
    led_on(&leds->blue);
    delay_ms(100);
    led_off(&leds->blue);

    if (!tx1_complete || tx_error) {
        error_pattern(leds);
        all_leds_off(leds);
        return;
    }

    delay_ms(200);

    /* ========== SECOND TRANSMISSION (with STOP) ========== */
    led_on(&leds->green); delay_ms(200);

    status2 = I2C_MasterSendDataIT(&g_i2c1_handle, (uint8_t*)tx_message, tx_length,
                                   SLAVE_ADDR, I2C_DISABLE_SR);

    if (status2 != I2C_OK) {
        led_off(&leds->green);
        error_pattern(leds);
        all_leds_off(leds);
        return;
    }

    /* Wait for second transmission to complete */
    uint32_t timeout2 = 500;
    while (!tx2_complete && !tx_error && timeout2 > 0) {
        led_off(&leds->green); delay_ms(25);
        led_on(&leds->green); delay_ms(25);
        timeout2 -= 50;
    }

    led_off(&leds->green);
    delay_ms(100);
    led_on(&leds->green);
    delay_ms(100);
    led_off(&leds->green);

    if (tx2_complete && !tx_error) {
        success_pattern(leds);
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

    if (i2c1_master_tx_init(&g_i2c1_handle) != I2C_OK) {
        while (1) { error_pattern(&status_leds); delay_ms(1000); }
    }

    I2C_PeripheralControl(g_i2c1_handle.pI2Cx, ENABLE);
    delay_ms(100);

    led_on(&status_leds.green); delay_ms(1000); led_off(&status_leds.green);

    while (1) {
        i2c_repeated_start_interrupt_cycle(&status_leds);
        delay_ms(3000);
    }
}

/* =============================================================================
 * INTERRUPT + REPEATED START OPERATION:
 *
 * 1. Call I2C_MasterSendDataIT(..., I2C_ENABLE_SR) for first message
 * 2. Function returns immediately, ISR handles transmission
 * 3. Wait for callback to set tx1_complete
 * 4. Call I2C_MasterSendDataIT(..., I2C_DISABLE_SR) for second message
 * 5. Function returns immediately, ISR handles transmission
 * 6. Wait for callback to set tx2_complete
 * 7. Both transmissions in ONE transaction (Sr instead of STOP between)
 *
 * BENEFITS:
 * - Non-blocking operation
 * - Atomic repeated start transaction
 * - CPU free during transmissions
 * - Best of both worlds!
 * =============================================================================
 */
