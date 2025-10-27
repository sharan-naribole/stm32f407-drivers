/* =============================================================================
 * Project  : STM32F407G-DISC1 Bare-Metal GPIO Driver Test
 * File     : led_blink.c
 * Purpose  : Demonstrate a simple GPIO output by toggling an on-board LED
 *            with a software delay loop.
 *
 * Scenario :
 *   - Hardware: STM32F407G-DISC1 board
 *   - On-board LED (green) is connected to PD12
 *
 *   The program:
 *     1) Configures PD12 as an OUTPUT.
 *     2) Enters an infinite loop.
 *     3) Toggles PD12 using GPIO_ToggleOutputPin().
 *     4) Waits for a crude software delay.
 *
 * Key Learnings:
 *   - How to set up a GPIO pin as an output.
 *   - Difference between Push-Pull vs Open-Drain drive:
 *       * Push-Pull (normal LED driving, strong HIGH and LOW)
 *       * Open-Drain (needs external or internal pull-up; can only pull LOW)
 *   - Why busy-wait delays are not suitable for precise timing.
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

/* -----------------------------------------------------------------------------
 * Busy-wait delay function
 *   - Burns CPU cycles in a loop.
 *   - Purpose: slow down toggling so the LED blink is visible.
 *   - NOT accurate timing. Use SysTick/Timer for precise control.
 * ---------------------------------------------------------------------------*/
static void delay(volatile uint32_t t)
{
    while (t--) {
        __asm volatile("nop"); // 1 cycle no-operation
    }
}

/* -----------------------------------------------------------------------------
 * Configure PD12 LED in Push-Pull mode
 * ---------------------------------------------------------------------------*/
static void led_config_pushpull(GPIO_PinHandle_t *h)
{
    h->config.port  = GPIO_PORT_D;
    h->config.pin   = 12;
    h->config.mode  = GPIO_MODE_OUTPUT;
    h->config.otype = GPIO_OTYPE_PP;     // Push-Pull
    h->config.pull  = GPIO_NOPULL;
    h->config.speed = GPIO_SPEED_LOW;
    h->config.af    = 0;
    GPIO_Init(h);
}

/* -----------------------------------------------------------------------------
 * Configure PD12 LED in Open-Drain mode (with pull-up)
 *   - Note: On DISC1 board, open-drain does not light LED reliably
 *     because the LED is wired to VDD via resistor (active-high).
 *   - This mode is mainly for demonstration (would work if LED
 *     were wired to 3.3V → resistor → LED → GPIO pin).
 * ---------------------------------------------------------------------------*/
static void led_config_opendrain(GPIO_PinHandle_t *h)
{
    h->config.port  = GPIO_PORT_D;
    h->config.pin   = 12;
    h->config.mode  = GPIO_MODE_OUTPUT;
    h->config.otype = GPIO_OTYPE_OD;     // Open-Drain
    h->config.pull  = GPIO_PULLUP;       // enable internal pull-up
    h->config.speed = GPIO_SPEED_LOW;
    h->config.af    = 0;
    GPIO_Init(h);
}

/* -----------------------------------------------------------------------------
 * Blink loop: toggles the LED forever
 * ---------------------------------------------------------------------------*/
static void blink_loop(GPIO_PinHandle_t *h)
{
    while (1) {
        GPIO_ToggleOutputPin(h->port, h->config.pin);
        delay(600000); // crude software delay
    }
}

int main(void)
{
    GPIO_PinHandle_t led = {0};

    /* Select one configuration */
    led_config_pushpull(&led);   // Case 1: Push-Pull
    // led_config_opendrain(&led);  // Case 2: Open-Drain (not effective on DISC1 LED)

    blink_loop(&led);  // never returns
}
