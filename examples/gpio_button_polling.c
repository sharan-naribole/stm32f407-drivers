/* =============================================================================
 * Project  : STM32F407G-DISC1 Bare-Metal GPIO Driver Test
 * File     : button_toggle_led.c
 * Purpose  : Demonstrate GPIO input + output by toggling an LED
 *            whenever the on-board USER button is pressed.
 *
 * Scenario :
 *   - Hardware: STM32F407G-DISC1 board
 *   - On-board LED (green) is connected to PD12
 *   - On-board USER button (blue) is connected to PA0
 *
 *   The program:
 *     1) Configures PD12 as an OUTPUT (push-pull, low speed).
 *     2) Configures PA0 as an INPUT (no pull — DISC1 has external pulldown).
 *     3) Continuously polls PA0 in the main loop.
 *     4) On each button press (rising edge: low → high), toggles PD12.
 *     5) Uses a crude software delay for debouncing.
 *
 * Key Learnings:
 *   - How to set up a GPIO pin as input vs output.
 *   - How to detect button edges with simple software.
 *   - How to use the driver functions: GPIO_Init(), GPIO_ReadFromInputPin(),
 *     and GPIO_ToggleOutputPin().
 *   - Limitations of polling (CPU busy-wait) and why interrupts are better
 *     for responsive designs (covered in the next step).
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

/* -----------------------------------------------------------------------------
 * Busy-wait delay function
 *   - Burns CPU cycles in a loop.
 *   - Purpose: provide visible LED blink rate & crude debounce.
 *   - NOT accurate timing. Use SysTick/Timer for real applications.
 * ---------------------------------------------------------------------------*/
static void delay(volatile uint32_t t)
{
    while (t--) {
        __asm volatile("nop"); // no operation, just consume 1 cycle
    }
}

int main(void)
{
    /* -------------------------------------------------------------------------
     * Configure LED on PD12
     *   - Port: GPIOD
     *   - Pin : 12
     *   - Mode: Output
     *   - OType: Push-Pull (strong drive high/low)
     *   - Pull : No pull (not needed for output)
     *   - Speed: Low (sufficient for LED)
     * -----------------------------------------------------------------------*/
    GPIO_PinHandle_t led = {0};
    led.config.port  = GPIO_PORT_D;
    led.config.pin   = 12;
    led.config.mode  = GPIO_MODE_OUTPUT;
    led.config.otype = GPIO_OTYPE_PP;
    led.config.pull  = GPIO_NOPULL;
    led.config.speed = GPIO_SPEED_LOW;
    led.config.af    = 0;
    GPIO_Init(&led);

    /* -------------------------------------------------------------------------
     * Configure USER Button on PA0
     *   - Port: GPIOA
     *   - Pin : 0
     *   - Mode: Input
     *   - OType: Push-Pull (ignored in input mode)
     *   - Pull : No pull (board has external pulldown resistor)
     *   - Speed: Low (ignored in input mode)
     * -----------------------------------------------------------------------*/
    GPIO_PinHandle_t btn = {0};
    btn.config.port  = GPIO_PORT_A;
    btn.config.pin   = 0;
    btn.config.mode  = GPIO_MODE_INPUT;
    btn.config.otype = GPIO_OTYPE_PP;   // harmless in input mode
    btn.config.pull  = GPIO_NOPULL;     // DISC1 button already pulled down
    btn.config.speed = GPIO_SPEED_LOW;
    btn.config.af    = 0;
    GPIO_Init(&btn);

    /* -------------------------------------------------------------------------
     * Logic: Poll button state and detect RISING edge
     *   - prev_state remembers last sampled value
     *   - cur_state is the current sample
     *   - If transition 0 → 1 is detected, toggle the LED
     *   - Simple delay is added for debounce
     * -----------------------------------------------------------------------*/
    uint8_t prev_state = GPIO_ReadFromInputPin(btn.port, btn.config.pin);

    while (1) {
        uint8_t cur_state = GPIO_ReadFromInputPin(btn.port, btn.config.pin);

        if (cur_state == GPIO_PIN_SET && prev_state == GPIO_PIN_RESET) {
            // Rising edge detected → button press → toggle LED
            GPIO_ToggleOutputPin(led.port, led.config.pin);
        }

        prev_state = cur_state;  // update history
        delay(40000);            // crude debounce
    }
}
