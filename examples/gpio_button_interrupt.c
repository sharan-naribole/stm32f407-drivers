

// main_exti_falling.c
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

static void isr_delay_us(volatile uint32_t t);

int main(void)
{
	/* ---- LED: PD12 output, push-pull ---- */
	GPIO_PinHandle_t led = {0};
	led.config.port  = GPIO_PORT_D;
	led.config.pin   = 12;
	led.config.mode  = GPIO_MODE_OUTPUT;
	led.config.otype = GPIO_OTYPE_PP;
	led.config.pull  = GPIO_NOPULL;
	led.config.speed = GPIO_SPEED_LOW;
	led.config.af    = 0;
	GPIO_Init(&led);

	/* ---- Button: PA0 input ----
	 * DISC1 has an external pulldown on PA0 (idle LOW, press = HIGH).
	 */
	GPIO_PinHandle_t btn = {0};
	btn.config.port  = GPIO_PORT_A;
	btn.config.pin   = 0;
	btn.config.mode  = GPIO_MODE_INPUT;
	btn.config.otype = GPIO_OTYPE_PP;   // ignored in input mode
	btn.config.pull  = GPIO_NOPULL;     // external pulldown on board
	btn.config.speed = GPIO_SPEED_LOW;
	btn.config.af    = 0;
	GPIO_Init(&btn);

	/* ---- Route PA0 -> EXTI0, select FALLING edge, unmask ----
	 * With DISC1 wiring, this will trigger on BUTTON RELEASE.
	 */
	GPIO_ConfigEXTI(&btn, GPIO_EXTI_TRIGGER_FALLING, ENABLE);

	/* ---- NVIC: priority + enable EXTI0 ---- */
	GPIO_IRQPriorityConfig(EXTI0_IRQn, 5);
	GPIO_IRQInterruptConfig(EXTI0_IRQn, ENABLE);

	/* idle: main can sleep, do other work, or just loop */
	while (1) {
		__asm volatile("wfi");  // wait-for-interrupt (optional low-power)
	}
}

static void isr_delay_us(volatile uint32_t t)
{
	while (t--) { __asm volatile("nop"); }
}

/* ---- EXTI0 ISR: clear pending + toggle LED ---- */
void EXTI0_IRQHandler(void)
{
	/* small settle time for bounce */
	isr_delay_us(100000U);       /* ~few ms depending on CPU clock */

	/* Re-read the line; accept only if still LOW (falling-edge scenario) */
	if (GPIO_ReadFromInputPin(GPIOA, 0) == GPIO_PIN_RESET) {
		GPIO_ToggleOutputPin(GPIOD, 12);
	}

	GPIO_IRQHandling(0);         /* clear EXTI pending */
}
