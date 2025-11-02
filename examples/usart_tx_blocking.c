/* =============================================================================
 * Project  : STM32F407G-DISC1 USART Driver Test - Transmitter
 * File     : 013usart_tx_blocking.c
 * Purpose  : Demonstrate USART communication by sending messages from one
 *            STM32 to another using blocking mode.
 *
 * Hardware Setup:
 *   - Board: STM32F407G-DISC1 (TRANSMITTER side)
 *   - USART2 pins:
 *       • PA2 → USART2_TX (connect to PA3 of RECEIVER board)
 *       • PA3 → USART2_RX (connect to PA2 of RECEIVER board)
 *       • GND → GND (connect grounds of both boards)
 *   - On-board USER button (PA0) to trigger message transmission
 *   - On-board LED (PD12) blinks after each transmission
 *
 * Configuration:
 *   - Baud Rate  : 115200 bps
 *   - Word Length: 8 bits
 *   - Stop Bits  : 1
 *   - Parity     : None
 *   - Flow Control: None
 *
 * Wiring between two STM32F407 boards:
 *   Master (TX)         Slave (RX)
 *   PA2 (TX) ───────► PA3 (RX)
 *   PA3 (RX) ◄─────── PA2 (TX)
 *   GND      ◄──────► GND
 *
 * Scenario:
 *   1) Press USER button on transmitter board
 *   2) Transmitter sends "Hello from STM32!\n"
 *   3) LED blinks to indicate transmission complete
 *   4) Receiver board should receive the message
 *
 * Key Learnings:
 *   - How to configure GPIO pins for USART alternate function
 *   - How to initialize USART peripheral with specific configuration
 *   - How to use blocking transmission APIs
 *   - Peer-to-peer USART communication between two microcontrollers
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_usart_driver.h"
#include <string.h>

/* Message to transmit */
const char msg[] = "Hello from STM32 Transmitter!\n";

/* USART handle */
USART_Handle_t usart2;

/* LED handle for indication */
GPIO_PinHandle_t led;

/* Button handle */
GPIO_PinHandle_t button;

/* -----------------------------------------------------------------------------
 * Busy-wait delay function
 * ---------------------------------------------------------------------------*/
static void delay(volatile uint32_t t)
{
    while (t--) {
        __asm volatile("nop");
    }
}

/* -----------------------------------------------------------------------------
 * GPIO_InitUSART2Pins
 *   Configures PA2 (TX) and PA3 (RX) for USART2 alternate function.
 *   AF7 is the USART2 alternate function on STM32F407.
 * ---------------------------------------------------------------------------*/
void GPIO_InitUSART2Pins(void)
{
    GPIO_PinHandle_t usartPins = {0};

    /* Configure PA2 as USART2_TX */
    usartPins.config.port  = GPIO_PORT_A;
    usartPins.config.pin   = 2;
    usartPins.config.mode  = GPIO_MODE_AF;
    usartPins.config.otype = GPIO_OTYPE_PP;
    usartPins.config.pull  = GPIO_PULLUP;
    usartPins.config.speed = GPIO_SPEED_FAST;
    usartPins.config.af    = 7;  // AF7 = USART2
    GPIO_Init(&usartPins);

    /* Configure PA3 as USART2_RX */
    usartPins.config.pin   = 3;
    GPIO_Init(&usartPins);
}

/* -----------------------------------------------------------------------------
 * USART2_Init
 *   Initializes USART2 with the specified configuration:
 *   - 115200 baud, 8 bits, 1 stop bit, no parity
 * ---------------------------------------------------------------------------*/
void USART2_Init(void)
{
    usart2.dev = USART_DEVICE_USART2;

    usart2.cfg.mode         = USART_MODE_TXRX;
    usart2.cfg.baudRate     = 115200;
    usart2.cfg.wordLen      = USART_WORDLEN_8BITS;
    usart2.cfg.parity       = USART_PARITY_NONE;
    usart2.cfg.stopBits     = USART_STOPBITS_1;
    usart2.cfg.hwFlowCtrl   = USART_HW_FLOW_CTRL_NONE;
    usart2.cfg.oversampling = USART_OVER_16;

    USART_Init(&usart2);
}

/* -----------------------------------------------------------------------------
 * LED_Init
 *   Configures PD12 (green LED on DISC1 board) as output
 * ---------------------------------------------------------------------------*/
void LED_Init(void)
{
    led.config.port  = GPIO_PORT_D;
    led.config.pin   = 12;
    led.config.mode  = GPIO_MODE_OUTPUT;
    led.config.otype = GPIO_OTYPE_PP;
    led.config.pull  = GPIO_NOPULL;
    led.config.speed = GPIO_SPEED_LOW;
    led.config.af    = 0;
    GPIO_Init(&led);
}

/* -----------------------------------------------------------------------------
 * Button_Init
 *   Configures PA0 (USER button on DISC1 board) as input
 * ---------------------------------------------------------------------------*/
void Button_Init(void)
{
    button.config.port  = GPIO_PORT_A;
    button.config.pin   = 0;
    button.config.mode  = GPIO_MODE_INPUT;
    button.config.otype = GPIO_OTYPE_PP;
    button.config.pull  = GPIO_NOPULL;  // DISC1 has external pulldown
    button.config.speed = GPIO_SPEED_LOW;
    button.config.af    = 0;
    GPIO_Init(&button);
}

/* -----------------------------------------------------------------------------
 * Main
 * ---------------------------------------------------------------------------*/
int main(void)
{
    uint8_t prevButtonState = 0;
    uint8_t currButtonState = 0;

    /* Initialize peripherals */
    LED_Init();
    Button_Init();
    GPIO_InitUSART2Pins();
    USART2_Init();

    /* Enable USART2 peripheral */
    USART_PeripheralControl(&usart2, ENABLE);

    /* Turn off LED initially */
    GPIO_WriteToOutputPin(led.port, led.config.pin, GPIO_PIN_RESET);

    while (1)
    {
        /* Read button state */
        currButtonState = GPIO_ReadFromInputPin(button.port, button.config.pin);

        /* Detect button press (rising edge: 0 -> 1) */
        if (currButtonState && !prevButtonState)
        {
            /* Debounce delay */
            delay(500000);

            /* Transmit message */
            USART_SendData(&usart2, (const uint8_t*)msg, strlen(msg));

            /* Blink LED to indicate transmission */
            GPIO_WriteToOutputPin(led.port, led.config.pin, GPIO_PIN_SET);
            delay(1000000);
            GPIO_WriteToOutputPin(led.port, led.config.pin, GPIO_PIN_RESET);
        }

        prevButtonState = currButtonState;

        /* Small delay to reduce CPU load */
        delay(10000);
    }

    return 0;
}
