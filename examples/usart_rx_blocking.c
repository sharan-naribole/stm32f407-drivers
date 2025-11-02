/* =============================================================================
 * Project  : STM32F407G-DISC1 USART Driver Test - Receiver
 * File     : 010usart_rx_blocking.c
 * Purpose  : Demonstrate USART communication by receiving messages from another
 *            STM32 using blocking mode.
 *
 * Hardware Setup:
 *   - Board: STM32F407G-DISC1 (RECEIVER side)
 *   - USART2 pins:
 *       • PA2 → USART2_TX (connect to PA3 of TRANSMITTER board)
 *       • PA3 → USART2_RX (connect to PA2 of TRANSMITTER board)
 *       • GND → GND (connect grounds of both boards)
 *   - On-board LED (PD12) toggles each time a message is received
 *
 * Configuration:
 *   - Baud Rate  : 115200 bps
 *   - Word Length: 8 bits
 *   - Stop Bits  : 1
 *   - Parity     : None
 *   - Flow Control: None
 *
 * Wiring between two STM32F407 boards:
 *   Transmitter         Receiver
 *   PA2 (TX) ───────► PA3 (RX)
 *   PA3 (RX) ◄─────── PA2 (TX)
 *   GND      ◄──────► GND
 *
 * Scenario:
 *   1) Receiver waits for incoming data on USART2
 *   2) When data arrives, it reads the complete message
 *   3) LED toggles to indicate successful reception
 *   4) Repeats indefinitely
 *
 * Key Learnings:
 *   - How to configure GPIO pins for USART alternate function
 *   - How to initialize USART peripheral for reception
 *   - How to use blocking reception APIs
 *   - Peer-to-peer USART communication between two microcontrollers
 *
 * Notes:
 *   - This example receives exactly 31 bytes (length of transmitted message)
 *   - For production code, consider using interrupt-driven or DMA mode
 *   - For variable-length messages, implement a protocol with delimiters or length fields
 * =============================================================================
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_usart_driver.h"
#include <string.h>

/* Buffer to store received data */
#define RX_BUFFER_SIZE 64
uint8_t rxBuffer[RX_BUFFER_SIZE];

/* USART handle */
USART_Handle_t usart2;

/* LED handle for indication */
GPIO_PinHandle_t led;

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
 * Main
 * ---------------------------------------------------------------------------*/
int main(void)
{
    /* Initialize peripherals */
    LED_Init();
    GPIO_InitUSART2Pins();
    USART2_Init();

    /* Enable USART2 peripheral */
    USART_PeripheralControl(&usart2, ENABLE);

    /* Turn off LED initially */
    GPIO_WriteToOutputPin(led.port, led.config.pin, GPIO_PIN_RESET);

    while (1)
    {
        /* Clear receive buffer */
        memset(rxBuffer, 0, RX_BUFFER_SIZE);

        /* Wait for and receive message (31 bytes: "Hello from STM32 Transmitter!\n") */
        /* Note: This is a blocking call - will wait until all bytes are received */
        USART_ReceiveData(&usart2, rxBuffer, 31);

        /* Toggle LED to indicate message received */
        GPIO_ToggleOutputPin(led.port, led.config.pin);

        /* Optional: Send echo back to transmitter */
        // USART_SendData(&usart2, rxBuffer, 31);

        /* Small delay for visual feedback */
        delay(1000000);
    }

    return 0;
}
