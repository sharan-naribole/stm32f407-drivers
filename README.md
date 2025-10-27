# STM32F407 Bare-Metal Drivers

A comprehensive collection of bare-metal peripheral drivers and examples for the STM32F407G-DISC1 development board. This project demonstrates low-level hardware programming, direct register manipulation, and efficient peripheral control without relying on HAL libraries.

---

## ✨ Features

- **Pure bare-metal programming** - No HAL dependencies, direct register access
- **Complete GPIO driver** with EXTI interrupt support
- **Full-featured SPI driver** supporting Master/Slave, 8/16-bit modes
- **Well-documented examples** with detailed inline comments and flowcharts
- **Professional code structure** - Modular, reusable, production-ready patterns
- **Educational focus** - Ideal for learning ARM Cortex-M4 architecture

---

## 📖 Overview

This repository contains custom-built device drivers for the STM32F407VGT6 microcontroller, along with practical examples demonstrating their usage. All drivers are written from scratch using only CMSIS headers and direct register access, providing deep insight into ARM Cortex-M4 peripheral architecture.

**Target Hardware:** STM32F407G-DISC1 Discovery Board
**Microcontroller:** STM32F407VGT6 (ARM Cortex-M4, 168MHz, 1MB Flash, 192KB RAM)
**Development Approach:** Bare-metal programming with direct register manipulation

---

## 📁 Project Structure

```
stm32f407-drivers/
├── drivers/           # Peripheral driver implementations
│   ├── stm32f407xx.h                    # MCU-specific register definitions
│   ├── stm32f407xx_gpio_driver.h/c      # GPIO driver
│   └── stm32f407xx_spi_driver.h/c       # SPI driver
│
└── examples/          # Example applications demonstrating driver usage
    ├── gpio_led_blink.c                 # Basic GPIO output (LED toggle)
    ├── gpio_button_polling.c            # GPIO input polling
    ├── gpio_button_interrupt.c          # GPIO interrupt (EXTI)
    ├── spi_master_tx_blocking.c         # SPI master transmission
    └── spi_master_rx_blocking.c         # SPI master reception
```

---

## 🔧 Implemented Drivers

### GPIO Driver (`stm32f407xx_gpio_driver`)

A complete General Purpose Input/Output (GPIO) driver supporting all GPIO functionality on STM32F407.

**Features:**
- Pin mode configuration (Input, Output, Alternate Function, Analog)
- Output type selection (Push-Pull, Open-Drain)
- Pull-up/Pull-down resistor configuration
- Speed control (Low, Medium, Fast, High)
- Alternate function mapping (AF0-AF15)
- External interrupt configuration (EXTI)
- Interrupt priority and enable/disable control
- Atomic pin read/write/toggle operations

**Key APIs:**
- `GPIO_Init()` - Initialize GPIO pin with specified configuration
- `GPIO_ReadFromInputPin()` - Read digital input state
- `GPIO_WriteToOutputPin()` - Write digital output state
- `GPIO_ToggleOutputPin()` - Toggle output pin state
- `GPIO_ConfigEXTI()` - Configure external interrupt on GPIO pin
- `GPIO_IRQInterruptConfig()` - Enable/disable GPIO interrupt in NVIC
- `GPIO_IRQPriorityConfig()` - Set interrupt priority
- `GPIO_IRQHandling()` - Clear interrupt pending flags

**Supported Features:**
- All GPIO ports (GPIOA-GPIOI)
- All pins (0-15)
- Rising/Falling/Both edge triggers for interrupts
- Complete EXTI line management

---

### SPI Driver (`stm32f407xx_spi_driver`)

A full-featured Serial Peripheral Interface (SPI) driver supporting all SPI peripherals (SPI1-SPI3).

**Features:**
- Master and Slave mode operation
- Full-duplex, Half-duplex, and Simplex communication
- 8-bit and 16-bit data frame formats
- Hardware and Software NSS (Slave Select) management
- Configurable clock polarity (CPOL) and phase (CPHA)
- Programmable baud rate (fPCLK/2 to fPCLK/256)
- MSB-first or LSB-first bit order
- Motorola and TI frame formats
- Optional CRC calculation
- Blocking (polling) mode for data transfer
- Status and error handling

**Key APIs:**
- `SPI_Init()` - Initialize SPI peripheral with specified configuration
- `SPI_PeripheralControl()` - Enable/disable SPI peripheral
- `SPI_SendData()` - Transmit data in blocking mode
- `SPI_ReceiveData()` - Receive data in blocking mode
- `SPI_GetFlagStatus()` - Check SPI status flags

**Supported Configurations:**
- Clock speeds: Up to 21 MHz (SPI2/3 on APB1) or 42 MHz (SPI1 on APB2)
- Both CPOL=0/1 and CPHA=0/1 modes
- Software and hardware chip select management
- 8-bit and 16-bit data frames

---

## 💡 Examples

### GPIO Examples

#### 1. `gpio_led_blink.c` - Basic LED Blinking

**Goal:** Demonstrate basic GPIO output configuration and control.

**Description:**
Toggles the on-board green LED (PD12) using a software delay loop. Demonstrates both Push-Pull and Open-Drain output modes.

**Concepts Covered:**
- GPIO output configuration
- Push-Pull vs Open-Drain output types
- Pin toggle operation
- Software delay implementation

**Hardware:**
- LED: PD12 (Green LED on Discovery board)

---

#### 2. `gpio_button_polling.c` - Button-Controlled LED

**Goal:** Demonstrate GPIO input reading with polling technique.

**Description:**
Reads the on-board user button (PA0) in a continuous loop and toggles the LED on each button press using edge detection logic.

**Concepts Covered:**
- GPIO input configuration
- Polling-based input reading
- Software edge detection (rising edge)
- Software debouncing

**Hardware:**
- Button: PA0 (Blue user button)
- LED: PD12 (Green LED)

---

#### 3. `gpio_button_interrupt.c` - Interrupt-Driven Button

**Goal:** Demonstrate external interrupt (EXTI) configuration and handling.

**Description:**
Uses EXTI0 interrupt to detect button presses on PA0. The LED toggles in the interrupt service routine, demonstrating event-driven programming.

**Concepts Covered:**
- EXTI (External Interrupt) configuration
- Interrupt routing through SYSCFG
- NVIC configuration (priority and enable)
- Interrupt service routine implementation
- Wait-for-interrupt (WFI) low-power mode

**Hardware:**
- Button: PA0 (triggers EXTI0 on falling edge)
- LED: PD12 (toggled in ISR)

---

### SPI Examples

#### 4. `spi_master_tx_blocking.c` - SPI Master Transmission

**Goal:** Test SPI transmission in master mode with both 8-bit and 16-bit data frames.

**Description:**
Configures SPI2 as master and transmits a test message at maximum speed (21 MHz). Tests both DFF=0 (8-bit) and DFF=1 (16-bit) modes with comprehensive LED feedback showing test progress.

**Concepts Covered:**
- SPI master mode configuration
- GPIO alternate function mapping for SPI pins
- Manual chip select (NSS) control
- 8-bit vs 16-bit data frame formats
- Maximum baud rate configuration (APB1_CLK/2)
- Blocking transmission with `SPI_SendData()`
- Multi-LED status indication

**Hardware:**
- SPI2 pins: PB12 (NSS), PB13 (SCK), PB14 (MISO), PB15 (MOSI)
- Status LEDs: PD12 (Green), PD13 (Orange), PD14 (Red), PD15 (Blue)
- Test message: "SPI Master blocking transmission test..." (104 bytes)

**Test Sequence:**
1. Orange LED: System initialization
2. Green LED: 8-bit mode test (104 bytes)
3. Red LED: 16-bit mode test (52 words)
4. Blue LED: Overall success indication

---

#### 5. `spi_master_rx_blocking.c` - SPI Master Reception

**Goal:** Test SPI reception from a slave device using software NSS management.

**Description:**
Configures SPI1 as master and receives a predefined message from a slave STM32 board. Uses PB6 for manual chip select control and validates the received message.

**Concepts Covered:**
- SPI master receive operation
- Software NSS management (manual CS control)
- Master-slave communication protocol
- Inter-board SPI communication
- Received data validation
- Button-triggered operation with EXTI
- Timing considerations for slave synchronization

**Hardware:**
- SPI1 pins: PA5 (SCK), PA6 (MISO), PA7 (MOSI)
- Chip Select: PB6 (GPIO output)
- Button: PA0 (triggers reception via EXTI0)
- Status LEDs: PD12 (Green), PD13 (Orange), PD14 (Red), PD15 (Blue)
- Expected message: "Hello from Slave!" (18 bytes)

**Communication Flow:**
1. Orange LED: Ready state (waiting for button)
2. Button press: Triggers test cycle
3. Blue LED: CS assertion
4. Green LED: Receiving data
5. Validation: Success (Green blinks) or Error (Red patterns)

**Wiring (Master-to-Slave):**
```
Master          Slave
PA5 (SCK)   →   PA5 (SCK)
PA6 (MISO)  ←   PA6 (MISO)
PA7 (MOSI)  →   PA7 (MOSI)
PB6 (CS)    →   PB6 (CS)
GND         ─   GND
```

---

## 🚀 Getting Started

### Prerequisites

- STM32F407G-DISC1 development board
- STM32CubeIDE or compatible ARM GCC toolchain
- ST-Link debugger (integrated on Discovery board)
- Logic analyzer or oscilloscope (optional, for SPI verification)

### Building the Examples

1. Clone this repository
2. Open STM32CubeIDE
3. Import the project
4. Select an example file as the build target (rename to `main.c` or adjust build configuration)
5. Build and flash to the target board

### Running Examples

Each example is self-contained and can be run independently:

1. **GPIO LED Blink:** Flash and observe LED blinking
2. **GPIO Button Polling:** Press user button to toggle LED
3. **GPIO Button Interrupt:** Press user button (interrupt-driven toggle)
4. **SPI Master TX:** Connect logic analyzer to SPI2 pins to observe transmission
5. **SPI Master RX:** Connect two STM32 boards and run slave code on second board

---

## 🗺️ Development Roadmap

### Completed Drivers
- [x] GPIO Driver (with EXTI support)
- [x] SPI Driver (Master/Slave, blocking mode)

### In Progress / Planned Drivers
- [ ] **I2C Driver** - Inter-Integrated Circuit communication
  - Master and Slave modes
  - 7-bit and 10-bit addressing
  - Standard and Fast modes
  - Clock stretching support

- [ ] **UART Driver** - Universal Asynchronous Receiver/Transmitter
  - Configurable baud rates
  - 8/9-bit data frames
  - Hardware flow control (RTS/CTS)
  - DMA support

- [ ] **USART Driver** - Universal Synchronous/Asynchronous Receiver/Transmitter
  - Synchronous mode with clock
  - LIN mode support
  - IrDA encoder/decoder
  - Multi-processor communication

### Future Enhancements
- [ ] Interrupt-driven SPI (non-blocking mode)
- [ ] DMA support for SPI transfers
- [ ] Timer/PWM driver
- [ ] ADC driver
- [ ] RTC driver
- [ ] Additional examples and tutorials

---

## 🎯 Technical Highlights

- **No HAL Dependencies:** Pure register-level programming for maximum control and learning
- **CMSIS Compliant:** Uses CMSIS headers for portability
- **Well Documented:** Extensive inline comments explaining register operations
- **Modular Design:** Clean separation between drivers and application code
- **Comprehensive Examples:** Each example includes detailed flowcharts and LED status indicators
- **Production-Ready Patterns:** Proper error handling, status checking, and timeout management

---

## 📚 Learning Resources

This project is ideal for:
- Understanding ARM Cortex-M4 peripheral architecture
- Learning bare-metal embedded programming
- Preparing for professional embedded systems development
- Studying for embedded systems interviews
- Building a foundation before using HAL/LL libraries

---

## 🧪 Troubleshooting

### GPIO Issues
- **LED not blinking:** Check GPIOD clock enable in RCC_AHB1ENR register and verify pin configuration (MODER, OTYPER)
- **Button not responding:** Verify PA0 is configured as input, check for external pull-down resistor on Discovery board
- **Interrupt not firing:** Ensure EXTI line is configured in SYSCFG_EXTICR, interrupt is enabled in NVIC, and ISR name matches vector table

### SPI Issues
- **No SPI output:** Check SPI clock enable (RCC_APB1/2ENR), verify SPE bit is set in SPI_CR1, ensure GPIO pins configured for alternate function
- **Wrong data received:** Verify CPOL/CPHA match between master and slave, check baud rate divisor, ensure NSS timing is correct
- **SPI communication errors:** Check BSY flag before disabling SPI, verify TXE/RXNE flags, ensure sufficient delay for slave response
- **Master-Slave connection issues:** Verify all 5 connections (SCK, MISO, MOSI, CS, GND), check common ground, keep wires short for high-speed communication

### General Issues
- **Code won't flash:** Verify ST-Link connection, check target power, try erasing chip first
- **Hardfault on startup:** Check stack pointer initialization, verify vector table location, review startup code
- **Undefined reference errors:** Ensure all driver .c files are included in build, check linker script

---

## ❓ FAQ

**Q: Why bare-metal instead of HAL?**
A: Bare-metal programming provides complete control and deep understanding of hardware. This project is educational-focused, helping you master register-level programming before using abstraction layers.

**Q: Can I use these drivers in production?**
A: These drivers demonstrate production-ready patterns (error handling, proper initialization, etc.) but may need additional features like timeout handling, advanced error recovery, and power management for commercial use.

**Q: What's the difference between GPIO modes?**
A:
- **Input:** Read external signals (buttons, sensors)
- **Output:** Drive LEDs, control logic levels
- **Alternate Function:** Hand control to peripherals (SPI, I2C, UART)
- **Analog:** For ADC/DAC use, disables digital circuitry

**Q: How do I choose SPI clock speed?**
A: Start with a conservative speed (2 MHz) for testing. Increase gradually while monitoring signal quality with oscilloscope/logic analyzer. Consider wire length, slave device limits, and EMI requirements.

**Q: What are the next drivers to implement?**
A: The roadmap prioritizes I2C (for sensors), UART (for serial communication), and USART (for advanced protocols). Check the Development Roadmap section for details.

---

## 📚 References

### Official Documentation
- [STM32F407 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf) - Complete peripheral register descriptions
- [STM32F407G-DISC1 User Manual](https://www.st.com/resource/en/user_manual/dm00039084-discovery-kit-with-stm32f407vg-mcu-stmicroelectronics.pdf) - Board-specific information and schematics
- [ARM Cortex-M4 Generic User Guide](https://developer.arm.com/documentation/dui0553/latest/) - Core architecture and programming model

### Learning Resources
- [FastBit Embedded Brain Academy](https://www.udemy.com/user/kiran-nayak-2/) - Embedded Systems Programming on ARM Cortex-M3/M4 Processor
- [Making Embedded Systems](https://books.google.com/books/about/Making_Embedded_Systems.html?id=fLP4EAAAQBAJ) by Elecia White - Design patterns for embedded software
- [The Definitive Guide to ARM Cortex-M3/M4](https://www.amazon.com/Definitive-Guide-ARM-Cortex-M3-M4-Processors/dp/0124080820) by Joseph Yiu

### Community & Tools
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) - Free IDE from STMicroelectronics
- [ARM Keil MDK](https://www.keil.com/demo/eval/arm.htm) - Alternative development environment

---

## 📜 License

This project is open source and available for educational purposes.

---

## 👤 Author

Developed and tested on STM32F407G-DISC1 hardware.

For questions or contributions, please open an issue or pull request.

---

## 🙏 Acknowledgments

- STMicroelectronics for comprehensive STM32F407 documentation
- ARM for Cortex-M4 technical resources and architecture guides
- Embedded systems community for inspiration, best practices, and continuous learning
