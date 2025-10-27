#include "stm32f407xx_gpio_driver.h"

/* =============================================================================
 * GPIO_PeriClockControl
 *
 * Enables or disables the AHB1 peripheral clock for a given GPIO port.
 *
 * Parameters:
 *   pGPIOx : Pointer to GPIO port base (GPIOA..GPIOI)
 *   EnorDi : ENABLE (1) to turn clock ON, DISABLE (0) to turn it OFF
 *
 * Notes:
 * - The RCC->AHB1ENR register controls the clock gating for all AHB1 peripherals.
 * - Each GPIO port A..I is mapped to a bit [0..8] in RCC->AHB1ENR.
 * - Macros like GPIOA_CLK_EN()/DIS() expand to set/clear the right bit.
 * - A dummy read-back of RCC->AHB1ENR is performed after the write to:
 *      1) Ensure the write has reached the bus (flush write buffers).
 *      2) Guarantee the peripheral is clocked before its registers are accessed.
 * - Without the dummy read, back-to-back GPIO register writes may fail
 *   if the clock gate hasn’t yet opened in hardware.
 * =============================================================================
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (pGPIOx == GPIOA) {
        if (EnorDi == ENABLE) { GPIOA_CLK_EN(); (void)RCC->AHB1ENR; }
        else                  { GPIOA_CLK_DIS(); (void)RCC->AHB1ENR; }
    } else if (pGPIOx == GPIOB) {
        if (EnorDi == ENABLE) { GPIOB_CLK_EN(); (void)RCC->AHB1ENR; }
        else                  { GPIOB_CLK_DIS(); (void)RCC->AHB1ENR; }
    } else if (pGPIOx == GPIOC) {
        if (EnorDi == ENABLE) { GPIOC_CLK_EN(); (void)RCC->AHB1ENR; }
        else                  { GPIOC_CLK_DIS(); (void)RCC->AHB1ENR; }
    } else if (pGPIOx == GPIOD) {
        if (EnorDi == ENABLE) { GPIOD_CLK_EN(); (void)RCC->AHB1ENR; }
        else                  { GPIOD_CLK_DIS(); (void)RCC->AHB1ENR; }
    } else if (pGPIOx == GPIOE) {
        if (EnorDi == ENABLE) { GPIOE_CLK_EN(); (void)RCC->AHB1ENR; }
        else                  { GPIOE_CLK_DIS(); (void)RCC->AHB1ENR; }
    } else if (pGPIOx == GPIOF) {
        if (EnorDi == ENABLE) { GPIOF_CLK_EN(); (void)RCC->AHB1ENR; }
        else                  { GPIOF_CLK_DIS(); (void)RCC->AHB1ENR; }
    } else if (pGPIOx == GPIOG) {
        if (EnorDi == ENABLE) { GPIOG_CLK_EN(); (void)RCC->AHB1ENR; }
        else                  { GPIOG_CLK_DIS(); (void)RCC->AHB1ENR; }
    } else if (pGPIOx == GPIOH) {
        if (EnorDi == ENABLE) { GPIOH_CLK_EN(); (void)RCC->AHB1ENR; }
        else                  { GPIOH_CLK_DIS(); (void)RCC->AHB1ENR; }
    } else if (pGPIOx == GPIOI) {
        if (EnorDi == ENABLE) { GPIOI_CLK_EN(); (void)RCC->AHB1ENR; }
        else                  { GPIOI_CLK_DIS(); (void)RCC->AHB1ENR; }
    } else {
        // Invalid port pointer; optionally assert in debug builds.
        // assert(false);
    }
}

/* =============================================================================
 * GPIO_Init
 *
 * Initializes a single GPIO pin as described in h->config.
 * - Enables the AHB1 clock for the port
 * - Programs registers in a safe order:
 *   1) Pull-up/down (PUPDR)
 *   2) Output type (OTYPER)
 *   3) Output speed (OSPEEDR)
 *   4) Alternate function (AFR) if mode == AF
 *   5) Mode (MODER) — last, to avoid premature driving
 * =============================================================================
 */
void GPIO_Init(GPIO_PinHandle_t *h)
{
    if (!h) return;

    /* -------------------------------------------------------------------------
     * Step 0: Resolve GPIOx pointer from logical port id in config
     * -------------------------------------------------------------------------
     */
    switch (h->config.port) {
        case GPIO_PORT_A: h->port = GPIOA; break;
        case GPIO_PORT_B: h->port = GPIOB; break;
        case GPIO_PORT_C: h->port = GPIOC; break;
        case GPIO_PORT_D: h->port = GPIOD; break;
        case GPIO_PORT_E: h->port = GPIOE; break;
        case GPIO_PORT_F: h->port = GPIOF; break;
        case GPIO_PORT_G: h->port = GPIOG; break;
        case GPIO_PORT_H: h->port = GPIOH; break;
        case GPIO_PORT_I: h->port = GPIOI; break;
        default: return; // Invalid port id
    }

    /* -------------------------------------------------------------------------
     * Step 1: Enable the AHB1 clock for this port
     *   - Required before accessing any GPIO registers.
     *   - Uses macros from stm32f407xx.h (GPIOA_CLK_EN(), etc.)
     * -------------------------------------------------------------------------
     */
    GPIO_PeriClockControl(h->port, ENABLE);

    /* -------------------------------------------------------------------------
     * Step 2: Cache runtime mask for fast BSRR/ODR access
     * -------------------------------------------------------------------------
     */
    if (h->config.pin > 15U) return;   // Invalid pin number
    h->mask = (uint16_t)(1U << h->config.pin);

    GPIO_RegDef_t *port = h->port;
    const uint32_t pin_number = (uint32_t)h->config.pin;

    /* Compute common field info for 2-bit fields (MODER, OSPEEDR, PUPDR) */
    const uint32_t bit_pos_2 = pin_number * 2U;         // each pin takes 2 bits
    const uint32_t bit_mask_2 = (0x3UL << bit_pos_2);   // mask for those 2 bits

    /* -------------------------------------------------------------------------
     * Step 3: Configure Pull-up / Pull-down (PUPDR)
     *   - Determines default state of the input when not driven.
     *   - Encodings (per RM0090):
     *       00 = No pull, 01 = Pull-up, 10 = Pull-down
     * -------------------------------------------------------------------------
     */
    port->PUPDR = (port->PUPDR & ~bit_mask_2)
                | (((uint32_t)h->config.pull & 0x3UL) << bit_pos_2);

    /* -------------------------------------------------------------------------
     * Step 4: Configure Output type (OTYPER)
     *   - 0 = Push-pull, 1 = Open-drain
     *   - Only meaningful for Output/AF modes; harmless otherwise.
     * -------------------------------------------------------------------------
     */
    if (h->config.otype == GPIO_OTYPE_OD) {
        port->OTYPER |=  (1UL << pin_number);
    } else {
        port->OTYPER &= ~(1UL << pin_number);
    }

    /* -------------------------------------------------------------------------
     * Step 5: Configure Output speed (OSPEEDR)
     *   - 2 bits per pin
     *   - Encodings: 00 = Low, 01 = Medium, 10 = Fast, 11 = High
     *   - Only meaningful for Output/AF; harmless otherwise.
     * -------------------------------------------------------------------------
     */
    port->OSPEEDR = (port->OSPEEDR & ~bit_mask_2)
                  | (((uint32_t)h->config.speed & 0x3UL) << bit_pos_2);

    /* -------------------------------------------------------------------------
     * Step 6: Configure Alternate Function (AFR) if mode == AF
     *   - Each pin has 4-bit field in AFRL (pins 0-7) or AFRH (pins 8-15).
     *   - Program this *before* MODER, so the pin is correctly wired
     *     to its peripheral the moment MODER sets it to AF mode.
     * -------------------------------------------------------------------------
     */
    if (h->config.mode == GPIO_MODE_AF) {
        const uint32_t af_shift = (pin_number % 8U) * 4U;
        const uint32_t af_mask  = (0xFUL << af_shift);

        if (pin_number < 8U) {
            port->AFRL = (port->AFRL & ~af_mask)
                       | (((uint32_t)h->config.af & 0xFUL) << af_shift);
        } else {
            port->AFRH = (port->AFRH & ~af_mask)
                       | (((uint32_t)h->config.af & 0xFUL) << af_shift);
        }
    }

    /* -------------------------------------------------------------------------
     * Step 7: Configure Pin mode (MODER) — LAST
     *   - Encodings: 00 = Input, 01 = Output, 10 = AF, 11 = Analog
     *   - Writing this last ensures pin only starts driving after
     *     pull-ups, output type, speed, and AF are already configured.
     * -------------------------------------------------------------------------
     */
    port->MODER = (port->MODER & ~bit_mask_2)
                | (((uint32_t)h->config.mode & 0x3UL) << bit_pos_2);
}

/* =============================================================================
 * GPIO_DeInit
 *
 * Resets an entire GPIO port to its reset state by invoking the reset macro.
 * Each macro pulses RCC->AHB1RSTR bit for the given port.
 * =============================================================================
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if      (pGPIOx == GPIOA) { GPIOA_REG_RESET(); }
    else if (pGPIOx == GPIOB) { GPIOB_REG_RESET(); }
    else if (pGPIOx == GPIOC) { GPIOC_REG_RESET(); }
    else if (pGPIOx == GPIOD) { GPIOD_REG_RESET(); }
    else if (pGPIOx == GPIOE) { GPIOE_REG_RESET(); }
    else if (pGPIOx == GPIOF) { GPIOF_REG_RESET(); }
    else if (pGPIOx == GPIOG) { GPIOG_REG_RESET(); }
    else if (pGPIOx == GPIOH) { GPIOH_REG_RESET(); }
    else if (pGPIOx == GPIOI) { GPIOI_REG_RESET(); }
}

/* =============================================================================
 * GPIO_ReadFromInputPin
 *
 * Reads the logic level (0 or 1) from a single GPIO pin configured as input.
 *
 * Parameters:
 *   pGPIOx    : Pointer to GPIO port base (GPIOA..GPIOI)
 *   PinNumber : Pin index 0..15 within the port
 *
 * Return:
 *   GPIO_PIN_SET   (1) if the pin is HIGH
 *   GPIO_PIN_RESET (0) if the pin is LOW
 *
 * How it works:
 * - Each GPIO port has a 16-bit Input Data Register (IDR).
 * - Bit [n] of IDR reflects the logic level on pin n:
 *     0 = pin is low
 *     1 = pin is high
 * - We right-shift IDR by PinNumber and mask with 0x1 to isolate that bit.
 * - Cast result to uint8_t for a clean return.
 *
 * Notes:
 * - Pin must be configured as Input or Alternate Function (with input buffer enabled)
 *   for the IDR bit to be meaningful.
 * - If the port clock is disabled, IDR reads return undefined values.
 * =============================================================================
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1U);

    if (value)
        return GPIO_PIN_SET;   /* Logic HIGH */
    else
        return GPIO_PIN_RESET; /* Logic LOW  */
}

/* =============================================================================
 * GPIO_ReadFromInputPort
 *
 * Reads the entire 16-bit input data register (IDR) of a GPIO port.
 *
 * Parameters:
 *   pGPIOx : Pointer to GPIO port base (GPIOA..GPIOI)
 *
 * Return:
 *   Lower 16 bits of IDR, where:
 *     - Bit [n] corresponds to the logic level on pin n
 *     - 0 = pin is low, 1 = pin is high
 *
 * How it works:
 * - Each GPIO port has a 32-bit IDR register.
 * - Only the lower 16 bits are valid, one per physical pin (0..15).
 * - We simply mask the register to 0xFFFF and return it as uint16_t.
 *
 * Notes:
 * - Useful when you need to sample all pins at once (e.g., reading a parallel
 *   data bus).
 * - If you only care about a single pin, use GPIO_ReadFromInputPin() instead.
 * - If the port clock is disabled, IDR values are undefined.
 * =============================================================================
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    return (uint16_t)(pGPIOx->IDR & 0xFFFFU);
}

/* =============================================================================
 * GPIO_WriteToOutputPin
 *
 * Writes a logic level (SET or RESET) to a single output pin.
 *
 * Parameters:
 *   pGPIOx    : Pointer to GPIO port base (GPIOA..GPIOI)
 *   PinNumber : Pin index 0..15 within the port
 *   Value     : GPIO_PIN_SET (1) or GPIO_PIN_RESET (0)
 *
 * How it works:
 * - STM32 provides a special "Bit Set/Reset Register" (BSRR) for atomic updates.
 * - BSRR is 32 bits wide:
 *     - Lower 16 bits [0..15]   : Writing 1 sets the corresponding pin (ODR bit = 1)
 *     - Upper 16 bits [16..31] : Writing 1 resets the corresponding pin (ODR bit = 0)
 * - This avoids read-modify-write hazards on ODR and ensures glitch-free operation.
 *
 * Notes:
 * - Pin must be configured as OUTPUT or AF (driving mode) for effect.
 * - This operation is atomic — no risk of interrupt race conditions.
 * =============================================================================
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET) {
        /* Write 1 to lower half of BSRR → set pin */
        pGPIOx->BSRR = (1UL << PinNumber);
    } else {
        /* Write 1 to upper half of BSRR → reset pin */
        pGPIOx->BSRR = (1UL << (PinNumber + 16U));
    }
}

/* =============================================================================
 * GPIO_WriteToOutputPort
 *
 * Writes a 16-bit value directly to the Output Data Register (ODR).
 *
 * Parameters:
 *   pGPIOx : Pointer to GPIO port base (GPIOA..GPIOI)
 *   Value  : Lower 16 bits drive the pin states (0 = low, 1 = high)
 *
 * How it works:
 * - ODR is a 32-bit register, but only lower 16 bits map to GPIO pins.
 * - Writing to ODR directly sets the output state of all pins in the port.
 *
 * Notes:
 * - This is a **non-atomic** operation. If you want to update individual pins
 *   safely while other code/ISRs may also write to the port, use BSRR instead.
 * - Be careful: this overwrites ALL 16 pin states of the port.
 * - Useful when driving a parallel bus (e.g., 8-bit LCD data).
 * =============================================================================
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = (uint32_t)(Value & 0xFFFFU);
}

/* =============================================================================
 * GPIO_ToggleOutputPin
 *
 * Toggles a single output pin using the atomic BSRR interface.
 *
 * Parameters:
 *   pGPIOx    : Pointer to GPIO port base (GPIOA..GPIOI)
 *   PinNumber : Pin index 0..15 within the port
 *
 * How it works:
 * - Read the current output latch from ODR for the given pin.
 * - If the bit is 1 (pin HIGH), write to the UPPER half of BSRR to RESET it.
 * - If the bit is 0 (pin LOW), write to the LOWER half of BSRR to SET it.
 *
 * Why not ODR ^= mask?
 * - Direct ODR writes are non-atomic (read-modify-write) and can race with ISRs.
 * - BSRR updates a single bit atomically without touching other pins.
 *
 * Notes:
 * - Pin must be configured as OUTPUT or AF (driving). Toggling an input has no effect.
 * - There is an unavoidable read→write window between reading ODR and writing BSRR.
 *   If another context changes the same pin in that tiny window, the result may differ.
 *   If that matters, protect this with a brief critical section (disable/enable IRQs)
 *   around the toggle or centralize all pin updates in one context.
 * =============================================================================
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    const uint32_t bit = (1UL << PinNumber);

    /* Read current output latch (not the physical pin level) */
    if ((pGPIOx->ODR & bit) != 0U) {
        /* Currently HIGH → drive LOW by writing to upper 16 of BSRR */
        pGPIOx->BSRR = (bit << 16);   /* reset bit */
    } else {
        /* Currently LOW → drive HIGH by writing to lower 16 of BSRR */
        pGPIOx->BSRR = bit;           /* set bit */
    }
}

/* =============================================================================
 * Local helper: map a GPIO pin to an EXTI line and configure trigger & mask.
 *
 * Requirements:
 * - SYSCFG clock must be enabled (done inside this function).
 * - Your stm32f407xx.h defines SYSCFG/EXTI register blocks and SYSCFG_CLK_EN().
 *   (EXTI: IMR/RTSR/FTSR/PR; SYSCFG: EXTICR1..4).
 * =============================================================================
 */
void GPIO_ConfigEXTI(const GPIO_PinHandle_t *h, GPIO_ExtiTrigger_t trig, uint8_t enable)
{
    if (!h || !h->port || h->config.pin > 15U) return;

    /* --- 0) Enable SYSCFG APB2 clock so we can write EXTICR registers ------- */
    SYSCFG_CLK_EN();            /* APB2ENR bit 14 per your header */
    (void)RCC->APB2ENR;         /* read-back to ensure clock is live before writes */

    /* --- 1) Select the source GPIO port for this EXTI line in SYSCFG->EXTICR --
     * EXTICR is split into 4 registers, 4 lines each (4-bit field per line):
     *   EXTICR1: EXTI0..3   (bits [3:0], [7:4], [11:8], [15:12])
     *   EXTICR2: EXTI4..7
     *   EXTICR3: EXTI8..11
     *   EXTICR4: EXTI12..15
     *
     * Port coding on STM32F4 matches A=0, B=1, C=2, ... (your GPIO_PortId_t).
     */
    const uint32_t line      = (uint32_t)h->config.pin;  /* EXTI line number == pin */
    const uint32_t field_pos = (line % 4U) * 4U;         /* 4-bit field shift */
    const uint32_t field_msk = (0xFUL << field_pos);
    const uint32_t port_code = (uint32_t)h->config.port & 0xFU;

    if (line <= 3U) {
        SYSCFG->EXTICR1 = (SYSCFG->EXTICR1 & ~field_msk) | (port_code << field_pos);
    } else if (line <= 7U) {
        SYSCFG->EXTICR2 = (SYSCFG->EXTICR2 & ~field_msk) | (port_code << field_pos);
    } else if (line <= 11U) {
        SYSCFG->EXTICR3 = (SYSCFG->EXTICR3 & ~field_msk) | (port_code << field_pos);
    } else { /* 12..15 */
        SYSCFG->EXTICR4 = (SYSCFG->EXTICR4 & ~field_msk) | (port_code << field_pos);
    }

    /* --- 2) Program edge trigger selection in EXTI --------------------------- */
    /* Rising edge */
    if (trig == GPIO_EXTI_TRIGGER_RISING || trig == GPIO_EXTI_TRIGGER_BOTH) {
        EXTI->RTSR |=  (1UL << line);
    } else {
        EXTI->RTSR &= ~(1UL << line);
    }

    /* Falling edge */
    if (trig == GPIO_EXTI_TRIGGER_FALLING || trig == GPIO_EXTI_TRIGGER_BOTH) {
        EXTI->FTSR |=  (1UL << line);
    } else {
        EXTI->FTSR &= ~(1UL << line);
    }

    /* --- 3) Interrupt mask (IMR) -------------------------------------------- */
    if (enable == ENABLE) {
        EXTI->IMR |=  (1UL << line);   /* unmask to allow interrupt */
    } else {
        EXTI->IMR &= ~(1UL << line);   /* mask to disable interrupt */
    }

    /* --- 4) Clear any stale pending bit to avoid an immediate spurious IRQ --- */
    EXTI->PR = (1UL << line);          /* write 1 to clear pending */
}

/* =============================================================================
 * GPIO_IRQInterruptConfig
 *
 * Enable/Disable an NVIC IRQ line.
 *
 * Parameters:
 *   IRQNumber : Target IRQ line number (e.g., EXTI0_IRQn, EXTI9_5_IRQn, etc.)
 *   EnorDi    : ENABLE (1) to enable, DISABLE (0) to disable
 *
 * Notes:
 * - NVIC provides 32 IRQ enable bits per register. On STM32F407:
 *     * ISER0/ICER0 control IRQ  0..31
 *     * ISER1/ICER1 control IRQ 32..63
 *     * ISER2/ICER2 control IRQ 64..95
 * - Writing a '1' to an ISER bit enables that IRQ; writing a '1' to an ICER bit
 *   disables it. Other bits are unaffected.
 * - This only touches the NVIC. You still need to map GPIO→EXTI and set triggers
 *   via SYSCFG->EXTICR and EXTI->RTSR/FTSR (see GPIO_ConfigEXTI()).
 * =============================================================================
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (IRQNumber < 32U) {
            NVIC_ISER0 = (1UL << IRQNumber);
        } else if (IRQNumber < 64U) {
            NVIC_ISER1 = (1UL << (IRQNumber - 32U));
        } else if (IRQNumber < 96U) {
            NVIC_ISER2 = (1UL << (IRQNumber - 64U));
        } else {
            /* Out of supported range for STM32F407 */
        }
    }
    else /* DISABLE */
    {
        if (IRQNumber < 32U) {
            NVIC_ICER0 = (1UL << IRQNumber);
        } else if (IRQNumber < 64U) {
            NVIC_ICER1 = (1UL << (IRQNumber - 32U));
        } else if (IRQNumber < 96U) {
            NVIC_ICER2 = (1UL << (IRQNumber - 64U));
        } else {
            /* Out of supported range for STM32F407 */
        }
    }
}

/* =============================================================================
 * GPIO_IRQPriorityConfig
 *
 * Set NVIC priority for an IRQ line.
 *
 * Parameters:
 *   IRQNumber   : Target IRQ line number (e.g., EXTI0_IRQn, EXTI9_5_IRQn, etc.)
 *   IRQPriority: Encoded priority value.
 *
 * Notes:
 * - On STM32F4, each IRQ has one byte in NVIC_IPR (0xE000E400 + IRQn).
 * - Only the upper NO_PR_BITS_IMPLEMENTED bits of that byte are implemented
 *   in hardware (on F4 this is 4 bits → values 0..15 after shifting).
 * - We left-shift the user value so it lands in the implemented top bits.
 * - Smaller numerical priority values mean *higher* preemption priority.
 * - This function only sets the priority; enabling/disabling is done via
 *   GPIO_IRQInterruptConfig().
 * =============================================================================
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    if (IRQNumber >= 96U) {
        /* STM32F407 uses IRQ numbers 0..95 for external interrupts */
        return;
    }

    /* Shift so that the value occupies the implemented high bits of the byte */
    const uint8_t shift = (uint8_t)(8U - NO_PR_BITS_IMPLEMENTED);

    /* Byte-addressable priority register: one byte per IRQ */
    /* Write the whole byte; only the top bits take effect on F4 */
    NVIC_IPR_BASE[IRQNumber] = (uint8_t)(IRQPriority << shift);
}

/* =============================================================================
 * GPIO_IRQHandling
 *
 * Clear EXTI pending flag for a given line (PinNumber 0..15).
 *
 * Parameters:
 *   PinNumber : EXTI line to service (matches GPIO pin index 0..15).
 *
 * Notes:
 * - EXTI->PR is "write-1-to-clear": writing a 1 to bit N clears the pending flag
 *   for EXTI line N. Writing 0 has no effect.
 * - Always check PR before clearing to ensure this IRQ actually caused the
 *   interrupt (useful when multiple lines share one IRQ handler, e.g., EXTI9_5).
 * - This function only clears the EXTI pending bit. Your ISR should call any
 *   application logic (e.g., toggle an LED) before or after this clear.
 * =============================================================================
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    if (PinNumber <= 15U)
    {
        uint32_t line_bit = (1UL << PinNumber);

        /* Service only if this line is actually pending */
        if (EXTI->PR & line_bit)
        {
            /* Write 1 to clear the pending bit (W1C semantics) */
            EXTI->PR = line_bit;
        }
    }
}

