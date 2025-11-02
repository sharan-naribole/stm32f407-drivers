#include "stm32f407xx_usart_driver.h"
#include <stddef.h>   // for NULL

/* =============================================================================
 * USART_PeriClockControl
 *
 * Enables or disables the APB peripheral clock for a given USART instance.
 *
 * Parameters:
 *   pUSARTx : Pointer to USART base (USART1..6, UART4..5)
 *   EnorDi  : ENABLE (1) to turn clock ON, DISABLE (0) to turn it OFF
 *
 * Notes:
 * - USART1/6 are on APB2; USART2/3, UART4/5 are on APB1.
 * - Macros USARTx_CLK_EN/DIS toggle the appropriate ENR bits in RCC.
 * - A dummy read-back of the corresponding ENR is performed after the write to:
 *      1) ensure the write has reached the bus (flush write buffers), and
 *      2) guarantee the peripheral is clocked before its registers are accessed.
 * - This mirrors the pattern used in GPIO_PeriClockControl() for consistency.
 * =============================================================================
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
    if (pUSARTx == USART1)
    {
        if (EnorDi == ENABLE) { USART1_CLK_EN(); (void)RCC->APB2ENR; }
        else                  { USART1_CLK_DIS(); (void)RCC->APB2ENR; }
    }
    else if (pUSARTx == USART2)
    {
        if (EnorDi == ENABLE) { USART2_CLK_EN(); (void)RCC->APB1ENR; }
        else                  { USART2_CLK_DIS(); (void)RCC->APB1ENR; }
    }
    else if (pUSARTx == USART3)
    {
        if (EnorDi == ENABLE) { USART3_CLK_EN(); (void)RCC->APB1ENR; }
        else                  { USART3_CLK_DIS(); (void)RCC->APB1ENR; }
    }
    else if (pUSARTx == UART4)
    {
        if (EnorDi == ENABLE) { UART4_CLK_EN(); (void)RCC->APB1ENR; }
        else                  { UART4_CLK_DIS(); (void)RCC->APB1ENR; }
    }
    else if (pUSARTx == UART5)
    {
        if (EnorDi == ENABLE) { UART5_CLK_EN(); (void)RCC->APB1ENR; }
        else                  { UART5_CLK_DIS(); (void)RCC->APB1ENR; }
    }
    else if (pUSARTx == USART6)
    {
        if (EnorDi == ENABLE) { USART6_CLK_EN(); (void)RCC->APB2ENR; }
        else                  { USART6_CLK_DIS(); (void)RCC->APB2ENR; }
    }
    else
    {
        /* Invalid USART pointer; optionally assert in debug builds. */
    }
}

/* =============================================================================
 * Local helper: usart_resolve_regs
 *
 * Maps a logical device ID (USART_DEVICE_USART1..6, UART4..5) to the corresponding
 * peripheral base pointer.
 *
 * Parameters:
 *   dev : Logical device ID (enum USART_Device_t)
 *
 * Return:
 *   Pointer to USART register block (USART_RegDef_t*), or NULL if invalid.
 *
 * Notes:
 * - Keeps USART_Init() cleaner by centralizing the device→pointer mapping.
 * - Application code never calls this directly; it's purely internal.
 * =============================================================================
 */
static inline USART_RegDef_t* usart_resolve_regs(USART_Device_t dev)
{
    switch (dev) {
    case USART_DEVICE_USART1: return USART1;
    case USART_DEVICE_USART2: return USART2;
    case USART_DEVICE_USART3: return USART3;
    case USART_DEVICE_UART4:  return UART4;
    case USART_DEVICE_UART5:  return UART5;
    case USART_DEVICE_USART6: return USART6;
    default:                  return (USART_RegDef_t*)0;
    }
}

/* =============================================================================
 * Local helper: usart_get_pclk
 *
 * Returns the peripheral clock frequency (PCLK1 or PCLK2) for a given USART instance.
 * This is needed for baud rate calculation.
 *
 * Parameters:
 *   pUSARTx : Pointer to USART base (USART1..6, UART4..5)
 *
 * Return:
 *   Peripheral clock frequency in Hz.
 *
 * Notes:
 * - USART1/6 are on APB2; USART2/3, UART4/5 are on APB1.
 * - This is a simplified implementation that assumes default clock configuration.
 * - For production code, you should query the actual RCC configuration.
 * - Default STM32F407 clocks: PCLK1 = 42 MHz, PCLK2 = 84 MHz (assuming 168 MHz SYSCLK).
 * =============================================================================
 */
static uint32_t usart_get_pclk(USART_RegDef_t *pUSARTx)
{
    /* Simplified: return typical values for STM32F407 at default clocking.
     * For a production driver, parse RCC->CFGR for AHB/APB1/APB2 prescalers. */

    if ((pUSARTx == USART1) || (pUSARTx == USART6))
    {
        /* APB2 = 84 MHz (typical) */
        return 84000000UL;
    }
    else
    {
        /* APB1 = 42 MHz (typical) */
        return 42000000UL;
    }
}

/* =============================================================================
 * Local helper: usart_set_baud_rate
 *
 * Calculates and sets the baud rate divisor in BRR register.
 *
 * Parameters:
 *   h        : Handle with configured USART instance
 *   baudRate : Desired baud rate (e.g., 9600, 115200)
 *
 * Notes:
 * - Formula (RM0090 §30.3.4):
 *     For OVER8=0 (oversampling by 16):
 *       USARTDIV = PCLK / (16 * baudRate)
 *       BRR[15:4] = Mantissa, BRR[3:0] = Fraction (4 bits)
 *     For OVER8=1 (oversampling by 8):
 *       USARTDIV = PCLK / (8 * baudRate)
 *       BRR[15:4] = Mantissa, BRR[2:0] = Fraction (3 bits, BRR[3]=0)
 * - Example: PCLK=42MHz, baud=115200, OVER8=0:
 *       USARTDIV = 42000000 / (16 * 115200) = 22.786
 *       Mantissa = 22, Fraction = 0.786 * 16 = 12.576 ≈ 13
 *       BRR = (22 << 4) | 13 = 0x16D
 * =============================================================================
 */
static void usart_set_baud_rate(USART_Handle_t *h, uint32_t baudRate)
{
    uint32_t pclk = usart_get_pclk(h->regs);
    uint32_t usartdiv;
    uint32_t mantissa, fraction;

    if (h->cfg.oversampling == USART_OVER_8)
    {
        /* Oversampling by 8 */
        usartdiv = (pclk * 25U) / (2U * baudRate);  /* multiply by 100, then shift */
        mantissa = usartdiv / 100U;
        fraction = usartdiv - (mantissa * 100U);
        fraction = ((fraction * 8U) + 50U) / 100U;  /* round to nearest, 3-bit fraction */
        fraction &= 0x07U;  /* keep lower 3 bits */
        h->regs->BRR = (mantissa << 4) | fraction;
    }
    else
    {
        /* Oversampling by 16 (default) */
        usartdiv = (pclk * 25U) / (4U * baudRate);  /* multiply by 100, then shift */
        mantissa = usartdiv / 100U;
        fraction = usartdiv - (mantissa * 100U);
        fraction = ((fraction * 16U) + 50U) / 100U; /* round to nearest, 4-bit fraction */
        fraction &= 0x0FU;  /* keep lower 4 bits */
        h->regs->BRR = (mantissa << 4) | fraction;
    }
}

/* =============================================================================
 * USART_Init
 *
 * Initializes the USART peripheral as described in h->cfg, targeting the
 * logical device selected in h->dev. This mirrors the GPIO/SPI_Init pattern:
 * - Application pre-fills h->dev and h->cfg
 * - USART_Init() resolves h->regs and programs registers
 *
 * Implementation steps:
 *   0) Resolve USARTx base pointer from h->dev and cache in h->regs
 *   1) Enable APB clock for this USART instance (via USART_PeriClockControl)
 *   2) Build CR1/CR2/CR3 shadow values based on h->cfg:
 *        - Word length (8/9 bits)
 *        - Parity control (none/even/odd)
 *        - Mode (TX only, RX only, or TX+RX)
 *        - Oversampling mode (8 or 16)
 *        - Stop bits (1, 0.5, 2, 1.5)
 *        - Hardware flow control (none/RTS/CTS/RTS+CTS)
 *   3) Calculate and set baud rate (BRR register)
 *   4) Write CR1, CR2, CR3 to hardware with UE=0 (per ST recommended order)
 *   5) Clear any sticky flags by dummy read of SR then DR
 *   6) Leave UE=0 for glitch-free bring-up (app enables later via
 *      USART_PeripheralControl)
 *   7) Reset runtime state in the handle (flags, buffers, counters)
 *
 * Parameters:
 *   h : Handle with h->dev and h->cfg pre-populated
 *
 * Return:
 *   USART_OK on success, USART_ERR_INVAL if h or h->dev is invalid
 * =============================================================================
 */
USART_Status_t USART_Init(USART_Handle_t *h)
{
    uint32_t cr1 = 0, cr2 = 0, cr3 = 0;

    if (h == NULL)
        return USART_ERR_INVAL;

    /* Step 0: Resolve peripheral base pointer */
    h->regs = usart_resolve_regs(h->dev);
    if (h->regs == NULL)
        return USART_ERR_INVAL;

    /* Step 1: Enable peripheral clock */
    USART_PeriClockControl(h->regs, ENABLE);

    /* Step 2: Build CR1 shadow */

    /* Word length */
    if (h->cfg.wordLen == USART_WORDLEN_9BITS)
        cr1 |= USART_CR1_M;

    /* Parity control */
    if (h->cfg.parity != USART_PARITY_NONE)
    {
        cr1 |= USART_CR1_PCE;
        if (h->cfg.parity == USART_PARITY_ODD)
            cr1 |= USART_CR1_PS;
    }

    /* Mode: TX and/or RX */
    if ((h->cfg.mode == USART_MODE_TX) || (h->cfg.mode == USART_MODE_TXRX))
        cr1 |= USART_CR1_TE;
    if ((h->cfg.mode == USART_MODE_RX) || (h->cfg.mode == USART_MODE_TXRX))
        cr1 |= USART_CR1_RE;

    /* Oversampling mode */
    if (h->cfg.oversampling == USART_OVER_8)
        cr1 |= USART_CR1_OVER8;

    /* Build CR2 shadow: STOP bits */
    switch (h->cfg.stopBits)
    {
    case USART_STOPBITS_1:
        /* 00 = 1 stop bit (default) */
        break;
    case USART_STOPBITS_0_5:
        cr2 |= (1U << USART_CR2_STOP_Pos);  /* 01 = 0.5 stop bits */
        break;
    case USART_STOPBITS_2:
        cr2 |= (2U << USART_CR2_STOP_Pos);  /* 10 = 2 stop bits */
        break;
    case USART_STOPBITS_1_5:
        cr2 |= (3U << USART_CR2_STOP_Pos);  /* 11 = 1.5 stop bits */
        break;
    }

    /* Build CR3 shadow: Hardware flow control */
    switch (h->cfg.hwFlowCtrl)
    {
    case USART_HW_FLOW_CTRL_NONE:
        /* default: no flow control */
        break;
    case USART_HW_FLOW_CTRL_RTS:
        cr3 |= USART_CR3_RTSE;
        break;
    case USART_HW_FLOW_CTRL_CTS:
        cr3 |= USART_CR3_CTSE;
        break;
    case USART_HW_FLOW_CTRL_RTS_CTS:
        cr3 |= USART_CR3_RTSE | USART_CR3_CTSE;
        break;
    }

    /* Step 3: Calculate and set baud rate */
    usart_set_baud_rate(h, h->cfg.baudRate);

    /* Step 4: Write configuration registers (with UE=0) */
    h->regs->CR1 = cr1;
    h->regs->CR2 = cr2;
    h->regs->CR3 = cr3;

    /* Step 5: Clear sticky flags by reading SR then DR */
    (void)h->regs->SR;
    (void)h->regs->DR;

    /* Step 6: Reset runtime state */
    h->pTx = NULL;
    h->txRemaining = 0;
    h->pRx = NULL;
    h->rxRemaining = 0;
    h->flags = 0;

    /* Step 7: Leave UE=0; app will enable via USART_PeripheralControl() */

    return USART_OK;
}

/* =============================================================================
 * USART_DeInit
 *
 * Resets a USART peripheral to its power-on state by pulsing the corresponding
 * bit in RCC->APBxRSTR. This mirrors GPIO_DeInit/SPI_DeInit.
 *
 * Parameters:
 *   pUSARTx : Pointer to USART base (USART1..6, UART4..5)
 * =============================================================================
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
    if (pUSARTx == USART1)
    {
        USART1_REG_RESET();
    }
    else if (pUSARTx == USART2)
    {
        USART2_REG_RESET();
    }
    else if (pUSARTx == USART3)
    {
        USART3_REG_RESET();
    }
    else if (pUSARTx == UART4)
    {
        UART4_REG_RESET();
    }
    else if (pUSARTx == UART5)
    {
        UART5_REG_RESET();
    }
    else if (pUSARTx == USART6)
    {
        USART6_REG_RESET();
    }
}

/* =============================================================================
 * USART_PeripheralControl
 *
 * Enable (UE=1) or disable (UE=0) the USART peripheral.
 * When disabling, waits for TC (transmission complete) to avoid truncating a
 * transfer in progress.
 *
 * Parameters:
 *   h      : Handle to the configured USART instance
 *   EnorDi : ENABLE or DISABLE
 * =============================================================================
 */
void USART_PeripheralControl(USART_Handle_t *h, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        h->regs->CR1 |= USART_CR1_UE;
    }
    else
    {
        /* Wait for TC=1 (transmission complete) before clearing UE */
        while (!(h->regs->SR & USART_SR_TC))
            ;
        h->regs->CR1 &= ~USART_CR1_UE;
    }
}

/* =============================================================================
 * USART_SendData (blocking)
 *
 * Transmits @p len bytes from buffer @p pTx in blocking mode.
 *
 * Algorithm:
 *   For each byte:
 *     1) Wait for TXE=1 (transmit data register empty)
 *     2) Write byte to DR
 *   After all bytes: wait for TC=1 (transmission complete)
 *
 * Parameters:
 *   h   : Handle to the configured USART instance
 *   pTx : Pointer to transmit buffer
 *   len : Number of bytes to transmit
 *
 * Return:
 *   USART_OK on success, USART_ERR_INVAL if parameters are invalid
 * =============================================================================
 */
USART_Status_t USART_SendData(USART_Handle_t *h, const uint8_t *pTx, uint16_t len)
{
    if ((h == NULL) || (pTx == NULL) || (len == 0))
        return USART_ERR_INVAL;

    for (uint16_t i = 0; i < len; i++)
    {
        /* Wait for TXE (transmit data register empty) */
        while (!(h->regs->SR & USART_SR_TXE))
            ;

        /* Write data to DR (8-bit or 9-bit depending on M bit) */
        if (h->cfg.wordLen == USART_WORDLEN_9BITS)
        {
            /* 9-bit mode: use lower 9 bits of DR */
            uint16_t data = pTx[i];
            h->regs->DR = (data & 0x01FFU);
        }
        else
        {
            /* 8-bit mode: use lower 8 bits of DR */
            h->regs->DR = (pTx[i] & 0xFFU);
        }
    }

    /* Wait for TC (transmission complete) */
    while (!(h->regs->SR & USART_SR_TC))
        ;

    return USART_OK;
}

/* =============================================================================
 * USART_ReceiveData (blocking)
 *
 * Receives @p len bytes into buffer @p pRx in blocking mode.
 *
 * Algorithm:
 *   For each byte:
 *     1) Wait for RXNE=1 (read data register not empty)
 *     2) Read byte from DR
 *     3) Check for errors (parity, framing, overrun)
 *
 * Parameters:
 *   h   : Handle to the configured USART instance
 *   pRx : Pointer to receive buffer
 *   len : Number of bytes to receive
 *
 * Return:
 *   USART_OK on success, error code if parity/framing/overrun errors detected
 * =============================================================================
 */
USART_Status_t USART_ReceiveData(USART_Handle_t *h, uint8_t *pRx, uint16_t len)
{
    if ((h == NULL) || (pRx == NULL) || (len == 0))
        return USART_ERR_INVAL;

    for (uint16_t i = 0; i < len; i++)
    {
        /* Wait for RXNE (read data register not empty) */
        while (!(h->regs->SR & USART_SR_RXNE))
            ;

        /* Check for errors */
        uint32_t sr = h->regs->SR;
        if (sr & USART_SR_PE)
            return USART_ERR_PARITY;
        if (sr & USART_SR_FE)
            return USART_ERR_FRAME;
        if (sr & USART_SR_NF)
            return USART_ERR_NOISE;
        if (sr & USART_SR_ORE)
            return USART_ERR_OVERRUN;

        /* Read data from DR (8-bit or 9-bit depending on M bit) */
        if (h->cfg.wordLen == USART_WORDLEN_9BITS)
        {
            /* 9-bit mode: read lower 9 bits */
            uint16_t data = h->regs->DR & 0x01FFU;
            pRx[i] = (uint8_t)data;
        }
        else
        {
            /* 8-bit mode: read lower 8 bits */
            pRx[i] = (uint8_t)(h->regs->DR & 0xFFU);
        }
    }

    return USART_OK;
}

/* =============================================================================
 * USART_SendDataIT (non-blocking / interrupt-driven)
 *
 * Initiates a non-blocking transmit operation. Enables the TXEIE interrupt,
 * which will be serviced by USART_IRQHandling().
 *
 * Parameters:
 *   h   : Handle to the configured USART instance
 *   pTx : Pointer to transmit buffer (must remain valid until transfer completes)
 *   len : Number of bytes to transmit
 *
 * Return:
 *   USART_OK on success, USART_ERR_BUSY if a TX operation is already in progress
 * =============================================================================
 */
USART_Status_t USART_SendDataIT(USART_Handle_t *h, const uint8_t *pTx, uint16_t len)
{
    if ((h == NULL) || (pTx == NULL) || (len == 0))
        return USART_ERR_INVAL;

    if (h->flags & USART_FLAG_TX_BUSY)
        return USART_ERR_BUSY;

    h->pTx = pTx;
    h->txRemaining = len;
    h->flags |= USART_FLAG_TX_BUSY;
    h->flags &= ~USART_FLAG_TX_DONE;

    /* Enable TXEIE interrupt */
    h->regs->CR1 |= USART_CR1_TXEIE;

    return USART_OK;
}

/* =============================================================================
 * USART_ReceiveDataIT (non-blocking / interrupt-driven)
 *
 * Initiates a non-blocking receive operation. Enables the RXNEIE interrupt,
 * which will be serviced by USART_IRQHandling().
 *
 * Parameters:
 *   h   : Handle to the configured USART instance
 *   pRx : Pointer to receive buffer (must remain valid until transfer completes)
 *   len : Number of bytes to receive
 *
 * Return:
 *   USART_OK on success, USART_ERR_BUSY if an RX operation is already in progress
 * =============================================================================
 */
USART_Status_t USART_ReceiveDataIT(USART_Handle_t *h, uint8_t *pRx, uint16_t len)
{
    if ((h == NULL) || (pRx == NULL) || (len == 0))
        return USART_ERR_INVAL;

    if (h->flags & USART_FLAG_RX_BUSY)
        return USART_ERR_BUSY;

    h->pRx = pRx;
    h->rxRemaining = len;
    h->flags |= USART_FLAG_RX_BUSY;
    h->flags &= ~USART_FLAG_RX_DONE;

    /* Enable RXNEIE interrupt */
    h->regs->CR1 |= USART_CR1_RXNEIE;

    return USART_OK;
}

/* =============================================================================
 * USART_IRQHandling
 *
 * USART interrupt service routine helper. Handles:
 *   - TXE:  Transmit data register empty
 *   - RXNE: Receive data register not empty
 *   - TC:   Transmission complete
 *   - IDLE: Idle line detected
 *   - Errors: PE, FE, NF, ORE
 *
 * Call this from your USARTx_IRQHandler() in your startup/application code.
 *
 * Parameters:
 *   h : Handle to the USART instance generating the interrupt
 * =============================================================================
 */
void USART_IRQHandling(USART_Handle_t *h)
{
    uint32_t sr = h->regs->SR;
    uint32_t cr1 = h->regs->CR1;

    /* TXE: Transmit data register empty */
    if ((sr & USART_SR_TXE) && (cr1 & USART_CR1_TXEIE))
    {
        if (h->txRemaining > 0)
        {
            /* Write next byte to DR */
            if (h->cfg.wordLen == USART_WORDLEN_9BITS)
            {
                uint16_t data = *h->pTx;
                h->regs->DR = (data & 0x01FFU);
            }
            else
            {
                h->regs->DR = (*h->pTx & 0xFFU);
            }

            h->pTx++;
            h->txRemaining--;

            if (h->txRemaining == 0)
            {
                /* Disable TXEIE, enable TCIE to wait for transmission complete */
                h->regs->CR1 &= ~USART_CR1_TXEIE;
                h->regs->CR1 |= USART_CR1_TCIE;
            }
        }
    }

    /* TC: Transmission complete */
    if ((sr & USART_SR_TC) && (cr1 & USART_CR1_TCIE))
    {
        /* Disable TCIE */
        h->regs->CR1 &= ~USART_CR1_TCIE;

        /* Set TX_DONE flag, clear TX_BUSY */
        h->flags &= ~USART_FLAG_TX_BUSY;
        h->flags |= USART_FLAG_TX_DONE;
    }

    /* RXNE: Receive data register not empty */
    if ((sr & USART_SR_RXNE) && (cr1 & USART_CR1_RXNEIE))
    {
        if (h->rxRemaining > 0)
        {
            /* Read byte from DR */
            if (h->cfg.wordLen == USART_WORDLEN_9BITS)
            {
                uint16_t data = h->regs->DR & 0x01FFU;
                *h->pRx = (uint8_t)data;
            }
            else
            {
                *h->pRx = (uint8_t)(h->regs->DR & 0xFFU);
            }

            h->pRx++;
            h->rxRemaining--;

            if (h->rxRemaining == 0)
            {
                /* Disable RXNEIE */
                h->regs->CR1 &= ~USART_CR1_RXNEIE;

                /* Set RX_DONE flag, clear RX_BUSY */
                h->flags &= ~USART_FLAG_RX_BUSY;
                h->flags |= USART_FLAG_RX_DONE;
            }
        }
    }

    /* IDLE: Idle line detected */
    if (sr & USART_SR_IDLE)
    {
        /* Clear IDLE flag by reading SR then DR */
        (void)h->regs->SR;
        (void)h->regs->DR;

        h->flags |= USART_FLAG_IDLE;
    }

    /* Error handling: PE, FE, NF, ORE */
    if (sr & USART_SR_PE)
    {
        h->flags |= USART_FLAG_PE;
        /* Clear PE flag by reading SR then DR */
        (void)h->regs->DR;
    }

    if (sr & USART_SR_FE)
    {
        h->flags |= USART_FLAG_FE;
        /* Clear FE flag by reading SR then DR */
        (void)h->regs->DR;
    }

    if (sr & USART_SR_NF)
    {
        h->flags |= USART_FLAG_NE;
        /* Clear NF flag by reading SR then DR */
        (void)h->regs->DR;
    }

    if (sr & USART_SR_ORE)
    {
        h->flags |= USART_FLAG_ORE;
        /* Clear ORE flag by reading SR then DR */
        (void)h->regs->DR;
    }
}

/* =============================================================================
 * NVIC Configuration (IRQ enable/disable/priority)
 * =============================================================================
 */

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            NVIC_ISER0 |= (1U << IRQNumber);
        }
        else if (IRQNumber < 64)
        {
            NVIC_ISER1 |= (1U << (IRQNumber % 32));
        }
        else if (IRQNumber < 96)
        {
            NVIC_ISER2 |= (1U << (IRQNumber % 32));
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            NVIC_ICER0 |= (1U << IRQNumber);
        }
        else if (IRQNumber < 64)
        {
            NVIC_ICER1 |= (1U << (IRQNumber % 32));
        }
        else if (IRQNumber < 96)
        {
            NVIC_ICER2 |= (1U << (IRQNumber % 32));
        }
    }
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    /* Each IPR register holds 4 priority fields (8 bits each).
     * STM32F4 implements the upper 4 bits of each field. */
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(volatile uint32_t*)(0xE000E400 + (iprx * 4)) &= ~(0xFFU << (8 * iprx_section));
    *(volatile uint32_t*)(0xE000E400 + (iprx * 4)) |= (IRQPriority << shift_amount);
}

/* =============================================================================
 * Utility functions
 * =============================================================================
 */

uint8_t USART_IsTxComplete(USART_Handle_t *h)
{
    return (h->regs->SR & USART_SR_TC) ? 1 : 0;
}

uint8_t USART_IsDataAvailable(USART_Handle_t *h)
{
    return (h->regs->SR & USART_SR_RXNE) ? 1 : 0;
}

void USART_ClearErrors(USART_Handle_t *h)
{
    /* Clear error flags by reading SR then DR */
    (void)h->regs->SR;
    (void)h->regs->DR;

    /* Clear error flags in handle */
    h->flags &= ~(USART_FLAG_PE | USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_ORE);
}

USART_Status_t USART_SetBaudRate(USART_Handle_t *h, uint32_t baudRate)
{
    if (h == NULL)
        return USART_ERR_INVAL;

    /* Check if UE is set */
    uint8_t wasEnabled = (h->regs->CR1 & USART_CR1_UE) ? 1 : 0;

    /* Disable USART if it was enabled */
    if (wasEnabled)
    {
        USART_PeripheralControl(h, DISABLE);
    }

    /* Update baud rate in config */
    h->cfg.baudRate = baudRate;

    /* Recalculate and set baud rate */
    usart_set_baud_rate(h, baudRate);

    /* Re-enable USART if it was enabled */
    if (wasEnabled)
    {
        USART_PeripheralControl(h, ENABLE);
    }

    return USART_OK;
}
