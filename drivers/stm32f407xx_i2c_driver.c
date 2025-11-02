/*
 * stm32f407xx_i2c_driver.c
 *
 *  @file    stm32f407xx_i2c_driver.c
 *  @brief   I2C driver implementation for STM32F407.
 *  @ref     RM0090 Rev 21, §27 (I2C interface)
 */

#include "stm32f407xx_i2c_driver.h"
#include <stddef.h>  /* For NULL definition */

/* ===== Private Helper Function Prototypes ================================= */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);

/* =============================================================================
 * Peripheral Clock Control
 * =============================================================================
 */

/**
 * @brief  Enable or disable APB1 clock for I2C peripheral.
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pI2Cx == I2C1)
            I2C1_CLK_EN();
        else if (pI2Cx == I2C2)
            I2C2_CLK_EN();
        else if (pI2Cx == I2C3)
            I2C3_CLK_EN();
    }
    else
    {
        if (pI2Cx == I2C1)
            I2C1_CLK_DIS();
        else if (pI2Cx == I2C2)
            I2C2_CLK_DIS();
        else if (pI2Cx == I2C3)
            I2C3_CLK_DIS();
    }
}

/* =============================================================================
 * Init and De-init
 * =============================================================================
 */

/**
 * @brief  Initialize I2C peripheral registers based on handle configuration.
 */
I2C_Status_t I2C_Init(I2C_Handle_t *pHandle)
{
    if (!pHandle)
        return I2C_ERR_INVAL;

    /* Resolve I2Cx register block from device ID */
    if (pHandle->dev == I2C_DEVICE_I2C1)
        pHandle->pI2Cx = I2C1;
    else if (pHandle->dev == I2C_DEVICE_I2C2)
        pHandle->pI2Cx = I2C2;
    else if (pHandle->dev == I2C_DEVICE_I2C3)
        pHandle->pI2Cx = I2C3;
    else
        return I2C_ERR_INVAL;

    I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;
    uint32_t tempReg = 0;

    /* 1. Enable peripheral clock */
    I2C_PeriClockControl(pI2Cx, ENABLE);

    /* 2. Configure CR1: ACK control (PE=0 at this point) */
    if (pHandle->cfg.ackControl == I2C_ACK_ENABLE)
        tempReg |= I2C_CR1_ACK;
    pI2Cx->CR1 = tempReg;

    /* 3. Configure CR2: FREQ field (APB1 clock frequency in MHz)
     * Assumption: APB1 = 16 MHz (modify based on your clock configuration).
     * For accurate results, read RCC->CFGR and calculate actual APB1 frequency.
     */
    tempReg = 0;
    uint32_t pclk1 = 16;  /* MHz - adjust per your system clock config */
    tempReg |= (pclk1 & 0x3F);  /* FREQ[5:0] */
    pI2Cx->CR2 = tempReg;

    /* 4. Program own address (OAR1) - 7-bit or 10-bit */
    tempReg = 0;
    tempReg |= (pHandle->cfg.ownAddress << 1);  /* ADD[7:1] */
    tempReg |= (1U << 14);  /* Bit 14 should always be kept at 1 (RM0090) */
    if (pHandle->cfg.addrMode == I2C_ADDR_10BIT)
        tempReg |= I2C_OAR1_ADDMODE;
    pI2Cx->OAR1 = tempReg;

    /* 5. Calculate and program CCR (Clock Control Register) */
    uint16_t ccrValue = 0;
    tempReg = 0;

    if (pHandle->cfg.sclSpeed <= I2C_SPEED_SM)
    {
        /* Standard mode: CCR = PCLK1 / (2 * fSCL)
         * Example: PCLK1=16MHz, fSCL=100kHz => CCR = 16M/(2*100k) = 80
         */
        ccrValue = (uint16_t)((pclk1 * 1000000U) / (2U * pHandle->cfg.sclSpeed));
    }
    else
    {
        /* Fast mode: set F/S bit */
        tempReg |= I2C_CCR_FS;

        if (pHandle->cfg.fmDutyCycle == I2C_FM_DUTY_2)
        {
            /* Tlow/Thigh = 2: CCR = PCLK1 / (3 * fSCL) */
            ccrValue = (uint16_t)((pclk1 * 1000000U) / (3U * pHandle->cfg.sclSpeed));
        }
        else
        {
            /* Tlow/Thigh = 16/9: CCR = PCLK1 / (25 * fSCL), DUTY=1 */
            tempReg |= I2C_CCR_DUTY;
            ccrValue = (uint16_t)((pclk1 * 1000000U) / (25U * pHandle->cfg.sclSpeed));
        }
    }

    tempReg |= (ccrValue & 0x0FFF);
    pI2Cx->CCR = tempReg;

    /* 6. Configure TRISE (Maximum rise time register)
     * Sm mode: TRISE = (PCLK1_freq_MHz) + 1
     * Fm mode: TRISE = (PCLK1_freq_MHz * 300ns / 1000ns) + 1 = (PCLK1 * 0.3) + 1
     * Simplified: Sm => PCLK1+1, Fm => (PCLK1*3/10)+1
     */
    if (pHandle->cfg.sclSpeed <= I2C_SPEED_SM)
        tempReg = pclk1 + 1U;
    else
        tempReg = ((pclk1 * 300U) / 1000U) + 1U;

    pI2Cx->TRISE = (tempReg & 0x3F);

    return I2C_OK;
}

/**
 * @brief  De-initialize (reset) I2C peripheral.
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
    if (pI2Cx == I2C1)
        I2C1_REG_RESET();
    else if (pI2Cx == I2C2)
        I2C2_REG_RESET();
    else if (pI2Cx == I2C3)
        I2C3_REG_RESET();
}

/* =============================================================================
 * Helper Functions
 * =============================================================================
 */

/**
 * @brief  Get flag status from SR1 or SR2.
 * @note   For SR2 flags, use bit positions > 15.
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName)
{
    if (flagName & (1U << 31))
    {
        /* SR2 flag (e.g., BUSY, MSL, TRA) - check SR2 */
        return (pI2Cx->SR2 & flagName) ? SET : RESET;
    }
    else
    {
        /* SR1 flag */
        return (pI2Cx->SR1 & flagName) ? SET : RESET;
    }
}

/**
 * @brief  Clear ADDR flag by reading SR1 then SR2.
 */
void I2C_ClearADDRFlag(I2C_Handle_t *pHandle)
{
    uint32_t dummy;
    /* Check device mode (Master or Slave) */
    if (pHandle->pI2Cx->SR2 & I2C_SR2_MSL)
    {
        /* Master mode */
        if (pHandle->state == I2C_STATE_BUSY_RX)
        {
            if (pHandle->rxSize == 1U)
            {
                /* Disable ACK for single byte reception */
                I2C_ManageAcking(pHandle->pI2Cx, DISABLE);

                /* Clear ADDR flag (read SR1, then SR2) */
                dummy = pHandle->pI2Cx->SR1;
                dummy = pHandle->pI2Cx->SR2;
                (void)dummy;  /* Avoid unused variable warning */
            }
        }
        else
        {
            /* Master transmitter or multi-byte receiver */
            dummy = pHandle->pI2Cx->SR1;
            dummy = pHandle->pI2Cx->SR2;
            (void)dummy;
        }
    }
    else
    {
        /* Slave mode */
        dummy = pHandle->pI2Cx->SR1;
        dummy = pHandle->pI2Cx->SR2;
        (void)dummy;
    }
}

/**
 * @brief  Generate START condition (master mode).
 */
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= I2C_CR1_START;
}

/**
 * @brief  Generate STOP condition (master mode).
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= I2C_CR1_STOP;
}

/**
 * @brief  Enable or disable peripheral (CR1.PE).
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
        pI2Cx->CR1 |= I2C_CR1_PE;
    else
        pI2Cx->CR1 &= ~I2C_CR1_PE;
}

/**
 * @brief  Enable or disable ACK (CR1.ACK).
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
        pI2Cx->CR1 |= I2C_CR1_ACK;
    else
        pI2Cx->CR1 &= ~I2C_CR1_ACK;
}

/**
 * @brief  Execute address phase for write (master transmitter).
 * @note   Sends slave address with R/W=0.
 */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
    slaveAddr = slaveAddr << 1;      /* Shift to bits [7:1] */
    slaveAddr &= ~(1U);              /* Clear bit 0 (R/W=0 for write) */
    pI2Cx->DR = slaveAddr;
}

/**
 * @brief  Execute address phase for read (master receiver).
 * @note   Sends slave address with R/W=1.
 */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
    slaveAddr = slaveAddr << 1;      /* Shift to bits [7:1] */
    slaveAddr |= 1U;                 /* Set bit 0 (R/W=1 for read) */
    pI2Cx->DR = slaveAddr;
}

/* =============================================================================
 * Master Data Transfer (Blocking)
 * =============================================================================
 */

/**
 * @brief  Master transmit (blocking).
 */
I2C_Status_t I2C_MasterSendData(I2C_Handle_t *pHandle, uint8_t *pTxBuffer,
                                 uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart)
{
    if (!pHandle || !pTxBuffer || len == 0)
        return I2C_ERR_INVAL;

    I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;

    /* 1. Generate START condition */
    I2C_GenerateStartCondition(pI2Cx);

    /* 2. Wait for SB flag (Start bit sent) */
    while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_SB));

    /* 3. Send slave address with R/W=0 (write) */
    I2C_ExecuteAddressPhaseWrite(pI2Cx, slaveAddr);

    /* 4. Wait for ADDR flag (address sent and acknowledged) */
    while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_ADDR));

    /* 5. Clear ADDR flag */
    I2C_ClearADDRFlag(pHandle);

    /* 6. Send data bytes */
    for (uint32_t i = 0; i < len; i++)
    {
        /* Wait for TxE (transmit buffer empty) */
        while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_TXE));

        pI2Cx->DR = pTxBuffer[i];
    }

    /* 7. Wait for TxE=1 and BTF=1 (byte transfer finished) */
    while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_TXE));
    while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_BTF));

    /* 8. Generate STOP condition (unless repeated start is requested) */
    if (repeatedStart == I2C_DISABLE_SR)
        I2C_GenerateStopCondition(pI2Cx);

    return I2C_OK;
}

/**
 * @brief  Master receive (blocking).
 * @note   Implements special procedures for 1-byte, 2-byte, and N-byte reception.
 */
I2C_Status_t I2C_MasterReceiveData(I2C_Handle_t *pHandle, uint8_t *pRxBuffer,
                                    uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart)
{
    if (!pHandle || !pRxBuffer || len == 0)
        return I2C_ERR_INVAL;

    I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;

    /* 1. Generate START condition */
    I2C_GenerateStartCondition(pI2Cx);

    /* 2. Wait for SB flag */
    while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_SB));

    /* 3. Send slave address with R/W=1 (read) */
    I2C_ExecuteAddressPhaseRead(pI2Cx, slaveAddr);

    /* 4. Wait for ADDR flag */
    while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_ADDR));

    /* === Special handling based on length (RM0090 §27.3.3) === */

    if (len == 1U)
    {
        /* Single byte reception */
        /* Disable ACK */
        I2C_ManageAcking(pI2Cx, DISABLE);

        /* Clear ADDR flag */
        I2C_ClearADDRFlag(pHandle);

        /* Wait for RxNE */
        while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_RXNE));

        /* Generate STOP */
        if (repeatedStart == I2C_DISABLE_SR)
            I2C_GenerateStopCondition(pI2Cx);

        /* Read data */
        *pRxBuffer = (uint8_t)pI2Cx->DR;
    }
    else if (len == 2U)
    {
        /* Two-byte reception (requires POS bit and special handling) */
        /* Set POS bit */
        pI2Cx->CR1 |= I2C_CR1_POS;

        /* Clear ADDR */
        I2C_ClearADDRFlag(pHandle);

        /* Disable ACK */
        I2C_ManageAcking(pI2Cx, DISABLE);

        /* Wait for BTF (both bytes received) */
        while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_BTF));

        /* Generate STOP */
        if (repeatedStart == I2C_DISABLE_SR)
            I2C_GenerateStopCondition(pI2Cx);

        /* Read both bytes */
        pRxBuffer[0] = (uint8_t)pI2Cx->DR;
        pRxBuffer[1] = (uint8_t)pI2Cx->DR;
    }
    else
    {
        /* Multi-byte reception (N > 2) */
        /* Clear ADDR */
        I2C_ClearADDRFlag(pHandle);

        /* Read data until only 2 bytes remain */
        for (uint32_t i = 0; i < (len - 2U); i++)
        {
            /* Wait for RxNE */
            while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_RXNE));

            /* Read data */
            pRxBuffer[i] = (uint8_t)pI2Cx->DR;
        }

        /* Wait for BTF (N-1 byte in DR, N byte in shift register) */
        while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_BTF));

        /* Disable ACK */
        I2C_ManageAcking(pI2Cx, DISABLE);

        /* Read N-1 byte */
        pRxBuffer[len - 2U] = (uint8_t)pI2Cx->DR;

        /* Wait for BTF again */
        while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_BTF));

        /* Generate STOP */
        if (repeatedStart == I2C_DISABLE_SR)
            I2C_GenerateStopCondition(pI2Cx);

        /* Read last byte */
        pRxBuffer[len - 1U] = (uint8_t)pI2Cx->DR;
    }

    /* Re-enable ACK for future operations */
    if (pHandle->cfg.ackControl == I2C_ACK_ENABLE)
        I2C_ManageAcking(pI2Cx, ENABLE);

    return I2C_OK;
}

/* =============================================================================
 * Slave Data Transfer (Blocking)
 * =============================================================================
 */

/**
 * @brief  Slave transmit (blocking).
 */
I2C_Status_t I2C_SlaveSendData(I2C_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t len)
{
    if (!pHandle || !pTxBuffer || len == 0)
        return I2C_ERR_INVAL;

    I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;

    /* Wait for ADDR flag (master has addressed this slave) */
    while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_ADDR));

    /* Clear ADDR */
    I2C_ClearADDRFlag(pHandle);

    /* Send data bytes */
    for (uint32_t i = 0; i < len; i++)
    {
        /* Wait for TxE */
        while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_TXE));

        pI2Cx->DR = pTxBuffer[i];
    }

    /* Wait for AF (acknowledge failure - master sends NACK) */
    while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_AF));

    /* Clear AF flag */
    pI2Cx->SR1 &= ~I2C_SR1_AF;

    return I2C_OK;
}

/**
 * @brief  Slave receive (blocking).
 */
I2C_Status_t I2C_SlaveReceiveData(I2C_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t len)
{
    if (!pHandle || !pRxBuffer || len == 0)
        return I2C_ERR_INVAL;

    I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;

    /* Wait for ADDR flag */
    while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_ADDR));

    /* Clear ADDR */
    I2C_ClearADDRFlag(pHandle);

    /* Receive data bytes */
    for (uint32_t i = 0; i < len; i++)
    {
        /* Wait for RxNE */
        while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_RXNE));

        pRxBuffer[i] = (uint8_t)pI2Cx->DR;
    }

    /* Wait for STOP flag */
    while (!I2C_GetFlagStatus(pI2Cx, I2C_FLAG_STOPF));

    /* Clear STOP flag (read SR1, write CR1) */
    uint32_t dummy = pI2Cx->SR1;
    (void)dummy;
    pI2Cx->CR1 |= 0x0000;

    return I2C_OK;
}

/* =============================================================================
 * Master Data Transfer (Interrupt-driven)
 * =============================================================================
 */

/**
 * @brief  Start non-blocking master transmit.
 */
I2C_Status_t I2C_MasterSendDataIT(I2C_Handle_t *pHandle, uint8_t *pTxBuffer,
                                   uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart)
{
    if (!pHandle || !pTxBuffer || len == 0)
        return I2C_ERR_INVAL;

    if (pHandle->state != I2C_STATE_READY)
        return I2C_ERR_BUSY;

    /* Save transaction parameters */
    pHandle->pTxBuffer = pTxBuffer;
    pHandle->txLen = len;
    pHandle->txCount = 0;
    pHandle->devAddr = slaveAddr;
    pHandle->repeatedStart = repeatedStart;
    pHandle->state = I2C_STATE_BUSY_TX;

    I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;

    /* Generate START */
    I2C_GenerateStartCondition(pI2Cx);

    /* Enable interrupts: ITEVTEN and ITBUFEN */
    pI2Cx->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);

    return I2C_OK;
}

/**
 * @brief  Start non-blocking master receive.
 */
I2C_Status_t I2C_MasterReceiveDataIT(I2C_Handle_t *pHandle, uint8_t *pRxBuffer,
                                      uint32_t len, uint8_t slaveAddr, uint8_t repeatedStart)
{
    if (!pHandle || !pRxBuffer || len == 0)
        return I2C_ERR_INVAL;

    if (pHandle->state != I2C_STATE_READY)
        return I2C_ERR_BUSY;

    /* Save transaction parameters */
    pHandle->pRxBuffer = pRxBuffer;
    pHandle->rxLen = len;
    pHandle->rxSize = len;
    pHandle->rxCount = 0;
    pHandle->devAddr = slaveAddr;
    pHandle->repeatedStart = repeatedStart;
    pHandle->state = I2C_STATE_BUSY_RX;

    I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;

    /* Generate START */
    I2C_GenerateStartCondition(pI2Cx);

    /* Enable interrupts */
    pI2Cx->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);

    return I2C_OK;
}

/**
 * @brief  Close send operation.
 */
void I2C_CloseSendData(I2C_Handle_t *pHandle)
{
    /* Disable interrupts */
    pHandle->pI2Cx->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);

    /* Reset handle members */
    pHandle->pTxBuffer = NULL;
    pHandle->txLen = 0;
    pHandle->txCount = 0;
    pHandle->state = I2C_STATE_READY;
}

/**
 * @brief  Close receive operation.
 */
void I2C_CloseReceiveData(I2C_Handle_t *pHandle)
{
    /* Disable interrupts */
    pHandle->pI2Cx->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);

    /* Reset handle members */
    pHandle->pRxBuffer = NULL;
    pHandle->rxLen = 0;
    pHandle->rxCount = 0;
    pHandle->state = I2C_STATE_READY;
}

/* =============================================================================
 * IRQ Configuration
 * =============================================================================
 */

/**
 * @brief  Enable/Disable NVIC IRQ for I2C.
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    uint8_t regIndex = IRQNumber / 32U;
    uint8_t bitPos   = IRQNumber % 32U;

    if (EnorDi == ENABLE)
    {
        if (regIndex == 0)
            NVIC_ISER0 |= (1U << bitPos);
        else if (regIndex == 1)
            NVIC_ISER1 |= (1U << bitPos);
        else if (regIndex == 2)
            NVIC_ISER2 |= (1U << bitPos);
    }
    else
    {
        if (regIndex == 0)
            NVIC_ICER0 |= (1U << bitPos);
        else if (regIndex == 1)
            NVIC_ICER1 |= (1U << bitPos);
        else if (regIndex == 2)
            NVIC_ICER2 |= (1U << bitPos);
    }
}

/**
 * @brief  Set NVIC priority for I2C IRQ.
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprIndex = IRQNumber / 4U;
    uint8_t section  = IRQNumber % 4U;
    uint8_t shiftAmt = (8U * section) + (8U - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_IPR_BASE + iprIndex) &= ~(0xFF << (8U * section));
    *(NVIC_IPR_BASE + iprIndex) |= (IRQPriority << shiftAmt);
}

/* =============================================================================
 * IRQ Handlers (Event and Error)
 * =============================================================================
 */

/**
 * @brief  I2C Event IRQ handler (handles SB, ADDR, TxE, RxNE, BTF, STOPF).
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pHandle)
{
    I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;
    uint32_t temp1, temp2, temp3;

    /* Check if interrupts are enabled */
    temp1 = pI2Cx->CR2 & I2C_CR2_ITEVTEN;
    temp2 = pI2Cx->CR2 & I2C_CR2_ITBUFEN;

    /* === Event: SB (Start bit sent) === */
    temp3 = pI2Cx->SR1 & I2C_SR1_SB;
    if (temp1 && temp3)
    {
        /* SB flag is set: execute address phase */
        if (pHandle->state == I2C_STATE_BUSY_TX)
            I2C_ExecuteAddressPhaseWrite(pI2Cx, pHandle->devAddr);
        else if (pHandle->state == I2C_STATE_BUSY_RX)
            I2C_ExecuteAddressPhaseRead(pI2Cx, pHandle->devAddr);
    }

    /* === Event: ADDR (Address sent/matched) === */
    temp3 = pI2Cx->SR1 & I2C_SR1_ADDR;
    if (temp1 && temp3)
    {
        /* Clear ADDR flag */
        I2C_ClearADDRFlag(pHandle);
    }

    /* === Event: BTF (Byte transfer finished) === */
    temp3 = pI2Cx->SR1 & I2C_SR1_BTF;
    if (temp1 && temp3)
    {
        if (pHandle->state == I2C_STATE_BUSY_TX)
        {
            /* Make sure TxE is also set (both DR and shift register empty) */
            if (pI2Cx->SR1 & I2C_SR1_TXE)
            {
                /* Transmission complete */
                if (pHandle->txLen == 0)
                {
                    /* Generate STOP or repeated start */
                    if (pHandle->repeatedStart == I2C_DISABLE_SR)
                        I2C_GenerateStopCondition(pI2Cx);

                    /* Close transmission */
                    I2C_CloseSendData(pHandle);

                    /* Notify application */
                    I2C_ApplicationEventCallback(pHandle, I2C_EVENT_TX_CMPLT);
                }
            }
        }
        else if (pHandle->state == I2C_STATE_BUSY_RX)
        {
            /* Reception complete (handled in RxNE for simplicity) */
        }
    }

    /* === Event: TxE (Transmit buffer empty) === */
    temp3 = pI2Cx->SR1 & I2C_SR1_TXE;
    if (temp1 && temp2 && temp3)
    {
        if (pHandle->state == I2C_STATE_BUSY_TX)
        {
            if (pHandle->txLen > 0)
            {
                /* Load data into DR */
                pI2Cx->DR = *(pHandle->pTxBuffer);
                pHandle->pTxBuffer++;
                pHandle->txLen--;
                pHandle->txCount++;
            }
        }
    }

    /* === Event: RxNE (Receive buffer not empty) === */
    temp3 = pI2Cx->SR1 & I2C_SR1_RXNE;
    if (temp1 && temp2 && temp3)
    {
        if (pHandle->state == I2C_STATE_BUSY_RX)
        {
            if (pHandle->rxSize == 1U)
            {
                /* Single byte reception */
                *pHandle->pRxBuffer = (uint8_t)pI2Cx->DR;
                pHandle->rxLen--;
            }
            else if (pHandle->rxSize > 1U)
            {
                if (pHandle->rxLen == 2U)
                {
                    /* Disable ACK */
                    I2C_ManageAcking(pI2Cx, DISABLE);
                }

                /* Read data */
                *pHandle->pRxBuffer = (uint8_t)pI2Cx->DR;
                pHandle->pRxBuffer++;
                pHandle->rxLen--;
            }

            if (pHandle->rxLen == 0)
            {
                /* Reception complete */
                if (pHandle->repeatedStart == I2C_DISABLE_SR)
                    I2C_GenerateStopCondition(pI2Cx);

                /* Close reception */
                I2C_CloseReceiveData(pHandle);

                /* Re-enable ACK if configured */
                if (pHandle->cfg.ackControl == I2C_ACK_ENABLE)
                    I2C_ManageAcking(pI2Cx, ENABLE);

                /* Notify application */
                I2C_ApplicationEventCallback(pHandle, I2C_EVENT_RX_CMPLT);
            }
        }
    }
}

/**
 * @brief  I2C Error IRQ handler (handles BERR, ARLO, AF, OVR, TIMEOUT).
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pHandle)
{
    I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;
    uint32_t temp1, temp2;

    /* Check if error interrupt is enabled */
    temp1 = pI2Cx->CR2 & I2C_CR2_ITERREN;

    /* === Error: BERR (Bus error) === */
    temp2 = pI2Cx->SR1 & I2C_SR1_BERR;
    if (temp1 && temp2)
    {
        /* Clear BERR flag */
        pI2Cx->SR1 &= ~I2C_SR1_BERR;

        /* Notify application */
        I2C_ApplicationEventCallback(pHandle, I2C_ERROR_BERR);
    }

    /* === Error: ARLO (Arbitration lost) === */
    temp2 = pI2Cx->SR1 & I2C_SR1_ARLO;
    if (temp1 && temp2)
    {
        /* Clear ARLO flag */
        pI2Cx->SR1 &= ~I2C_SR1_ARLO;

        /* Notify application */
        I2C_ApplicationEventCallback(pHandle, I2C_ERROR_ARLO);
    }

    /* === Error: AF (Acknowledge failure) === */
    temp2 = pI2Cx->SR1 & I2C_SR1_AF;
    if (temp1 && temp2)
    {
        /* Clear AF flag */
        pI2Cx->SR1 &= ~I2C_SR1_AF;

        /* Notify application */
        I2C_ApplicationEventCallback(pHandle, I2C_ERROR_AF);
    }

    /* === Error: OVR (Overrun/Underrun) === */
    temp2 = pI2Cx->SR1 & I2C_SR1_OVR;
    if (temp1 && temp2)
    {
        /* Clear OVR flag */
        pI2Cx->SR1 &= ~I2C_SR1_OVR;

        /* Notify application */
        I2C_ApplicationEventCallback(pHandle, I2C_ERROR_OVR);
    }

    /* === Error: TIMEOUT === */
    temp2 = pI2Cx->SR1 & I2C_SR1_TIMEOUT;
    if (temp1 && temp2)
    {
        /* Clear TIMEOUT flag */
        pI2Cx->SR1 &= ~I2C_SR1_TIMEOUT;

        /* Notify application */
        I2C_ApplicationEventCallback(pHandle, I2C_ERROR_TIMEOUT);
    }
}

/* =============================================================================
 * Application Callback (Weak implementation)
 * =============================================================================
 */

/**
 * @brief  Weak callback function (user can override in application code).
 */
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle, I2C_Event_t event)
{
    /* Default implementation: do nothing.
     * User should override this in application code to handle events.
     */
    (void)pHandle;
    (void)event;
}
