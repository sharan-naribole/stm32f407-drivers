#include "stm32f407xx_spi_driver.h"
#include <stddef.h>   // for NULL
#include <string.h>   // for memcpy

/* =============================================================================
 * SPI_PeriClockControl
 *
 * Enables or disables the APB peripheral clock for a given SPI instance.
 *
 * Parameters:
 *   pSPIx : Pointer to SPI base (SPI1, SPI2, SPI3)
 *   EnorDi: ENABLE (1) to turn clock ON, DISABLE (0) to turn it OFF
 *
 * Notes:
 * - SPI1 is on APB2; SPI2 and SPI3 are on APB1 (see stm32f407xx.h memory map).
 * - Macros SPI1_CLK_EN/DIS, SPI2_CLK_EN/DIS, SPI3_CLK_EN/DIS toggle the
 *   appropriate ENR bits in RCC.
 * - A dummy read-back of the corresponding ENR is performed after the write to:
 *      1) ensure the write has reached the bus (flush write buffers), and
 *      2) guarantee the peripheral is clocked before its registers are accessed.
 * - This mirrors the pattern used in GPIO_PeriClockControl() for consistency.
 * =============================================================================
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (pSPIx == SPI1)
	{
		if (EnorDi == ENABLE) { SPI1_CLK_EN(); (void)RCC->APB2ENR; }
		else                  { SPI1_CLK_DIS(); (void)RCC->APB2ENR; }
	}
	else if (pSPIx == SPI2)
	{
		if (EnorDi == ENABLE) { SPI2_CLK_EN(); (void)RCC->APB1ENR; }
		else                  { SPI2_CLK_DIS(); (void)RCC->APB1ENR; }
	}
	else if (pSPIx == SPI3)
	{
		if (EnorDi == ENABLE) { SPI3_CLK_EN(); (void)RCC->APB1ENR; }
		else                  { SPI3_CLK_DIS(); (void)RCC->APB1ENR; }
	}
	else
	{
		/* Invalid SPI pointer; optionally assert in debug builds. */
	}
}

/* =============================================================================
 * Local helper: spi_resolve_regs
 *
 * Maps a logical device ID (SPI_DEVICE_SPI1..3) to the corresponding
 * peripheral base pointer (SPI1..SPI3).
 *
 * Parameters:
 *   dev : Logical device ID (enum SPI_Device_t)
 *
 * Return:
 *   Pointer to SPI register block (SPI_RegDef_t*), or NULL if invalid.
 *
 * Notes:
 * - Keeps SPI_Init() cleaner by centralizing the device→pointer mapping.
 * - Application code never calls this directly; it’s purely internal.
 * =============================================================================
 */
static inline SPI_RegDef_t* spi_resolve_regs(SPI_Device_t dev)
{
	switch (dev) {
	case SPI_DEVICE_SPI1: return SPI1;
	case SPI_DEVICE_SPI2: return SPI2;
	case SPI_DEVICE_SPI3: return SPI3;
	default:              return (SPI_RegDef_t*)0;
	}
}

/* =============================================================================
 * SPI_Init
 *
 * Initializes the SPI peripheral as described in h->cfg, targeting the
 * logical device selected in h->dev. This mirrors the GPIO_Init pattern:
 * - Application pre-fills h->dev and h->cfg
 * - SPI_Init() resolves h->regs and programs registers
 *
 * Implementation steps:
 *   0) Resolve SPIx base pointer from h->dev and cache in h->regs
 *   1) Enable APB clock for this SPI instance (via SPI_PeriClockControl)
 *   2) Build CR1/CR2 shadow values based on h->cfg:
 *        - Mode (master/slave), baud rate, CPOL/CPHA
 *        - Frame format (8/16-bit, MSB/LSB first)
 *        - Bus topology (full-duplex, simplex-RX, half-duplex)
 *        - NSS handling (software/hardware)
 *        - TI frame mode, CRC enable/polynomial
 *   3) Write CR1 and CR2 to hardware with SPE=0 (per ST recommended order)
 *   4) Clear any sticky flags by dummy read of SR then DR
 *   5) Leave SPE=0 for glitch-free bring-up (app enables later via
 *      SPI_PeripheralControl)
 *   6) Reset runtime state in the handle (flags, buffers, counters)
 *
 * Parameters:
 *   h : Handle with h->dev and h->cfg pre-populated
 *
 * Return:
 *   SPI_OK on success
 *   SPI_ERR_INVAL if handle is NULL or device invalid
 *
 * Notes:
 * - Safe-order programming ensures pins/CS lines are stable before
 *   the SPI peripheral starts driving the bus.
 * - Half-duplex direction (BIDIOE) must be toggled at transfer time.
 * =============================================================================
 */
SPI_Status_t SPI_Init(SPI_Handle_t *h)
{
	if (!h) return SPI_ERR_INVAL;

	/* 0) Resolve register block from logical device and store in handle */
	h->regs = spi_resolve_regs(h->dev);
	if (!h->regs) return SPI_ERR_INVAL;
	SPI_RegDef_t *r = h->regs;
	const SPI_Config_t *c = &h->cfg;

	/* 1) Enable peripheral clock for this SPI instance */
	SPI_PeriClockControl(r, ENABLE);

	/* 2) Build CR1 in a local shadow (keep SPE=0 during config) */
	uint32_t cr1 = 0;

	/* Role: Master/Slave */
	if (c->mode == SPI_MODE_MASTER) cr1 |= SPI_CR1_MSTR;

	/* Baud rate (master only): BR[2:0] */
	if (c->mode == SPI_MODE_MASTER) {
		/* BR[2:0] lives at bits 5:3 on F4 — use the Pos macro from stm32f407xx.h */
		cr1 |= ((uint32_t)(c->baudDiv & 0x7U) << SPI_CR1_BR_Pos);
	}

	/* CPOL/CPHA */
	if (c->cpol == SPI_CPOL_1) cr1 |= SPI_CR1_CPOL;
	if (c->cpha == SPI_CPHA_1) cr1 |= SPI_CR1_CPHA;

	/* Data frame size: 8/16-bit */
	if (c->datasize == SPI_DFF_16BIT) cr1 |= SPI_CR1_DFF;

	/* First bit: MSB/LSB */
	if (c->firstBit == SPI_FIRSTBIT_LSB) cr1 |= SPI_CR1_LSBFIRST;

	/* Bus topology: Full-duplex / Simplex-RX / Half-duplex (1-line) */
	switch (c->bus) {
	case SPI_BUS_FULL_DUPLEX:
		/* BIDIMODE=0, RXONLY=0 */
		cr1 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_RXONLY);
		break;
	case SPI_BUS_SIMPLEX_RX:
		/* 2-line unidirectional: RXONLY=1, BIDIMODE=0 */
		cr1 |=  SPI_CR1_RXONLY;
		cr1 &= ~SPI_CR1_BIDIMODE;
		break;
	case SPI_BUS_HALF_DUPLEX:
		/* 1-line bidirectional: BIDIMODE=1, BIDIOE depends on TX/RX at runtime */
		cr1 |=  SPI_CR1_BIDIMODE;
		cr1 &= ~SPI_CR1_RXONLY;
		/* Note: BIDIOE is in CR1 as well; set/clear it at transfer start as needed */
		break;
	}

	/* NSS handling (software vs hardware) */
	uint32_t cr2 = 0;
	if (c->nss == SPI_NSS_SOFTWARE) {
		cr1 |= SPI_CR1_SSM;
		if (c->mode == SPI_MODE_MASTER) {
			cr1 |= SPI_CR1_SSI;   // master needs SSI=1 to avoid MODF
		} else {
			cr1 &= ~SPI_CR1_SSI;  // slave must have SSI=0 so it's 'selected'
		}
	} else {
		/* Hardware management: SSM=0 */
		cr1 &= ~SPI_CR1_SSM;
		/* Master with HW output drives NSS via SSOE (CR2) */
		if (c->nss == SPI_NSS_HARD_OUTPUT && c->mode == SPI_MODE_MASTER) {
			cr2 |= SPI_CR2_SSOE;
		}
		/* Slave with HW input just samples NSS; nothing else needed here */
	}

	/* TI frame format (CR2.FRF) */
	if (c->tiMode) cr2 |= SPI_CR2_FRF;

	/* CRC enable and polynomial */
	if (c->crcEnable) {
		cr1 |= SPI_CR1_CRCEN;
		r->CRCPR = (uint32_t)c->crcPolynomial; /* default reset is 0x7 */
	} else {
		cr1 &= ~SPI_CR1_CRCEN;
	}

	/* 3) Program CR1 (with SPE=0) and CR2 */
	r->CR1 = cr1 & ~SPI_CR1_SPE;
	r->CR2 = cr2;

	/* 4) Clear sticky flags by SR then DR read (see RM0090 errata-safe sequence) */
	(void)r->SR;
	(void)r->DR;

	/* 5) Leave SPE=0; user enables later via SPI_PeripheralControl(h, ENABLE) */
	/* h->flags runtime state reset */
	h->flags = 0;
	h->pTx = NULL; h->txRemaining = 0;
	h->pRx = NULL; h->rxRemaining = 0;

	return SPI_OK;
}

/* =============================================================================
 * SPI_DeInit
 *
 * Resets a SPI peripheral to its power-on state by pulsing the corresponding
 * RCC reset bit. This mirrors GPIO_DeInit() style (port-wide reset via RCC).
 *
 * Parameters:
 *   pSPIx : Target SPI base (SPI1, SPI2, or SPI3)
 *
 * What it does:
 *   - Pulses RCC->APB2RSTR bit 12 for SPI1 (APB2 domain), or
 *     RCC->APB1RSTR bit 14/15 for SPI2/SPI3 (APB1 domain).
 *   - Leaves the peripheral clock gate unchanged (on or off as-is).
 *
 * Notes:
 *   - This resets ALL registers of the selected SPI instance (CR1/CR2/SR/DR/etc.).
 *   - If your app requires the clock off after reset, call SPI_PeriClockControl()
 *     with DISABLE explicitly.
 * =============================================================================
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else {
		/* Invalid pointer; optionally assert() in debug builds. */
	}
}


/* =============================================================================
 * Utility Functions
 * =============================================================================
 * Local helper: spi_pop_frame()
 *
 * Reads one frame from a byte pointer based on the active data size (8/16-bit),
 * advances the pointer, and returns the frame as a 16-bit value (lower 8 bits
 * used when DFF=8). Using memcpy avoids unaligned halfword accesses when the
 * caller's buffer isn't 16-bit aligned.
 *
 * in/out:
 *   src    : *in  pointer to current TX byte
 *            *out advanced by 1 (8-bit) or 2 (16-bit)
 *   is16   : 0 => 8-bit frames, 1 => 16-bit frames
 * returns:
 *   0x00..0xFF / 0x0000..0xFFFF frame value to write to DR
 * ---------------------------------------------------------------------------
 */
static inline uint16_t spi_pop_frame(const uint8_t **src, uint8_t is16)
{
	if (!is16) {
		uint16_t v = (uint16_t)(**src);
		(*src)++;
		return v;
	} else {
		uint16_t v;
		/* Safe read even if *src is unaligned */
		memcpy(&v, *src, sizeof(uint16_t));
		(*src) += 2U;
		return v;
	}
}

/**
 * @brief  Check if SPI peripheral is busy (BSY flag in SR).
 * @param  h  SPI handle.
 * @return 1 if busy (transfer in progress), 0 if idle.
 *
 * BSY Flag Meaning:
 * - 0: SPI is not busy (idle)
 * - 1: SPI is busy in communication or TX buffer is not empty
 *
 * Usage:
 *   if (SPI_IsBusy(&spi_handle)) {
 *       // Wait for current transfer to complete
 *   }
 *
 * Notes:
 * - This checks the hardware BSY flag, not the driver's busy flags
 * - BSY=1 means the peripheral is actively shifting data or TX buffer has data
 * - Safe to call anytime, even during interrupt transfers
 */
uint8_t SPI_IsBusy(SPI_Handle_t *h)
{
	if (!h || !h->regs) {
		return 0U;  /* Invalid handle = not busy */
	}

	return (uint8_t)((h->regs->SR & SPI_SR_BSY) ? 1U : 0U);
}

/**
 * @brief  Clear OVR (overrun) fault condition.
 * @param  h  SPI handle.
 *
 * OVR Error Condition:
 * - Occurs when received data is not read before next frame arrives
 * - Common in slave mode when master sends data too fast
 * - Can also occur in master mode with poor interrupt timing
 *
 * Clear Sequence (per STM32 reference manual):
 * 1. Wait for BSY=0 (bus idle) - ensures safe timing
 * 2. Read DR register (clears received data)
 * 3. Read SR register (clears OVR flag)
 *
 * Usage:
 *   if (SPI_GetFlags(&h) & SPI_FLAG_OVR) {
 *       SPI_ClearOVR(&h);  // Clear hardware and driver flags
 *   }
 *
 * Notes:
 * - This function waits for BSY=0 before clearing (may block briefly)
 * - Clears both hardware OVR flag and driver SPI_FLAG_OVR
 * - Safe to call even if OVR is not set
 */
void SPI_ClearOVR(SPI_Handle_t *h)
{
	if (!h || !h->regs) {
		return;
	}

	/* Wait for bus idle before clearing OVR - ensures safe timing */
	while (h->regs->SR & SPI_SR_BSY) {
		/* Spin wait - typically very brief */
	}

	/* Clear OVR flag using standard sequence: read DR then SR */
	(void)h->regs->DR;   /* Read DR (clears any pending data) */
	(void)h->regs->SR;   /* Read SR (clears OVR flag) */

	/* Clear the corresponding driver flag */
	SPI_ClearFlags(h, SPI_FLAG_OVR);
}

/* -----------------------------------------------------------------------------
 * Local helper: spi_push_frame()
 *
 * Writes one frame to a byte buffer based on the active data size (8/16-bit),
 * advances the pointer. Counterpart to spi_pop_frame() for RX operations.
 * Using memcpy avoids unaligned halfword accesses when the caller's buffer
 * isn't 16-bit aligned.
 *
 * in/out:
 *   dst    : *in  pointer to current RX byte destination
 *            *out advanced by 1 (8-bit) or 2 (16-bit)
 *   frame  : 16-bit value read from DR (only lower 8 bits used when DFF=8)
 *   is16   : 0 => 8-bit frames, 1 => 16-bit frames
 * --------------------------------------------------------------------------- */
static inline void spi_push_frame(uint8_t **dst, uint16_t frame, uint8_t is16)
{
	if (!is16) {
		**dst = (uint8_t)(frame & 0xFFU);
		(*dst)++;
	} else {
		/* Safe write even if *dst is unaligned */
		memcpy(*dst, &frame, sizeof(uint16_t));
		(*dst) += 2U;
	}
}

/* =============================================================================
 * SPI_SendData — Blocking transmit for all SPI modes
 *
 * State Machine Flow:
 *   ┌──────┐    SPI_SendData()    ┌─────────┐
 *   │ IDLE │────────────────────→ │ TX_BUSY │
 *   └──────┘                      └─────────┘
 *      ↑                              │
 *      │ Set TX_DONE                  │ Transfer Loop
 *      │ Clear TX_BUSY                │ (len times)
 *      │                              ▼
 *   ┌─────────┐    Wait TXE=1     ┌─────────────┐
 *   │ TX_DONE │◄───────────────── │ Frame Send  │
 *   └─────────┘    Wait BSY=0     └─────────────┘
 *                                      │ ↑
 *                                      └─┘ Next frame
 *
 * Transfer Loop (per frame):
 *        ┌────────────────┐
 *        │ Wait TXE=1     │ ← TX buffer empty?
 *        │ (spin)         │
 *        └────────┬───────┘
 *                 ▼
 *        ┌────────────────┐
 *        │ DR ← frame     │ ← Write data to shift register
 *        └────────┬───────┘
 *                 ▼
 *     ┌──────────────────────┐
 *     │ Full-duplex mode?    │
 *     └──────┬───────────────┘
 *            │ Yes            │ No
 *            ▼                ▼
 *   ┌────────────────┐  ┌──────────────┐
 *   │ Wait RXNE=1    │  │ Check OVR    │
 *   │ Dummy read DR  │  │ Continue     │
 *   └────────────────┘  └──────────────┘
 *            │                │
 *            └────────┬───────┘
 *                     ▼
 *            ┌────────────────┐
 *            │ Check OVR      │ ← Error handling
 *            │ Next frame     │
 *            └────────────────┘
 *
 * Final Sequence:
 *   1. Wait TXE=1 (last frame loaded)
 *   2. Wait BSY=0 (transmission complete)
 *   3. Set TX_DONE flag, clear TX_BUSY
 *
 * Supported Modes:
 *   - Full-duplex: TX with RX dummy reads to prevent overrun
 *   - Half-duplex: Sets BIDIOE=1 for TX direction, restores on exit
 *   - Master/Slave: Works for both (slave needs external clock)
 *   - 8/16-bit: Frame size handled automatically based on DFF
 *
 * Error Handling:
 *   - OVR: Clears error, sets OVR flag, returns SPI_ERR_OVR
 *   - Invalid config: Returns SPI_ERR_INVAL (e.g., simplex-RX mode)
 *
 * @param h    SPI handle with initialized peripheral (SPE=1)
 * @param pTx  Pointer to transmit buffer
 * @param len  Number of frames to send (bytes for 8-bit, words for 16-bit)
 * @return     SPI_OK on success, error code on failure
 * =============================================================================
 */
SPI_Status_t SPI_SendData(SPI_Handle_t *h, const void *pTx, uint16_t len)
{
	if (!h || !h->regs || !pTx) return SPI_ERR_INVAL;

	/* Add hardware busy check using utility function */
	if (SPI_IsBusy(h)) {
		return SPI_ERR_BUSY;
	}

	SPI_RegDef_t *r = h->regs;

	/* Snapshot CR1 once; derive mode & frame size */
	const uint32_t cr1_initial = r->CR1;
	const uint8_t  is_1line    = (uint8_t)((cr1_initial & SPI_CR1_BIDIMODE) ? 1U : 0U);
	const uint8_t  is_rxonly   = (uint8_t)((cr1_initial & SPI_CR1_RXONLY)   ? 1U : 0U);
	const uint8_t  is_16bit    = (uint8_t)((cr1_initial & SPI_CR1_DFF)      ? 1U : 0U);

	/* Invalid: 2-line Simplex RX cannot transmit */
	if (!is_1line && is_rxonly) return SPI_ERR_INVAL;

	/* Flag book-keeping */
	SPI_ClearFlags(h, (uint16_t)(SPI_FLAG_TX_DONE));  /* start clean */
	h->flags |= SPI_FLAG_TX_BUSY;

	/* Half-duplex: force TX direction for the duration */
	const uint8_t bidioe_was_set = (uint8_t)((cr1_initial & SPI_CR1_BIDIOE) ? 1U : 0U);
	if (is_1line) {
		r->CR1 = cr1_initial | SPI_CR1_BIDIOE;
		(void)r->CR1;
	}

	const uint8_t *src = (const uint8_t*)pTx;

	while (len > 0U) {
		/* Wait TX buffer empty */
		while ((r->SR & SPI_SR_TXE) == 0U) { /* spin */ }

		/* Push one frame to DR (works for 8- or 16-bit) */
		r->DR = (uint32_t)spi_pop_frame(&src, is_16bit);
		len--;

		/* In 2-line full-duplex, consume RX to avoid OVR */
		if (!is_1line) {
			while ((r->SR & SPI_SR_RXNE) == 0U) { /* spin */ }
			(void)r->DR; /* dummy read clears RXNE */
		}

		/* Enhanced OVR check using utility function */
		if (r->SR & SPI_SR_OVR) {
			/* Use utility function for proper OVR clearing */
			SPI_ClearOVR(h);
			h->flags &= (uint16_t)~SPI_FLAG_TX_BUSY;
			h->flags |= SPI_FLAG_OVR;
			if (is_1line && !bidioe_was_set) { r->CR1 &= ~SPI_CR1_BIDIOE; (void)r->CR1; }
			return SPI_ERR_OVR;
		}
	}

	/* Drain last frame & ensure bus is idle using utility function */
	while ((r->SR & SPI_SR_TXE) == 0U) { /* spin */ }
	while (SPI_IsBusy(h)) { /* spin using utility function */ }

	/* Restore half-duplex direction if we changed it */
	if (is_1line && !bidioe_was_set) {
		r->CR1 &= ~SPI_CR1_BIDIOE;
		(void)r->CR1;
	}

	h->flags &= (uint16_t)~SPI_FLAG_TX_BUSY;
	h->flags |= SPI_FLAG_TX_DONE;

	return SPI_OK;
}

/* =============================================================================
 * SPI_ReceiveData — Simplified blocking receive (covers 95% of use cases)
 *
 * State Machine Flow:
 *   ┌──────┐   SPI_ReceiveData()  ┌─────────┐
 *   │ IDLE │─────────────────────→│ RX_BUSY │
 *   └──────┘                      └─────────┘
 *      ↑                               │
 *      │ Set RX_DONE                   │ Transfer Loop
 *      │ Clear RX_BUSY                 │ (len times)
 *      │                               ▼
 *   ┌─────────┐   Wait BSY=0      ┌─────────────┐
 *   │ RX_DONE │◄────────────────  │ Frame Recv  │
 *   └─────────┘                   └─────────────┘
 *                                      │ ↑
 *                                      └─┘ Next frame
 *
 * Master Mode Transfer Loop:
 *        ┌────────────────┐
 *        │ Wait TXE=1     │ ← TX buffer empty?
 *        │ Send dummy     │
 *        └────────┬───────┘
 *                 ▼
 *        ┌────────────────┐
 *        │ DR ← 0xFF      │ ← Send dummy to generate clock
 *        └────────┬───────┘
 *                 ▼
 *        ┌────────────────┐
 *        │ Wait RXNE=1    │ ← Receive data ready?
 *        │ (spin)         │
 *        └────────┬───────┘
 *                 ▼
 *        ┌────────────────┐
 *        │ data ← DR      │ ← Read received data
 *        │ Store in buf   │
 *        └────────┬───────┘
 *                 ▼
 *        ┌────────────────┐
 *        │ Check OVR      │ ← Error handling
 *        │ Next frame     │
 *        └────────────────┘
 *
 * Slave Mode Transfer Loop:
 *        ┌────────────────┐
 *        │ Wait RXNE=1    │ ← Wait for master's clock
 *        │ (spin)         │
 *        └────────┬───────┘
 *                 ▼
 *        ┌────────────────┐
 *        │ data ← DR      │ ← Read received data
 *        │ Store in buf   │
 *        └────────┬───────┘
 *                 ▼
 *        ┌────────────────┐
 *        │ Check OVR      │ ← Error handling
 *        │ Next frame     │
 *        └────────────────┘
 *
 * Design Philosophy:
 *   - Simplified from complex multi-mode version
 *   - Handles most common SPI receive scenarios
 *   - Master automatically sends dummy data for clock generation
 *   - Slave just waits for master's clock
 *
 * Typical Use Cases:
 *   - Master reading sensor data (sends 0xFF dummy bytes)
 *   - Slave receiving commands from master
 *   - Simple request/response protocols
 *
 * @param h    SPI handle with initialized peripheral (SPE=1)
 * @param pRx  Pointer to receive buffer
 * @param len  Number of frames to receive (bytes for 8-bit, words for 16-bit)
 * @return     SPI_OK on success, error code on failure
 * @note       Master mode: Automatically sends dummy bytes to generate clocks
 * @note       Slave mode: Waits for master to provide clocks
 * =============================================================================
 */
SPI_Status_t SPI_ReceiveData(SPI_Handle_t *h, void *pRx, uint16_t len)
{
	if (!h || !h->regs || !pRx || len == 0U) return SPI_ERR_INVAL;

	/* Add hardware busy check using utility function */
	if (SPI_IsBusy(h)) {
		return SPI_ERR_BUSY;
	}

	SPI_RegDef_t *r = h->regs;

	/* Read configuration once */
	const uint32_t cr1 = r->CR1;
	const uint8_t is_16bit = (uint8_t)((cr1 & SPI_CR1_DFF) ? 1U : 0U);
	const uint8_t is_master = (uint8_t)((cr1 & SPI_CR1_MSTR) ? 1U : 0U);

	/* Flag management */
	SPI_ClearFlags(h, SPI_FLAG_RX_DONE);
	h->flags |= SPI_FLAG_RX_BUSY;

	uint8_t *dst = (uint8_t*)pRx;
	const uint16_t dummy_byte = 0xFF;  /* Standard dummy value */

	while (len > 0U) {
		/* Master: Send dummy data to generate clocks */
		if (is_master) {
			while ((r->SR & SPI_SR_TXE) == 0U) { /* wait for TX empty */ }
			r->DR = dummy_byte;
		}

		/* Both master and slave: Wait for received data */
		while ((r->SR & SPI_SR_RXNE) == 0U) { /* wait for RX data */ }

		/* Read and store the received data */
		uint16_t received = (uint16_t)r->DR;
		spi_push_frame(&dst, received, is_16bit);
		len--;

		/* Enhanced error check using utility function */
		if (r->SR & SPI_SR_OVR) {
			/* Use utility function for proper OVR clearing */
			SPI_ClearOVR(h);
			h->flags &= ~SPI_FLAG_RX_BUSY;
			h->flags |= SPI_FLAG_OVR;
			return SPI_ERR_OVR;
		}
	}

	/* Wait for completion using utility function */
	while (SPI_IsBusy(h)) { /* wait for idle using utility function */ }

	h->flags &= ~SPI_FLAG_RX_BUSY;
	h->flags |= SPI_FLAG_RX_DONE;
	return SPI_OK;
}

/* =============================================================================
 * SPI_SendReceiveData — Explicit full-duplex transfer
 *
 * State Machine Flow:
 *   ┌──────┐  SPI_SendReceiveData()  ┌─────────────────┐
 *   │ IDLE │────────────────────────→│ TX_BUSY+RX_BUSY │
 *   └──────┘                         └─────────────────┘
 *      ↑                                      │
 *      │ Set TX_DONE+RX_DONE                  │ Transfer Loop
 *      │ Clear BUSY flags                     │ (len times)
 *      │                                      ▼
 *   ┌──────────────────┐   Wait BSY=0   ┌────────────────┐
 *   │ TX_DONE+RX_DONE  │◄────────────── │ Frame Exchange │
 *   └──────────────────┘                └────────────────┘
 *                                              │ ↑
 *                                              └─┘ Next frame
 *
 * Transfer Loop (Full-Duplex Exchange):
 *        ┌────────────────┐
 *        │ Wait TXE=1     │ ← TX buffer empty?
 *        │ (spin)         │
 *        └────────┬───────┘
 *                 ▼
 *        ┌────────────────┐
 *        │ DR ← tx_frame  │ ← Send actual command/data
 *        └────────┬───────┘
 *                 ▼
 *        ┌────────────────┐
 *        │ Wait RXNE=1    │ ← Response ready?
 *        │ (spin)         │
 *        └────────┬───────┘
 *                 ▼
 *        ┌────────────────┐
 *        │ rx_frame ← DR  │ ← Read meaningful response
 *        │ Store in buf   │
 *        └────────┬───────┘
 *                 ▼
 *        ┌────────────────┐
 *        │ Check OVR      │ ← Error handling
 *        │ Next frame     │
 *        └────────────────┘
 *
 * When to Use vs SPI_ReceiveData:
 *
 *   Use SPI_SendReceiveData for:        Use SPI_ReceiveData for:
 *   ┌─────────────────────┐            ┌─────────────────────┐
 *   │ Register read/write │            │ Sensor data read    │
 *   │ Command + response  │            │ Simple RX operation │
 *   │ Memory operations   │            │ Dummy TX is fine    │
 *   │ Both TX/RX matter   │            │ Don't care about TX │
 *   └─────────────────────┘            └─────────────────────┘
 *
 * Example Scenarios:
 *   - Reading register: Send register address, receive register value
 *   - ADC conversion: Send conversion command, receive result
 *   - Memory read: Send read command + address, receive data
 *
 * @param h    SPI handle with initialized peripheral (SPE=1)
 * @param pTx  Pointer to transmit buffer (meaningful data, not dummy)
 * @param pRx  Pointer to receive buffer
 * @param len  Number of frames to transfer (bytes for 8-bit, words for 16-bit)
 * @return     SPI_OK on success, error code on failure
 * @note       Only works in full-duplex mode (BIDIMODE=0, RXONLY=0)
 * @note       Both TX and RX data are processed simultaneously
 * =============================================================================
 */
SPI_Status_t SPI_SendReceiveData(SPI_Handle_t *h, const void *pTx, void *pRx, uint16_t len)
{
	if (!h || !h->regs || !pTx || !pRx || len == 0U) return SPI_ERR_INVAL;

	/* Add hardware busy check using utility function */
	if (SPI_IsBusy(h)) {
		return SPI_ERR_BUSY;
	}

	SPI_RegDef_t *r = h->regs;
	const uint8_t is_16bit = (uint8_t)((r->CR1 & SPI_CR1_DFF) ? 1U : 0U);

	/* Flag management */
	SPI_ClearFlags(h, SPI_FLAG_TX_DONE | SPI_FLAG_RX_DONE);
	h->flags |= (SPI_FLAG_TX_BUSY | SPI_FLAG_RX_BUSY);

	const uint8_t *src = (const uint8_t*)pTx;
	uint8_t *dst = (uint8_t*)pRx;

	while (len > 0U) {
		/* Send data */
		while ((r->SR & SPI_SR_TXE) == 0U) { /* wait */ }
		r->DR = spi_pop_frame(&src, is_16bit);

		/* Receive data */
		while ((r->SR & SPI_SR_RXNE) == 0U) { /* wait */ }
		uint16_t received = (uint16_t)r->DR;
		spi_push_frame(&dst, received, is_16bit);

		len--;

		/* Enhanced error check using utility function */
		if (r->SR & SPI_SR_OVR) {
			/* Use utility function for proper OVR clearing */
			SPI_ClearOVR(h);
			h->flags &= ~(SPI_FLAG_TX_BUSY | SPI_FLAG_RX_BUSY);
			h->flags |= SPI_FLAG_OVR;
			return SPI_ERR_OVR;
		}
	}

	/* Wait for completion using utility functions */
	while ((r->SR & SPI_SR_TXE) == 0U) { /* wait */ }
	while (SPI_IsBusy(h)) { /* wait for idle using utility function */ }

	h->flags &= ~(SPI_FLAG_TX_BUSY | SPI_FLAG_RX_BUSY);
	h->flags |= (SPI_FLAG_TX_DONE | SPI_FLAG_RX_DONE);
	return SPI_OK;
}

/* =============================================================================
 * SPI_PeripheralControl — Enable/Disable SPI peripheral operation
 *
 * State Control:
 *   ┌─────────────┐     ENABLE      ┌─────────────┐
 *   │ SPE=0       │────────────────→│ SPE=1       │
 *   │ (Inactive)  │                 │ (Active)    │
 *   │ Config OK   │                 │ Pins Active │
 *   └─────────────┘                 └─────────────┘
 *          ↑                               │
 *          │ DISABLE                       │
 *          │ (Wait BSY=0)                  │
 *          └───────────────────────────────┘
 *
 * Hardware State Transitions:
 *
 *   SPE=0 (DISABLE):                SPE=1 (ENABLE):
 *   ┌─────────────────┐            ┌─────────────────┐
 *   │ • Pins = GPIO   │            │ • Pins = SPI    │
 *   │ • No clocks     │            │ • Clocks active │
 *   │ • Config R/W    │            │ • Config R/O    │
 *   │ • No transfers  │            │ • Transfers OK  │
 *   └─────────────────┘            └─────────────────┘
 *
 * Safety Features:
 *   - DISABLE waits for BSY=0 to avoid truncating active transfers
 *   - Prevents glitches during configuration changes
 *   - Ensures clean state transitions
 *
 * Typical Usage Sequence:
 *            ┌─────────────┐
 *            │ SPI_Init()  │ ← Configure but SPE=0
 *            └──────┬──────┘
 *                   ▼
 *            ┌─────────────┐
 *            │ GPIO setup  │ ← Configure pins
 *            └──────┬──────┘
 *                   ▼
 *        ┌────────────────────────┐
 *        │ SPI_PeripheralControl  │ ← Enable SPE=1
 *        │ (h, ENABLE)           │
 *        └──────────┬─────────────┘
 *                   ▼
 *            ┌─────────────┐
 *            │ Transfers   │ ← Send/Receive data
 *            └──────┬──────┘
 *                   ▼
 *        ┌────────────────────────┐
 *        │ SPI_PeripheralControl  │ ← Disable SPE=0 (optional)
 *        │ (h, DISABLE)          │
 *        └───────────────────────┘
 *
 * @param h       SPI handle with valid configuration
 * @param EnorDi  ENABLE to set SPE=1, DISABLE to clear SPE=0
 * @note          DISABLE operation may block waiting for current transfer completion
 * @note          Required after SPI_Init() to begin actual SPI communication
 * =============================================================================
 */
void SPI_PeripheralControl(SPI_Handle_t *h, uint8_t EnorDi)
{
	if (!h || !h->regs) return;

	if (EnorDi == ENABLE) {
		h->regs->CR1 |= SPI_CR1_SPE;
	} else {
		/* Wait for bus idle before disabling to avoid truncating transfers */
		while ((h->regs->SR & SPI_SR_BSY) != 0U) { /* spin */ }
		h->regs->CR1 &= ~SPI_CR1_SPE;
	}
	/* Ensure write completes */
	(void)h->regs->CR1;
}

/* =============================================================================
 * SPI IRQ Configuration and ISR registration (NVIC)
 * =============================================================================
 */

/**
 * @brief  Enable/Disable an NVIC IRQ line for SPI.
 * @param  IRQNumber  SPI1_IRQn, SPI2_IRQn, SPI3_IRQn.
 * @param  EnorDi     ENABLE or DISABLE.
 *
 * Notes:
 * - NVIC provides 32 IRQ enable bits per register. On STM32F407:
 *     * ISER0/ICER0 control IRQ  0..31
 *     * ISER1/ICER1 control IRQ 32..63
 *     * ISER2/ICER2 control IRQ 64..95
 * - Writing a '1' to an ISER bit enables that IRQ; writing a '1' to an ICER bit
 *   disables it. Other bits are unaffected.
 * - SPI IRQ numbers (from stm32f407xx.h):
 *     * SPI1_IRQn = 35
 *     * SPI2_IRQn = 36
 *     * SPI3_IRQn = 51
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

/**
 * @brief  Set NVIC priority for a SPI IRQ line.
 * @param  IRQNumber   Target IRQ line.
 * @param  IRQPriority Encoded priority (upper NO_PR_BITS_IMPLEMENTED bits used).
 *
 * Notes:
 * - On STM32F4, each IRQ has one byte in NVIC_IPR (0xE000E400 + IRQn).
 * - Only the upper NO_PR_BITS_IMPLEMENTED bits of that byte are implemented
 *   in hardware (on F4 this is 4 bits → values 0..15 after shifting).
 * - We left-shift the user value so it lands in the implemented top bits.
 * - Smaller numerical priority values mean *higher* preemption priority.
 * - This function only sets the priority; enabling/disabling is done via
 *   SPI_IRQInterruptConfig().
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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

/**
 * @brief  Start non-blocking transmit (enables TXEIE) with slave RXNE handling.
 * @param  h    SPI handle with initialized peripheral.
 * @param  pTx  Pointer to transmit buffer.
 * @param  len  Number of frames to transmit (bytes for 8-bit, words for 16-bit).
 * @return SPI_OK on success, SPI_ERR_BUSY if already transmitting, SPI_ERR_INVAL for invalid params.
 *
 * State Machine:
 *   ┌──────   SPI_SendDataIT()   ┌──────────
 *   │ IDLE │────────────────────→ │ TX_BUSY  │
 *   └──────┘                     └──────────┘
 *      ↑                              │
 *      │ ISR sets TX_DONE             │ ISR handles TXE
 *      │ clears TX_BUSY               │ sends frames
 *      │                              ▼
 *   ┌──────────     len == 0     ┌──────────
 *   │ TX_DONE  │◄─────────────────│ Sending  │
 *   └──────────┘                  └──────────┘
 *
 * Behavior:
 * - Master: enables TXEIE only (no implicit receive expected).
 * - Slave:  enables TXEIE + RXNEIE so that RX side is drained automatically
 *           (prevents RXNE from sticking and OVR faults when master clocks data).
 */
SPI_Status_t SPI_SendDataIT(SPI_Handle_t *h, const void *pTx, uint16_t len)
{
    /* Input validation */
    if (!h || !h->regs || !pTx || len == 0U) {
        return SPI_ERR_INVAL;
    }

    /* Enhanced busy check - check both driver flags and hardware */
    if ((h->flags & SPI_FLAG_TX_BUSY) || SPI_IsBusy(h)) {
        return SPI_ERR_BUSY;
    }

    /* Set up transfer state in handle */
    h->pTx = (const uint8_t*)pTx;     /* Store buffer pointer */
    h->txRemaining = len;             /* Store remaining frame count */

    /* Update flags: clear any previous completion flag, set busy */
    SPI_ClearFlags(h, SPI_FLAG_TX_DONE);
    h->flags |= SPI_FLAG_TX_BUSY;

    /* Always enable TXE interrupt */
    h->regs->CR2 |= SPI_CR2_TXEIE;

    /* NEW: If configured as slave (MSTR=0), also enable RXNEIE to drain RX */
    if ((h->regs->CR1 & SPI_CR1_MSTR) == 0U) {
        h->regs->CR2 |= SPI_CR2_RXNEIE;
    }

    return SPI_OK;
}


/**
 * @brief  Start non-blocking receive (enables RXNEIE).
 * @param  h    SPI handle with initialized peripheral.
 * @param  pRx  Pointer to receive buffer.
 * @param  len  Number of frames to receive (bytes for 8-bit, words for 16-bit).
 * @return SPI_OK on success, SPI_ERR_BUSY if already receiving, SPI_ERR_INVAL for invalid params.
 *
 * State Machine:
 *   ┌──────┐   SPI_ReceiveDataIT()   ┌──────────┐
 *   │ IDLE │─────────────────────→   │ RX_BUSY  │
 *   └──────┘                        └──────────┘
 *      ↑                                 │
 *      │ ISR sets RX_DONE                │ ISR handles RXNE
 *      │ clears RX_BUSY                  │ receives frames
 *      │                                 ▼
 *   ┌──────────┐      len == 0      ┌──────────┐
 *   │ RX_DONE  │◄────────────────── │ Receiving│
 *   └──────────┘                    └──────────┘
 *
 * Master vs Slave Behavior:
 *
 * MASTER MODE:
 * - Must generate clock by sending dummy data
 * - Enables both TXEIE (for clock) and RXNEIE (for data)
 * - ISR sends 0xFF dummy bytes while storing received data
 *
 * SLAVE MODE:
 * - Just waits for master's clock
 * - Only enables RXNEIE
 * - ISR only needs to read received data
 *
 * Operation:
 * 1. Validates inputs and checks if SPI is already busy with RX
 * 2. Sets up RX buffer pointer and remaining count in handle
 * 3. Sets SPI_FLAG_RX_BUSY flag and clears any previous RX_DONE
 * 4. Enables RXNEIE interrupt for data reception
 * 5. If master mode: also enables TXEIE for dummy clock generation
 * 6. ISR handles both TX (dummy) and RX (real data) until completion
 * 7. ISR sets RX_DONE flag and clears RX_BUSY when complete
 *
 * Usage Pattern:
 *   // Start non-blocking receive
 *   if (SPI_ReceiveDataIT(&spi_handle, rx_buffer, length) == SPI_OK) {
 *       // Transfer started successfully
 *
 *       // Poll for completion (or do other work)
 *       while (!(SPI_GetFlags(&spi_handle) & SPI_FLAG_RX_DONE)) {
 *           // Optional: do other work while transfer proceeds
 *       }
 *
 *       // Clear the completion flag for next transfer
 *       SPI_ClearFlags(&spi_handle, SPI_FLAG_RX_DONE);
 *   }
 *
 * Notes:
 * - Function returns immediately - transfer proceeds in background via ISR
 * - Master mode automatically sends dummy bytes (0xFF) for clock generation
 * - Only one RX transfer can be active at a time per SPI handle
 * - Compatible with all SPI modes: master/slave, full/half/simplex, 8/16-bit
 */
SPI_Status_t SPI_ReceiveDataIT(SPI_Handle_t *h, void *pRx, uint16_t len)
{
	/* Input validation */
	if (!h || !h->regs || !pRx || len == 0U) {
		return SPI_ERR_INVAL;
	}

	/* Enhanced busy check - check both driver flags and hardware */
	if ((h->flags & SPI_FLAG_RX_BUSY) || SPI_IsBusy(h)) {
		return SPI_ERR_BUSY;
	}

	/* Read configuration to determine master/slave mode */
	const uint8_t is_master = (uint8_t)((h->regs->CR1 & SPI_CR1_MSTR) ? 1U : 0U);

	/* Set up RX transfer state in handle */
	h->pRx = (uint8_t*)pRx;           /* Store RX buffer pointer */
	h->rxRemaining = len;             /* Store remaining frame count */

	/* Update flags: clear any previous completion flag, set busy */
	SPI_ClearFlags(h, SPI_FLAG_RX_DONE);
	h->flags |= SPI_FLAG_RX_BUSY;

	/* Enable RXNE interrupt for data reception */
	h->regs->CR2 |= SPI_CR2_RXNEIE;

	/* Master mode: also enable TXE interrupt for dummy clock generation */
	if (is_master) {
		/* Set up dummy TX state for clock generation */
		h->pTx = NULL;                /* NULL indicates dummy TX for clock */
		h->txRemaining = len;         /* Same count as RX for clock pulses */
		h->flags |= SPI_FLAG_TX_BUSY; /* Internal flag for ISR logic */

		/* Enable TXE interrupt to start sending dummy bytes */
		h->regs->CR2 |= SPI_CR2_TXEIE;
	}

	return SPI_OK;
}

/* =============================================================================
 * ASCII ART DIAGRAMS FOR SPI IRQ HANDLING FUNCTIONS
 * =============================================================================
 */

/**
 * ERROR IRQ HANDLER - spi_handle_error_irq()
 *
 * Priority-Based Error Processing:
 *
 *    ┌─────────────────────────────────────────┐
 *    │         ERROR INTERRUPT                 │
 *    │    (ERRIE=1 in CR2 enables)            │
 *    └──────────────┬──────────────────────────┘
 *                   │
 *                   ▼
 *    ┌─────────────────────────────────────────┐
 *    │         Check SR Flags                  │
 *    │   OVR | MODF | CRCERR | FRE            │
 *    └──────┬──────┬──────┬──────┬─────────────┘
 *           │      │      │      │
 *           ▼      ▼      ▼      ▼
 *    ┌──────────┐ ┌─────┐ ┌─────┐ ┌─────┐
 *    │   OVR    │ │MODF │ │ CRC │ │ FRE │
 *    │CRITICAL  │ │CRIT │ │ LOG │ │ LOG │
 *    │   ABORT  │ │ABORT│ │CONT │ │CONT │
 *    └─────┬────┘ └──┬──┘ └──┬──┘ └──┬──┘
 *          │         │       │       │
 *          ▼         ▼       ▼       ▼
 *    ┌─────────────────────────────────────────┐
 *    │        Clean Up Transfers               │
 *    │  • Disable TXEIE/RXNEIE                │
 *    │  • Clear TX_BUSY/RX_BUSY flags         │
 *    │  • Reset buffer pointers               │
 *    │  • Set error flag                      │
 *    └─────────────────────────────────────────┘
 *                   │
 *                   ▼
 *    ┌─────────────────────────────────────────┐
 *    │     Return Error Status                 │
 *    │  1 = Critical (abort transfer)         │
 *    │  0 = Continue processing                │
 *    └─────────────────────────────────────────┘
 */

/**
 * @brief  Handle SPI error interrupts (OVR, MODF, CRCERR, FRE).
 * @param  h   SPI handle.
 * @param  sr  Status register snapshot.
 * @param  cr2 Control register 2 snapshot.
 * @return 1 if critical error occurred (transfer should abort), 0 if OK to continue.
 */
static uint8_t spi_handle_error_irq(SPI_Handle_t *h, uint32_t sr, uint32_t cr2)
{
	SPI_RegDef_t *r = h->regs;
	uint8_t critical_error = 0U;

	/* Only process if error interrupts are enabled */
	if (!(cr2 & SPI_CR2_ERRIE)) {
		return 0U;
	}

	/* CRITICAL: Overrun Error - abort transfers */
	if (sr & SPI_SR_OVR) {
		/* Use utility function for proper OVR clearing */
		SPI_ClearOVR(h);

		/* Abort ongoing transfers */
		if (h->flags & SPI_FLAG_TX_BUSY) {
			r->CR2 &= ~SPI_CR2_TXEIE;           /* Disable TXE interrupt */
			h->flags &= ~SPI_FLAG_TX_BUSY;      /* Clear busy flag */
			h->pTx = NULL;                      /* Clear buffer pointer */
			h->txRemaining = 0U;                /* Clear remaining count */
		}

		if (h->flags & SPI_FLAG_RX_BUSY) {
			r->CR2 &= ~SPI_CR2_RXNEIE;          /* Disable RXNE interrupt */
			h->flags &= ~SPI_FLAG_RX_BUSY;      /* Clear busy flag */
			h->pRx = NULL;                      /* Clear buffer pointer */
			h->rxRemaining = 0U;                /* Clear remaining count */
		}

		h->flags |= SPI_FLAG_OVR;               /* Set error flag */
		critical_error = 1U;
	}

	/* CRITICAL: Mode Fault - NSS pulled low in master mode */
	if (sr & SPI_SR_MODF) {
		/* Clear MODF by reading SR then writing CR1 (per reference manual) */
		(void)r->SR;
		r->CR1 = r->CR1;  /* Dummy write to CR1 clears MODF */

		/* Abort ongoing transfers */
		if (h->flags & SPI_FLAG_TX_BUSY) {
			r->CR2 &= ~SPI_CR2_TXEIE;
			h->flags &= ~SPI_FLAG_TX_BUSY;
			h->pTx = NULL;
			h->txRemaining = 0U;
		}

		if (h->flags & SPI_FLAG_RX_BUSY) {
			r->CR2 &= ~SPI_CR2_RXNEIE;
			h->flags &= ~SPI_FLAG_RX_BUSY;
			h->pRx = NULL;
			h->rxRemaining = 0U;
		}

		h->flags |= SPI_FLAG_MODF;
		critical_error = 1U;
	}

	/* NON-CRITICAL: CRC Error - log but continue */
	if (sr & SPI_SR_CRCERR) {
		/* Clear by writing 0 to CRCERR bit */
		r->SR &= ~SPI_SR_CRCERR;
		h->flags |= SPI_FLAG_CRCERR;
		/* Continue with transfer */
	}

	/* NON-CRITICAL: Frame Error (TI mode) - log but continue */
	if (sr & SPI_SR_FRE) {
		/* FRE clears automatically when SR is read */
		h->flags |= SPI_FLAG_FRE;
		/* Continue with transfer */
	}

	return critical_error;
}

/**
 * TX IRQ HANDLER - spi_handle_tx_irq()
 *
 * Transmit Buffer Empty Handling:
 *
 *    ┌─────────────────────────────────────────┐
 *    │           TXE INTERRUPT                 │
 *    │    (TXEIE=1 enables, TXE=1 fires)     │
 *    └──────────────┬──────────────────────────┘
 *                   │
 *                   ▼
 *    ┌─────────────────────────────────────────┐
 *    │         Validate Conditions             │
 *    │   TXE=1? TXEIE=1? TX_BUSY=1?           │
 *    │       txRemaining > 0?                  │
 *    └──────────────┬──────────────────────────┘
 *                   │ All OK
 *                   ▼
 *    ┌─────────────────────────────────────────┐
 *    │        Determine Frame Source           │
 *    └─────┬─────────────────────────┬─────────┘
 *          │                         │
 *     pTx != NULL              pTx == NULL
 *          │                         │
 *          ▼                         ▼
 *    ┌──────────┐              ┌──────────┐
 *    │ Real TX  │              │ Dummy TX │
 *    │ Data     │              │ (Master  │
 *    │spi_pop() │              │ RX Mode) │
 *    └─────┬────┘              └─────┬────┘
 *          │                         │
 *          └─────────┬─────────────────┘
 *                    │ frame_to_send
 *                    ▼
 *    ┌─────────────────────────────────────────┐
 *    │           Send Frame                    │
 *    │         DR ← frame_to_send              │
 *    │       txRemaining--                     │
 *    └──────────────┬──────────────────────────┘
 *                   │
 *                   ▼
 *    ┌─────────────────────────────────────────┐
 *    │      Check if Transfer Complete         │
 *    │        txRemaining == 0?                │
 *    └─────┬─────────────────────────┬─────────┘
 *          │ No                      │ Yes
 *          ▼                         ▼
 *    ┌──────────┐              ┌──────────┐
 *    │ Continue │              │ Complete │
 *    │Transfer  │              │Transfer  │
 *    │ (Wait    │              │• Disable │
 *    │  Next    │              │  TXEIE   │
 *    │  TXE)    │              │• Set Flags│
 *    └──────────┘              └──────────┘
 */

/**
 * @brief  Handle SPI TX interrupt (TXE).
 * @param  h        SPI handle.
 * @param  sr       Status register snapshot.
 * @param  cr2      Control register 2 snapshot.
 * @param  is_16bit 1 if 16-bit mode, 0 if 8-bit mode.
 */
static void spi_handle_tx_irq(SPI_Handle_t *h, uint32_t sr, uint32_t cr2, uint8_t is_16bit)
{
	SPI_RegDef_t *r = h->regs;

	/* Check if TXE interrupt is active and enabled */
	if (!(sr & SPI_SR_TXE) || !(cr2 & SPI_CR2_TXEIE)) {
		return;
	}

	/* Check if TX transfer is active and has remaining data */
	if (!(h->flags & SPI_FLAG_TX_BUSY) || h->txRemaining == 0U) {
		return;
	}

	uint16_t frame_to_send;

	if (h->pTx != NULL) {
		/* Real TX data - normal transmission */
		frame_to_send = spi_pop_frame(&h->pTx, is_16bit);
	} else {
		/* Dummy TX for master RX clock generation */
		frame_to_send = 0xFF; /* Standard dummy byte */
	}

	/* Send the frame */
	r->DR = (uint32_t)frame_to_send;
	h->txRemaining--;

	/* Check if TX transmission complete */
	if (h->txRemaining == 0U) {
		/* Disable TXE interrupt */
		r->CR2 &= ~SPI_CR2_TXEIE;

		/* Set completion flags - but only for real TX transfers */
		if (h->pTx != NULL) {
			/* Real TX transfer completed */
			h->flags &= ~SPI_FLAG_TX_BUSY;
			h->flags |= SPI_FLAG_TX_DONE;
			h->pTx = NULL;
		} else {
			/* Dummy TX for RX clock completed - just clear busy */
			h->flags &= ~SPI_FLAG_TX_BUSY;
			/* Don't set TX_DONE flag for dummy TX */
		}
	}
}

/**
 * RX IRQ HANDLER - spi_handle_rx_irq()
 *
 * Receive Buffer Not Empty Handling:
 *
 *    ┌─────────────────────────────────────────┐
 *    │          RXNE INTERRUPT                 │
 *    │   (RXNEIE=1 enables, RXNE=1 fires)    │
 *    └──────────────┬──────────────────────────┘
 *                   │
 *                   ▼
 *    ┌─────────────────────────────────────────┐
 *    │         Validate Conditions             │
 *    │  RXNE=1? RXNEIE=1? RX_BUSY=1?          │
 *    │       rxRemaining > 0?                  │
 *    └──────────────┬──────────────────────────┘
 *                   │ All OK
 *                   ▼
 *    ┌─────────────────────────────────────────┐
 *    │         Read Received Frame             │
 *    │       received_frame ← DR               │
 *    └──────────────┬──────────────────────────┘
 *                   │
 *                   ▼
 *    ┌─────────────────────────────────────────┐
 *    │        Store in RX Buffer               │
 *    │      spi_push_frame(pRx, frame)        │
 *    │         rxRemaining--                   │
 *    └──────────────┬──────────────────────────┘
 *                   │
 *                   ▼
 *    ┌─────────────────────────────────────────┐
 *    │      Check if Transfer Complete         │
 *    │        rxRemaining == 0?                │
 *    └─────┬─────────────────────────┬─────────┘
 *          │ No                      │ Yes
 *          ▼                         ▼
 *    ┌──────────┐              ┌──────────┐
 *    │ Continue │              │ Complete │
 *    │Transfer  │              │Transfer  │
 *    │ (Wait    │              │• Disable │
 *    │  Next    │              │  RXNEIE  │
 *    │  RXNE)   │              │• Set Flags│
 *    └──────────┘              └──────────┘
 *
 * NOTE: Updated behavior
 * - If RXNE triggers but no RX transfer is active, the handler now discards
 *   the DR contents to clear RXNE. This prevents OVR faults when the slave is
 *   only transmitting and not receiving meaningful data.
 */
static void spi_handle_rx_irq(SPI_Handle_t *h, uint32_t sr, uint32_t cr2, uint8_t is_16bit)
{
    SPI_RegDef_t *r = h->regs;

    /* Check if RXNE interrupt is active and enabled */
    if (!(sr & SPI_SR_RXNE) || !(cr2 & SPI_CR2_RXNEIE)) {
        return;
    }

    /* Read received frame from data register */
    uint16_t received_frame = (uint16_t)r->DR;

    if ((h->flags & SPI_FLAG_RX_BUSY) && h->rxRemaining > 0U) {
        /* Store in RX buffer */
        spi_push_frame(&h->pRx, received_frame, is_16bit);
        h->rxRemaining--;

        /* Check if RX reception complete */
        if (h->rxRemaining == 0U) {
            /* Disable RXNE interrupt */
            r->CR2 &= ~SPI_CR2_RXNEIE;

            /* Set completion flags */
            h->flags &= ~SPI_FLAG_RX_BUSY;
            h->flags |= SPI_FLAG_RX_DONE;
            h->pRx = NULL;
        }
    } else {
        /* NEW: No active receive — discard to clear RXNE & prevent OVR */
        (void)received_frame;
    }
}

/**
 * MAIN IRQ HANDLER - SPI_IRQHandling()
 *
 * Orchestrator Function:
 *
 *    ┌─────────────────────────────────────────┐
 *    │        SPI_IRQHandling()                │
 *    │     Called from SPIx_IRQHandler()      │
 *    └──────────────┬──────────────────────────┘
 *                   │
 *                   ▼
 *    ┌─────────────────────────────────────────┐
 *    │        Take Register Snapshots          │
 *    │      sr = SR, cr2 = CR2, is_16bit      │
 *    └──────────────┬──────────────────────────┘
 *                   │
 *                   ▼
 *    ┌─────────────────────────────────────────┐
 *    │       Handle Errors First               │
 *    │    spi_handle_error_irq(h,sr,cr2)      │
 *    └─────┬─────────────────────────┬─────────┘
 *          │ Critical Error          │ OK
 *          ▼                         ▼
 *    ┌──────────┐              ┌──────────┐
 *    │  ABORT   │              │ Continue │
 *    │ Transfer │              │Processing│
 *    │  EXIT    │              │          │
 *    └──────────┘              └─────┬────┘
 *                                    │
 *                                    ▼
 *                   ┌─────────────────────────────────┐
 *                   │        Handle TX IRQ            │
 *                   │   spi_handle_tx_irq(...)       │
 *                   └─────────────┬───────────────────┘
 *                                 │
 *                                 ▼
 *                   ┌─────────────────────────────────┐
 *                   │        Handle RX IRQ            │
 *                   │   spi_handle_rx_irq(...)       │
 *                   └─────────────────────────────────┘
 *
 * OVERALL DATA FLOW - Complete Picture
 *
 * Application to Hardware Flow:
 *
 *    ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
 *    │Application  │────▶│ SPI Driver  │────▶│ Hardware    │
 *    │             │     │             │     │             │
 *    │SPI_SendIT() │     │Enable TXEIE │     │Generate TXE │
 *    │SPI_RxIT()   │     │Enable RXNEIE│     │Generate RXNE│
 *    └─────────────┘     └─────────────┘     └─────┬───────┘
 *                                                  │
 *    ┌─────────────┐     ┌─────────────┐           │
 *    │Application  │◄────│    NVIC     │◄──────────┘
 *    │             │     │             │
 *    │SPIx_IRQ     │     │Route to CPU │
 *    │Handler()    │     │             │
 *    └─────┬───────┘     └─────────────┘
 *          │
 *          ▼
 *    ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
 *    │SPI_IRQ      │────▶│   Modular   │────▶│  Complete   │
 *    │Handling()   │     │  Handlers   │     │  Transfer   │
 *    │             │     │             │     │             │
 *    │Orchestrator │     │TX/RX/Error  │     │Set FLAGS    │
 *    └─────────────┘     └─────────────┘     └─────────────┘
 *                                                  │
 *    ┌─────────────┐                               │
 *    │Application  │◄──────────────────────────────┘
 *    │Poll FLAGS   │
 *    │TX_DONE      │
 *    │RX_DONE      │
 *    └─────────────┘
 */

/**
 * @brief  SPI ISR helper: handles TXE, RXNE, and ERR interrupts.
 * @param  h  SPI handle with active interrupt transfer state.
 *
 * Modular Design:
 * ┌─────────────────┐
 * │ SPI_IRQHandling │
 * └─────────┬───────┘
 *           │
 *     ┌─────┴─────┬─────────────┬─────────────┐
 *     │           │             │             │
 * ┌───▼───┐ ┌─────▼─────┐ ┌─────▼─────┐ ┌─────▼─────┐
 * │ ERROR │ │ TX (TXE)  │ │ RX (RXNE) │ │   ...     │
 * │ IRQ   │ │ Handler   │ │ Handler   │ │  Future   │
 * └───────┘ └───────────┘ └───────────┘ └───────────┘
 *
 * Benefits:
 * - Each interrupt type has dedicated function
 * - Easier testing and debugging
 * - Cleaner code organization
 * - Reusable components
 *
 * Usage:
 *   void SPI1_IRQHandler(void) {
 *       SPI_IRQHandling(&spi1_handle);
 *   }
 */
void SPI_IRQHandling(SPI_Handle_t *h)
{
	if (!h || !h->regs) return;

	SPI_RegDef_t *r = h->regs;
	const uint32_t sr = r->SR;    /* Snapshot status register */
	const uint32_t cr2 = r->CR2;  /* Snapshot control register */
	const uint8_t is_16bit = (uint8_t)((r->CR1 & SPI_CR1_DFF) ? 1U : 0U);

	/* Handle error interrupts first - may abort transfer */
	if (spi_handle_error_irq(h, sr, cr2)) {
		/* Critical error occurred - exit early */
		return;
	}

	/* Handle TX interrupt */
	spi_handle_tx_irq(h, sr, cr2, is_16bit);

	/* Handle RX interrupt */
	spi_handle_rx_irq(h, sr, cr2, is_16bit);
}
