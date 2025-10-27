#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/* ===== Basic types ======================================================== */
typedef enum {
	GPIO_PORT_A = 0U,
	GPIO_PORT_B,
	GPIO_PORT_C,
	GPIO_PORT_D,
	GPIO_PORT_E,
	GPIO_PORT_F,
	GPIO_PORT_G,
	GPIO_PORT_H,
	GPIO_PORT_I
} GPIO_PortId_t;

typedef uint8_t GPIO_PinNum_t;

typedef enum {
	GPIO_MODE_INPUT = 0U,
	GPIO_MODE_OUTPUT = 1U,
	GPIO_MODE_AF = 2U,
	GPIO_MODE_ANALOG = 3U
} GPIO_Mode_t;

typedef enum {
	GPIO_OTYPE_PP = 0U, GPIO_OTYPE_OD = 1U
} GPIO_OType_t;

typedef enum {
	GPIO_NOPULL = 0U, GPIO_PULLUP = 1U, GPIO_PULLDOWN = 2U
} GPIO_Pull_t;

typedef enum {
	GPIO_SPEED_LOW = 0U,
	GPIO_SPEED_MEDIUM = 1U,
	GPIO_SPEED_FAST = 2U,
	GPIO_SPEED_HIGH = 3U
} GPIO_Speed_t;

typedef enum {
	GPIO_EXTI_TRIGGER_RISING = 0U,
	GPIO_EXTI_TRIGGER_FALLING = 1U,
	GPIO_EXTI_TRIGGER_BOTH = 2U
} GPIO_ExtiTrigger_t;

typedef enum {
	GPIO_OK = 0, GPIO_ERR_INVAL, GPIO_ERR_UNSUPPORTED, GPIO_ERR_BUSY
} GPIO_Status_t;

/** GPIO pin configuration (no initial output level here). */
typedef struct {
    GPIO_PortId_t port;   /* A..I (0..8) */
    GPIO_PinNum_t pin;    /* 0..15 */
    GPIO_Mode_t   mode;   /* MODER: INPUT/OUTPUT/AF/ANALOG */
    GPIO_OType_t  otype;  /* OTYPER: PP/OD */
    GPIO_Pull_t   pull;   /* PUPDR: NOPULL/PULLUP/PULLDOWN */
    GPIO_Speed_t  speed;  /* OSPEEDR: LOW/MEDIUM/FAST/HIGH */
    uint8_t       af;     /* AFR[0..15] when mode==AF */
} GPIO_PinConfig_t;  /* keeps your original enum-based design */

/** GPIO pin handle — caches resolved port pointer & bit mask for fast I/O. */
typedef struct {
    GPIO_RegDef_t   *port;   /* Resolved GPIOx pointer (from config.port) */
    uint16_t         mask;   /* (1U << config.pin) cached for fast BSRR */
    GPIO_PinConfig_t config; /* Full configuration using your enums */
} GPIO_PinHandle_t;


/* =============================================================================
 * Peripheral Clock setup
 * =============================================================================
 */

/**
 * @brief  Enable/Disable AHB1 clock for a GPIO port.
 * @param  pGPIOx   Target port register block (GPIOA..GPIOI).
 * @param  EnorDi   ENABLE or DISABLE (see stm32f407xx.h).
 * @note   Uses RCC->AHB1ENR bit corresponding to the port.
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* =============================================================================
 * Init and De-init
 * =============================================================================
 */

/**
 * @brief  Initialize a GPIO pin as per @p pGPIOHandle->GPIO_PinConfig on
 *         the port @p pGPIOHandle->pGPIOx.
 * @param  pGPIOHandle  Handle containing target port and pin configuration.
 * @note   No “initial level” is set here. If you need glitch-free bring-up
 *         (e.g., CS#/RESET#), preload ODR via BSRR before switching to Output.
 */
void GPIO_Init(GPIO_PinHandle_t *pGPIOHandle);

/**
 * @brief  De-initialize a GPIO port to its reset state.
 *         Typical implementation pulses RCC->AHB1RSTR for the given port,
 *         which resets ALL pins on that port.
 * @param  pGPIOx  Target port base (GPIOA..GPIOI).
 * @note   If you need to de-init only a single pin (not the whole port),
 *         implement a pin-scope helper in your .c file (not part of this API).
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* =============================================================================
 * Data read and write
 * =============================================================================
 */

/**
 * @brief  Read logic level from a single input pin (IDR).
 * @param  pGPIOx     Port base.
 * @param  PinNumber  0..15.
 * @return GPIO_PIN_SET(1) if high; GPIO_PIN_RESET(0) if low.
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * @brief  Read entire port input data register (IDR).
 * @param  pGPIOx  Port base.
 * @return Lower 16 bits reflect pin states.
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/**
 * @brief  Write logic level to a single output pin using atomic BSRR.
 * @param  pGPIOx     Port base.
 * @param  PinNumber  0..15.
 * @param  Value      GPIO_PIN_SET(1) or GPIO_PIN_RESET(0).
 * @note   Uses BSRR (set/reset) to avoid read-modify-write hazards.
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);

/**
 * @brief  Write an entire 16-bit value to the port output data register (ODR).
 * @param  pGPIOx  Port base.
 * @param  Value   Lower 16 bits driven to ODR.
 * @warning This is a non-atomic write to ODR; concurrent pin updates should
 *          use per-pin BSRR writes to avoid contention.
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

/**
 * @brief  Toggle a single output pin.
 * @param  pGPIOx     Port base.
 * @param  PinNumber  0..15.
 * @note   Implementation may read ODR and emit the opposite via BSRR.
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* =============================================================================
 * IRQ Configuration and ISR handling
 * =============================================================================
 */

/* =============================================================================
 * EXTI Configuration
 * =============================================================================
 */

/**
 * @brief  Configure an external interrupt (EXTI) line for a given GPIO pin.
 *         Maps the pin’s port to the EXTI line via SYSCFG, programs rising/falling
 *         edge triggers, and unmasks/masks the line in EXTI->IMR.
 *
 * @param  h        GPIO pin handle (must have valid .config.port and .config.pin).
 * @param  trigger  Trigger selection:
 *                  - GPIO_EXTI_TRIGGER_RISING
 *                  - GPIO_EXTI_TRIGGER_FALLING
 *                  - GPIO_EXTI_TRIGGER_BOTH
 * @param  enable   ENABLE to unmask the interrupt, DISABLE to mask it.
 *
 * @note   - This function enables the SYSCFG APB2 clock internally before writing
 *           to EXTICR registers.
 *         - Clears any stale pending bit in EXTI->PR to avoid spurious IRQ.
 *         - NVIC configuration (priority, enable) must be done separately using
 *           GPIO_IRQInterruptConfig() / GPIO_IRQPriorityConfig().
 */
void GPIO_ConfigEXTI(const GPIO_PinHandle_t *h,
                     GPIO_ExtiTrigger_t trigger,
                     uint8_t enable);

/**
 * @brief  Enable/Disable an NVIC IRQ line.
 * @param  IRQNumber  Target IRQ line number (e.g., EXTI0, EXTI9_5, etc.).
 * @param  EnorDi     ENABLE or DISABLE.
 * @note   This configures the NVIC only. To route a GPIO to an EXTI line and
 *         set edge triggers, also program SYSCFG->EXTICR and EXTI->RTSR/FTSR.
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief  Set NVIC priority for an IRQ line.
 * @param  IRQNumber   Target IRQ line number.
 * @param  IRQPriority Priority value (width depends on MCU/port config).
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief  GPIO EXTI line ISR helper: clears EXTI->PR for @p PinNumber (line N).
 * @param  PinNumber  Pin / EXTI line number 0..15.
 * @note   Call this from within your EXTIx IRQHandler to clear the pending bit.
 */
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
