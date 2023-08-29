/*
 * STM32F446xx_GPIO_driver.h
 *
 *      Author: AYOUB
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "STM32F446xx.h"

/*
 * This is a Configuration structure for a GPIO pin
 */

typedef struct {
	uint8_t GPIO_PinNumber; /* Possible Values from @GPIO Pin Number */
	uint8_t PinMode; /* Possible Values from @GPIO Pin Mode */
	uint8_t PinSpeed; /* Possible Values from @GPIO Output Speed */
	uint8_t Pin_PuPdControl; /* Possible Values from @GPIO PUPD */
	uint8_t GPIO_PinOPtype; /* Possible Values from @GPIO Output Type  */
	uint8_t GPIO_Pin_AltFunMode;

}GPIO_PinConfig_t;

/*
 * This is a handle structure of a GPIO pin
 */

typedef struct {
	GPIO_RegDef_t * pGPIOx ; /*This hold the base address of the GPIO port to which the Pin belong */
	GPIO_PinConfig_t GpioPinConfig; /*This holds Pin Configuration Setting*/

}GPIO_Handle_t;

/*
 * @GPIO Pin Number
 * GPIO Possible Pin Numbers
 */

#define GPIO_PIN_NO_0	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15

/*
 * @GPIO Pin Mode
 * GPIO Pin Possible Modes
 */

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_FT		4
#define GPIO_MODE_RT		5
#define GPIO_MODE_RFT		6

/*
 * @GPIO Output Type
 * GPIO Pin Possible Output Type
 */

#define GPIO_OP_Type_PP		0
#define GPIO_OP_Type_OD		1

/* @GPIO Output Speed
 * GPIO Pin Possible Output Speed
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO PUPD
 * GPIO Pin Pull-Up and Pull-Down configuration macro
 */

#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2
#define GPIO_RESERVED		3


/*********************************************************************************************************************************
 * 												APIs SUPPORTED BY THIS DRIVER													 *
 * 								 For More information about the APIs check the function definition							     *
 *********************************************************************************************************************************/

/*
 * peripheral clock setup
 */
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);

/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandlex);
void GPIO_Dinit(GPIO_RegDef_t *pGPIO);

/*
 * Data read and write
 */

uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/*
 * IRQ configuration and ISR handling
 */

void GPIO_IrqInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void GPIO_IRQ_PriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IrqHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
