/*
 * STM32F446xx_GPIO_driver.c
 *
 *      Author: AYOUB
 */
#include "STM32F446xx_GPIO_driver.h"

/*
 * peripheral clock setup
 */

/********************************************************
 * @ fn 			- GPIO_PeriClkControl
 *
 * @brief			- This function is responsible for Enabling or Disabling peripheral clock for a given GPIO port
 *
 * @param[i]		- Base address of GPIO Port
 * @param[i]		- ENABLE or DISABLE macro
 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIO, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		if(pGPIO == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIO == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIO == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIO == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIO == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIO == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIO == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIO == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else if(ENorDI == DISABLE)
	{
		if(pGPIO == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIO == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIO == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIO == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIO == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIO == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIO == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIO == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */

/********************************************************
 * @ fn 			- GPIO_INIT
 *
 * @brief			- This function initialize the GPIO Port and GPIO pin and set its configuration
 *
 * @param[i]		- Base address of GPIO Handle variable

 *
 * @return 			- none
 *
 * @note 			- none
 *
 *******************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandlex)
{
	//enable the clock for the GPIO peripheral
	GPIO_PeriClkControl(pGPIOHandlex->pGPIOx , ENABLE);

	uint32_t temp; //temp register

	// 1. configure the mode of a GPIO Pin
	if (pGPIOHandlex->GpioPinConfig.PinMode <= GPIO_MODE_ANALOG ){
		//the Non Interrupt Mode
		temp = pGPIOHandlex->GpioPinConfig.PinMode << (2 * pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);
		pGPIOHandlex->pGPIOx->MODER &= ~(0x3 << pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);// Clearing
		pGPIOHandlex->pGPIOx->MODER |= temp; // Setting
		temp = 0;
	} else {
		// the interrupt mode
		if(pGPIOHandlex->GpioPinConfig.PinMode == GPIO_MODE_FT){
			//1. configure FTSR
			EXTI->FTSR = (1 << pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);
			//clearing the corresponding RTSR bit
			EXTI->RTSR = ~(1 << pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandlex->GpioPinConfig.PinMode == GPIO_MODE_RT){
			//1. configure RTSR
			EXTI->RTSR = (1 << pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);
			//clearing the corresponding RTSR bit
			EXTI->FTSR = ~(1 << pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandlex->GpioPinConfig.PinMode == GPIO_MODE_RFT){
			//1. configure FTSR and RTSR
			EXTI->RTSR = (1 << pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);
			EXTI->FTSR = (1 << pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);

		}

		//2. configure the PORT selection in SYSCFG_EXTICR
		uint8_t temp1 = (pGPIOHandlex->GpioPinConfig.GPIO_PinNumber) / 4; // dividing the GPIO PIN NUMBER by the number of bits of each EXTIx configuration
		uint8_t temp2 = (pGPIOHandlex->GpioPinConfig.GPIO_PinNumber) % 4; // the rest of the division indicate how many cases we have to shift from base EXTIx;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandlex->pGPIOx);
		SYSCFG->EXTICR[temp1] = (portcode << temp2);

		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);
	}
	temp = 0;

	// 2. configure the Speed

	temp = pGPIOHandlex->GpioPinConfig.PinSpeed << (2 * pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);
	pGPIOHandlex->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);// Clearing
	pGPIOHandlex->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. configure the Pull-up Pull-down register

	temp = pGPIOHandlex->GpioPinConfig.Pin_PuPdControl << (2 * pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);
	pGPIOHandlex->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);// Clearing
	pGPIOHandlex->pGPIOx->PUPDR |= temp;//Setting
	temp = 0;

	// 4. configure the output type

	temp = pGPIOHandlex->GpioPinConfig.GPIO_PinOPtype << ( pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);
	pGPIOHandlex->pGPIOx->OTYPER &= ~(1 << pGPIOHandlex->GpioPinConfig.GPIO_PinNumber);// Clearing
	pGPIOHandlex->pGPIOx->OTYPER |= temp;//Setting
	temp = 0;

	// 5. configure the alternate functionality

	if (pGPIOHandlex->GpioPinConfig.PinMode == GPIO_MODE_ALTFN) {
		// configure the alternative function
		uint8_t temp1, temp2;
		temp1 = (pGPIOHandlex->GpioPinConfig.GPIO_PinNumber) / AFR_PINS;
		temp2 = (pGPIOHandlex->GpioPinConfig.GPIO_PinNumber) % AFR_PINS;
		pGPIOHandlex->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));// Clearing
		pGPIOHandlex->pGPIOx->AFR[temp1] |= pGPIOHandlex->GpioPinConfig.GPIO_Pin_AltFunMode << (4 * temp2);//Setting
	}

}

void GPIO_Dinit(GPIO_RegDef_t *pGPIO){
	if(pGPIO == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIO == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIO == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIO == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIO == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIO == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIO == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIO == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Data read and write
 */

uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> pinNumber) & 0x00000001;
	return value;
}

/*
 *
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value){
	if(value == GPIO_PIN_SET){
		//Set the bit corresponding to the Pin Number
		pGPIOx->ODR |= (1 << pinNumber);
	}
	else if (value == GPIO_PIN_RESET){
		//Clear the bit corresponding to the Pin Number
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;//Write value in the output data register
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	pGPIOx->ODR ^= (1 << pinNumber);
}


/*
 * IRQ configuration and ISR handling
 */

void GPIO_IrqInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		if (IRQNumber <= 31) {
			//set ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) {
			//set ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber < 96) {
			//set ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else if (ENorDI == DISABLE)
	{
		if (IRQNumber <= 31) {
			//set ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) {
			//set ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96) {
			//set ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

void GPIO_IRQ_PriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NB_PR_BIT_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) = (IRQPriority << shift_amount);
}

void GPIO_IrqHandling(uint8_t PinNumber){
	//Clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber)) {
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}









