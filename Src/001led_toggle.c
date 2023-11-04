/*
 * 001led_toggle.c
 *
 *  Created on: 1 mars 2023
 *      Author: AYOUB
 */

#include "./../Inc/STM32F446xx.h"
#include "./../Inc/STM32F446xx_GPIO_driver.h"

void delay(void){
	for(uint32_t i = 0 ; i < 500000 ; i++);
}

int main(void) {

	GPIO_Handle_t GPIOled;

  //memset(&GPIOled,0,sizeof(GPIOled));
	GPIOled.pGPIOx = GPIOD;
	GPIOled.GpioPinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOled.GpioPinConfig.PinMode = GPIO_MODE_OUT;
	GPIOled.GpioPinConfig.PinSpeed = GPIO_SPEED_FAST;
	GPIOled.GpioPinConfig.GPIO_PinOPtype = GPIO_OP_Type_PP;
	GPIOled.GpioPinConfig.Pin_PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClkControl(GPIOD, ENABLE);
	GPIO_Init(&GPIOled);

	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}
	GPIO_RegDef_t *pGPIOX;
	pGPIOX->LCKR;
	return 0;
}

