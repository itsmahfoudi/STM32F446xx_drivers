/*
 * STM32F446xx_RCC_driver.h
 *
 *  Created on: Aug 29, 2023
 *      Author: ayoub
 */

#ifndef DRIVERS_INC_STM32F446XX_RCC_DRIVER_H_
#define DRIVERS_INC_STM32F446XX_RCC_DRIVER_H_

#include "STM32F446xx.h"

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLL_ClkValue();

#endif /* DRIVERS_INC_STM32F446XX_RCC_DRIVER_H_ */
