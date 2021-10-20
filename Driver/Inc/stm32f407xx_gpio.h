/*
 * stm32f407xx_gpio.h
 *
 *  Created on: Oct 13, 2021
 *      Author: caoth
 */

#ifndef STM32F407XX_GPIO_H_
#define STM32F407XX_GPIO_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPd;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig;


typedef struct
{
	GPIO_RegDef *pGPIOx;
	GPIO_PinConfig GPIO_PinConf;
}GPIO_Handle;


//PIN NUMBER
#define GPIO_PIN_0  		0
#define GPIO_PIN_1  		1
#define GPIO_PIN_2  		2
#define GPIO_PIN_3  		3
#define GPIO_PIN_4  		4
#define GPIO_PIN_5  		5
#define GPIO_PIN_6  		6
#define GPIO_PIN_7  		7
#define GPIO_PIN_8  		8
#define GPIO_PIN_9  		9
#define GPIO_PIN_10  		10
#define GPIO_PIN_11  		11
#define GPIO_PIN_12  		12
#define GPIO_PIN_13  		13
#define GPIO_PIN_14  		14
#define GPIO_PIN_15  		15
//MODE
#define GPIO_MODE_IN  		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALF    	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT  	4
#define GPIO_MODE_IT_RT  	5
#define GPIO_MODE_IT_RFT 	6
//OUTPUT TYPE
#define GPIO_OUT_TYPE_PP  	0
#define GPIO_OUT_TYPE_OD  	1
//OUTPUT SPEED
#define GPIO_SPEED_LOW   	0
#define GPIO_SPEED_MED   	1
#define GPIO_SPEED_FAST  	2
#define GPIO_SPEED_HIGH 	3
//PULL UP, PULL DOWN
#define GPIO_NO_PUPD  		0
#define GPIO_PU  			1
#define GPIO_PD  			2

//Init, Deinit
void GPIO_Init(GPIO_Handle *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef *pGPIOx);

//clock control
void GPIO_PeriphClkControl(GPIO_RegDef *pGPIOx,uint8_t ENorDI);

//data read, write
uint8_t  GPIO_ReadInputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber, uint8_t State);
void GPIO_WriteOutputPort(GPIO_RegDef *pGPIOx, uint8_t State);
void GPIO_ToggleOutputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber);

//Interupt
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);
void GPIO_IRQHandle(uint8_t PinNumber);

#endif /* STM32F407XX_GPIO_H_ */
