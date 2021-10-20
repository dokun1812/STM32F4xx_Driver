/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Oct 13, 2021
 *      Author: NhatHoang
 */
#include "stm32f407xx_gpio.h"
void GPIO_PeriphClkControl(GPIO_RegDef *pGPIOx, uint8_t ENorDI){
	//ENABLE
	if(ENorDI==ENABLE){
		if(pGPIOx==GPIOA){
			GPIOA_PERIPH_CLK_EN();
		}else if(pGPIOx==GPIOB){
			GPIOB_PERIPH_CLK_EN();
		}else if(pGPIOx==GPIOC){
			GPIOC_PERIPH_CLK_EN();
		}else if(pGPIOx==GPIOD){
			GPIOD_PERIPH_CLK_EN();
		}else if(pGPIOx==GPIOE){
			GPIOE_PERIPH_CLK_EN();
		}else if(pGPIOx==GPIOF){
			GPIOF_PERIPH_CLK_EN();
		}else if(pGPIOx==GPIOG){
			GPIOG_PERIPH_CLK_EN();
		}else if(pGPIOx==GPIOH){
			GPIOH_PERIPH_CLK_EN();
		}else if(pGPIOx==GPIOI){
			GPIOI_PERIPH_CLK_EN();
		}
	//DISABLE
	}else{
		if(pGPIOx==GPIOA){
			GPIOA_PERIPH_CLK_DI();
		}else if(pGPIOx==GPIOB){
			GPIOB_PERIPH_CLK_DI();
		}else if(pGPIOx==GPIOC){
			GPIOC_PERIPH_CLK_DI();
		}else if(pGPIOx==GPIOD){
			GPIOD_PERIPH_CLK_DI();
		}else if(pGPIOx==GPIOE){
			GPIOE_PERIPH_CLK_DI();
		}else if(pGPIOx==GPIOF){
			GPIOF_PERIPH_CLK_DI();
		}else if(pGPIOx==GPIOG){
			GPIOG_PERIPH_CLK_DI();
		}else if(pGPIOx==GPIOH){
			GPIOH_PERIPH_CLK_DI();
		}else if(pGPIOx==GPIOI){
			GPIOI_PERIPH_CLK_DI();
		}
	}
}

void GPIO_Init(GPIO_Handle *pGPIO_Handle){
	uint32_t temp=0;
	GPIO_PeriphClkControl(pGPIO_Handle->pGPIOx, ENABLE);
	if(pGPIO_Handle->GPIO_PinConf.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//MODE NO INTERUPT
		temp=(pGPIO_Handle->GPIO_PinConf.GPIO_PinMode <<(2 * pGPIO_Handle->GPIO_PinConf.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER &= ~(0x3 << pGPIO_Handle->GPIO_PinConf.GPIO_PinNumber );
		pGPIO_Handle->pGPIOx->MODER |= temp;
		temp=0;
	}else{
		//MODE INTERUPT
	}
	temp=0;
	temp=(pGPIO_Handle->GPIO_PinConf.GPIO_PinSpeed <<(2 * pGPIO_Handle->GPIO_PinConf.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIO_Handle->GPIO_PinConf.GPIO_PinNumber );
	pGPIO_Handle->pGPIOx->OSPEEDR |= temp;

	temp=0;
	temp=(pGPIO_Handle->GPIO_PinConf.GPIO_PinPuPd <<(2 * pGPIO_Handle->GPIO_PinConf.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR &= ~(0x3 << pGPIO_Handle->GPIO_PinConf.GPIO_PinNumber );
	pGPIO_Handle->pGPIOx->PUPDR |= temp;

	temp=0;
	temp=(pGPIO_Handle->GPIO_PinConf.GPIO_PinOPType << pGPIO_Handle->GPIO_PinConf.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OTYPER &= ~(0x1 << pGPIO_Handle->GPIO_PinConf.GPIO_PinNumber );
	pGPIO_Handle->pGPIOx->OTYPER |= temp;

	if(pGPIO_Handle->GPIO_PinConf.GPIO_PinMode == GPIO_MODE_ALF){
		uint8_t temp1, temp2;
		temp1 = pGPIO_Handle->GPIO_PinConf.GPIO_PinNumber /8; //select AFR register
		temp2 = pGPIO_Handle->GPIO_PinConf.GPIO_PinNumber %8; //bit postion on register
		pGPIO_Handle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIO_Handle->pGPIOx->AFR[temp1] |=  (pGPIO_Handle->GPIO_PinConf.GPIO_PinAltFunMode << (4 * temp2));
	}
}
void GPIO_DeInit(GPIO_RegDef *pGPIOx){
	if(pGPIOx==GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx==GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx==GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx==GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx==GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx==GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx==GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx==GPIOH){
		GPIOH_REG_RESET();
	}else if(pGPIOx==GPIOI){
		GPIOI_REG_RESET();
	}
}
uint8_t  GPIO_ReadInputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value =(uint8_t)((pGPIOx->IDR >> PinNumber)& 0x00000001);
	return value;
}
uint16_t GPIO_ReadInputPort(GPIO_RegDef *pGPIOx){
	uint16_t value;
	value =(uint16_t)(pGPIOx->IDR);
	return value;
}
void GPIO_WriteOutputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber, uint8_t State){
	if(State== GPIO_PIN_SET){
		pGPIOx->ODR |= (1<<PinNumber);
	}else{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
void GPIO_ToggleOutputPin(GPIO_RegDef *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);
}

