/*
 * stm32f407xx.h
 *
 *  Created on: Oct 7, 2021
 *      Author: caoth
 */
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include <stdint.h>

//generic macros
#define ENABLE 					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_SET				SET
#define FLAG_RESET				RESET
#define SET_BIT(REG,POS) 		((REG) |= (POS))
#define CLEAR_BIT(REG,POS) 		((REG) &= ~(POS))
#define IS_SET(REG,POS)			if((REG) & (POS)) return 1;


#define vo volatile

typedef struct
{
  vo uint32_t MODER;    //Address offset: 0x00     
  vo uint32_t OTYPER;   //Address offset: 0x04
  vo uint32_t OSPEEDR;  //Address offset: 0x08
  vo uint32_t PUPDR;    //Address offset: 0x0C
  vo uint32_t IDR;      //Address offset: 0x10
  vo uint32_t ODR;      //Address offset: 0x14
  vo uint32_t BSRR;     //Address offset: 0x18
  vo uint32_t LCKR;     //Address offset: 0x1C
  vo uint32_t AFR[2];   //AFR[0]: 0x20   AFR[1]: 0x24
} GPIO_RegDef;

typedef struct
{
  vo uint32_t CR;            //Address offset: 0x00 
  vo uint32_t PLLCFGR;       //Address offset: 0x04 
  vo uint32_t CFGR;          //Address offset: 0x08 
  vo uint32_t CIR;           //Address offset: 0x0C 
  vo uint32_t AHB1RSTR;      //Address offset: 0x10 
  vo uint32_t AHB2RSTR;      //Address offset: 0x14 
  vo uint32_t AHB3RSTR;      //Address offset: 0x18 
  uint32_t    RESERVED0;     //Reserved, 0x1C                                                                    
  vo uint32_t APB1RSTR;      //Address offset: 0x20 
  vo uint32_t APB2RSTR;      //Address offset: 0x24 
  uint32_t    RESERVED1[2];  //Reserved, 0x28-0x2C                                                               
  vo uint32_t AHB1ENR;       //Address offset: 0x30
  vo uint32_t AHB2ENR;       //Address offset: 0x34
  vo uint32_t AHB3ENR;       //Address offset: 0x38 
  uint32_t    RESERVED2;     //Reserved, 0x3C                                                                    
  vo uint32_t APB1ENR;       //Address offset: 0x40 
  vo uint32_t APB2ENR;       //Address offset: 0x44 
  uint32_t    RESERVED3[2];  //Reserved, 0x48-0x4C                                                               
  vo uint32_t AHB1LPENR;     //Address offset: 0x50 
  vo uint32_t AHB2LPENR;     //Address offset: 0x54 
  vo uint32_t AHB3LPENR;     //Address offset: 0x58 
  uint32_t    RESERVED4;     //Reserved, 0x5C                                                                    
  vo uint32_t APB1LPENR;     //Address offset: 0x60 
  vo uint32_t APB2LPENR;     //Address offset: 0x64 
  uint32_t    RESERVED5[2];  //Reserved, 0x68-0x6C                                                               
  vo uint32_t BDCR;          //Address offset: 0x70 
  vo uint32_t CSR;           //Address offset: 0x74 
  uint32_t    RESERVED6[2];  //Reserved, 0x78-0x7C                                                               
  vo uint32_t SSCGR;         //Address offset: 0x80 
  vo uint32_t PLLI2SCFGR;    //Address offset: 0x84 
} RCC_RegDef;

typedef struct {
	vo uint32_t CR1;
	vo uint32_t CR2;
	vo uint32_t SR;
	vo uint32_t DR;
	vo uint32_t CRCPR;
	vo uint32_t RXCRCR;
	vo uint32_t TXCRCR;
	vo uint32_t I2SCFGR;
	vo uint32_t I2SPR;

} SPI_RegDef_t;


//STORAGE MAP
#define FLASH_BASE            0x08000000UL
#define ROM_BASE			  0x1FFF0000UL
#define SRAM1_BASE            0x20000000UL // 112KB <=> 114688BYTE <=> 0X0001C000UL
#define SRAM2_BASE            0x2001C000UL // SRAM2_BASE = (SRAM1_BASE + 0X0001C000UL)
//PERIPHERAL MAP
#define PERIPH_BASE           0x40000000UL
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)

//AHB1 PERIPHERAL
//RCC MAP
#define RCC_BASE 			  (AHB1PERIPH_BASE + 0x3800U)
//GPIO MAP
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400UL)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000UL)
//GPIO REGISTER
#define GPIOA 				  (GPIO_RegDef *)GPIOA_BASE
#define GPIOB 				  (GPIO_RegDef *)GPIOB_BASE
#define GPIOC 				  (GPIO_RegDef *)GPIOC_BASE
#define GPIOD 				  (GPIO_RegDef *)GPIOD_BASE
#define GPIOE 				  (GPIO_RegDef *)GPIOE_BASE
#define GPIOF 				  (GPIO_RegDef *)GPIOF_BASE
#define GPIOG 				  (GPIO_RegDef *)GPIOG_BASE
#define GPIOH 				  (GPIO_RegDef *)GPIOH_BASE
#define GPIOI 				  (GPIO_RegDef *)GPIOI_BASE

//RCC REGISTER
#define RCC					  (RCC_RegDef *)RCC_BASE
//SPI BASE ON ABP1
#define SPI2_BASE			  (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE			  (APB1PERIPH_BASE + 0x3C00UL)

//SPI BASE ON ABP2
#define SPI1_BASE			  (APB2PERIPH_BASE + 0x13000UL)

//SPI REGISTER
#define SPI1				  (SPI_RegDef_t *)SPI1_BASE
#define SPI2				  (SPI_RegDef_t *)SPI2_BASE
#define SPI3				  (SPI_RegDef_t *)SPI3_BASE

//Bit postion definitions of SPI_CR
#define SPI_CR1_CPHA_POS 		  	0
#define SPI_CR1_CPHA				(1 << SPI_CR1_CPHA_POS)
#define SPI_CR1_CPOL_POS  		  	1
#define SPI_CR1_CPOL				(1 << SPI_CR1_CPOL_POS)
#define SPI_CR1_MSTR_POS 		  	2
#define SPI_CR1_MSTR				(1 << SPI_CR1_MSTR_POS)
#define SPI_CR1_BR_POS 		  		3						//BR[0:2]
#define SPI_CR1_BR					(1 << SPI_CR1_BR0_POS)
#define SPI_CR1_SPE_POS 			6
#define SPI_CR1_SPE					(1 << SPI_CR1_SPE_POS)
#define SPI_CR1_LSBFRIST_POS 	  	7
#define SPI_CR1_LSBFRIST			(1 << SPI_CR1_LSBFRIST_POS)
#define SPI_CR1_SSI_POS 			8
#define SPI_CR1_SSI					(1 << SPI_CR1_SSI_POS)
#define SPI_CR1_SSM_POS 			9
#define SPI_CR1_SSM					(1 << SPI_CR1_SSM_POS)
#define SPI_CR1_RXONLY_POS 		  	10
#define SPI_CR1_RXONLY				(1 << SPI_CR1_RXONLY_POS)
#define SPI_CR1_DFF_POS 			11
#define SPI_CR1_DFF					(1 << SPI_CR1_DFF_POS)
#define SPI_CR1_CRCNEXT_POS 		12
#define SPI_CR1_CRCNEXT				(1 << SPI_CR1_CRCNEXT_POS)
#define SPI_CR1_CRCEN_POS 		  	13
#define SPI_CR1_CRCEN				(1 << SPI_CR1_CRCEN_POS)
#define SPI_CR1_BIDIOE_POS 		  	14
#define SPI_CR1_BIDIOE				(1 << SPI_CR1_BIDIOE_POS)
#define SPI_CR1_BIDIMODE_POS 	  	15
#define SPI_CR1_BIDIMODE			(1 << SPI_CR1_BIDIMODE_POS)
//Bit postion definitions of SPI_CR2
#define SPI_CR2_RXDMAEN_POS 		0
#define SPI_CR2_RXDMAEN				(1 << SPI_CR2_RXDMAEN_POS)
#define SPI_CR2_TXDMAEN_POS  	  	1
#define SPI_CR2_TXDMAEN				(1 << SPI_CR2_TXDMAEN_POS)
#define SPI_CR2_SSOE_POS 		  	2
#define SPI_CR2_SSOE				(1 << SPI_CR2_SSOE_POS)
#define Reserved_POS 			  	3
#define SPI_CR2_FRF_POS 			4
#define SPI_CR2_FRF					(1 << SPI_CR2_FRF_POS)
#define SPI_CR2_ERRIE_POS 		  	5
#define SPI_CR2_ERRIE				(1 << SPI_CR2_ERRIE_POS)
#define SPI_CR2_RXNEIE_POS 		  	6
#define SPI_CR2_RXNEIE				(1 << SPI_CR2_RXNEIE_POS)
#define SPI_CR2_TXEIE_POS 		  	7
#define SPI_CR2_TXEIE				(1 << SPI_CR2_TXEIE_POS)
//Bit postion definitions of SPI_SR
#define SPI_SR_RXNE_POS 			0
#define SPI_SR_TXE_POS 			  	1
#define SPI_SR_CHSIDE_POS 		  	2
#define SPI_SR_UDR_POS 			  	3
#define SPI_SR_CRCERR_POS 		  	4
#define SPI_SR_MODF_POS 			5
#define SPI_SR_OVR_POS 			  	6
#define SPI_SR_BSY_POS 			  	7
#define SPI_SR_FRE_POS 			  	8
//Define Flag Status
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE_POS) 		/* SPI status flag: Rx buffer not empty flag       */
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE_POS)		/* SPI status flag: Tx buffer empty flag       	   */
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE_POS)
#define SPI_UDR_FLAG					(1 << SPI_SR_UDR_POS)
#define SPI_CRCERR_FLAG					(1 << SPI_SR_CRCERR_POS)	/* SPI Error flag: CRC error flag                  */
#define SPI_MODF_FLAG					(1 << SPI_SR_MODF_POS)		/* SPI Error flag: Mode fault flag                 */
#define SPI_OVR_FLAG					(1 << SPI_SR_OVR_POS)		/* SPI Error flag: Overrun flag                    */
#define SPI_BSY_FLAG					(1 << SPI_SR_BSY_POS)		/* SPI status flag: Busy flag                      */
#define SPI_FRE_FLAG					(1 << SPI_SR_FRE_POS)		/* SPI Error flag: TI mode frame format error flag */
//BIT DEFINE FOR GPIO ON RCC_AHB1RSTR
#define RCC_AHB1RSTR_GPIOA_POS            0
#define RCC_AHB1RSTR_GPIOA             	(1<< RCC_AHB1RSTR_GPIOA_POS)
#define RCC_AHB1RSTR_GPIOB_POS            1
#define RCC_AHB1RSTR_GPIOB             	(1<< RCC_AHB1RSTR_GPIOB_POS)
#define RCC_AHB1RSTR_GPIOC_POS            2
#define RCC_AHB1RSTR_GPIOC             	(1<< RCC_AHB1RSTR_GPIOC_POS)
#define RCC_AHB1RSTR_GPIOD_POS            3
#define RCC_AHB1RSTR_GPIOD             	(1<< RCC_AHB1RSTR_GPIOD_POS)
#define RCC_AHB1RSTR_GPIOE_POS            4
#define RCC_AHB1RSTR_GPIOE             	(1<< RCC_AHB1RSTR_GPIOE_POS)
#define RCC_AHB1RSTR_GPIOF_POS            5
#define RCC_AHB1RSTR_GPIOF             	(1<< RCC_AHB1RSTR_GPIOF_POS)
#define RCC_AHB1RSTR_GPIOG_POS            6
#define RCC_AHB1RSTR_GPIOG             	(1<< RCC_AHB1RSTR_GPIOG_POS)
#define RCC_AHB1RSTR_GPIOH_POS            7
#define RCC_AHB1RSTR_GPIOH             	(1<< RCC_AHB1RSTR_GPIOH_POS)
#define RCC_AHB1RSTR_GPIOI_POS            8
#define RCC_AHB1RSTR_GPIOI             	(1<< RCC_AHB1RSTR_GPIOI_POS)

//Clock enable for GPIO Peripheral
#define GPIOA_PERIPH_CLK_EN_POS 0
#define GPIOA_PERIPH_CLK_EN()	SET_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOA)
#define GPIOA_PERIPH_CLK_DI()	CLEAR_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOA)
#define GPIOB_PERIPH_CLK_EN_POS 1
#define GPIOB_PERIPH_CLK_EN()   SET_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOB)
#define GPIOB_PERIPH_CLK_DI()   CLEAR_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOB)
#define GPIOC_PERIPH_CLK_EN_POS 2
#define GPIOC_PERIPH_CLK_EN()   SET_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOC)
#define GPIOC_PERIPH_CLK_DI()   CLEAR_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOC)
#define GPIOD_PERIPH_CLK_EN_POS 3
#define GPIOD_PERIPH_CLK_EN()   SET_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOD)
#define GPIOD_PERIPH_CLK_DI()   CLEAR_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOD)
#define GPIOE_PERIPH_CLK_EN_POS 4
#define GPIOE_PERIPH_CLK_EN()   SET_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOE)
#define GPIOE_PERIPH_CLK_DI()   CLEAR_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOE)
#define GPIOF_PERIPH_CLK_EN_POS 5
#define GPIOF_PERIPH_CLK_EN()   SET_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOF)
#define GPIOF_PERIPH_CLK_DI()   CLEAR_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOF)
#define GPIOG_PERIPH_CLK_EN_POS 6
#define GPIOG_PERIPH_CLK_EN()   SET_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOG)
#define GPIOG_PERIPH_CLK_DI()   CLEAR_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOG)
#define GPIOH_PERIPH_CLK_EN_POS 7
#define GPIOH_PERIPH_CLK_EN()   SET_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOH)
#define GPIOH_PERIPH_CLK_DI()   CLEAR_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOH)
#define GPIOI_PERIPH_CLK_EN_POS 8
#define GPIOI_PERIPH_CLK_EN()   SET_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOI)
#define GPIOI_PERIPH_CLK_DI()   CLEAR_BIT((*RCC).AHB1ENR,RCC_AHB1RSTR_GPIOI)

//GPIO RESET REGISTER
#define GPIOA_REG_RESET()   do{SET_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOA); CLEAR_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOA); }while(0)
#define GPIOB_REG_RESET()   do{SET_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOB); CLEAR_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOB); }while(0)
#define GPIOC_REG_RESET()   do{SET_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOC); CLEAR_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOC); }while(0)
#define GPIOD_REG_RESET()   do{SET_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOD); CLEAR_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOD); }while(0)
#define GPIOE_REG_RESET()   do{SET_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOE); CLEAR_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOE); }while(0)
#define GPIOF_REG_RESET()   do{SET_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOF); CLEAR_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOF); }while(0)
#define GPIOG_REG_RESET()   do{SET_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOG); CLEAR_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOG); }while(0)
#define GPIOH_REG_RESET()   do{SET_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOH); CLEAR_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOH); }while(0)
#define GPIOI_REG_RESET()   do{SET_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOI); CLEAR_BIT((*RCC).AHB1RSTR,RCC_AHB1RSTR_GPIOI); }while(0)


//Clock enable for SPI Peripheral
#define RCC_APB1RSTR_SPI1_POS	12
#define RCC_AHB1RSTR_SPI1		(1<< RCC_APB1RSTR_SPI1_POS)
#define SPI1_PERIPH_CLK_EN()	SET_BIT((*RCC).APB1ENR,RCC_AHB1RSTR_SPI1)
#define SPI1_PERIPH_CLK_DI()	CLEAR_BIT((*RCC).APB1ENR,RCC_AHB1RSTR_SPI1)
#define RCC_APB1RSTR_SPI2_POS	14
#define RCC_AHB1RSTR_SPI2		(1<< RCC_APB1RSTR_SPI2_POS)
#define SPI2_PERIPH_CLK_EN()	SET_BIT((*RCC).APB1ENR,RCC_APB1RSTR_SPI1_POS)
#define SPI2_PERIPH_CLK_DI()	CLEAR_BIT((*RCC).APB1ENR,RCC_APB1RSTR_SPI1_POS)
#define RCC_APB1RSTR_SPI3_POS	15
#define RCC_AHB1RSTR_SPI3		(1<< RCC_APB1RSTR_SPI3_POS)
#define SPI3_PERIPH_CLK_EN()	SET_BIT((*RCC).APB1ENR,RCC_AHB1RSTR_SPI3)
#define SPI3_PERIPH_CLK_DI()	CLEAR_BIT((*RCC).APB1ENR,RCC_AHB1RSTR_SPI3)


//IRQ (Interrupt Request)
#define IRQ_NO_EXTI0
#define IRQ_NO_EXTI1
#define IRQ_NO_EXTI2
#define IRQ_NO_EXTI3
#define IRQ_NO_EXTI4
#define IRQ_NO_EXTI9_5
#define IRQ_NO_EXTI15_10



#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio.h"

#endif /* INC_STM32F407XX_H_ */
