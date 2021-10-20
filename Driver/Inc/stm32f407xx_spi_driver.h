/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Oct 13, 2021
 *      Author: COMPUTER
 */
#ifndef STM32F407XX_SPI_DRIVER_H_
#define STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

// Configuration structure for SPIx peripheral
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_Sclk_Speed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPT_SSM;
} SPI_Config_t;

// Handle structure for SPIx peripheral
typedef struct {
	SPI_RegDef_t *pSPIx; // Base address of SPIx peripheral
	SPI_Config_t SPIConfig;
}SPI_Handle_t;

//Define device mode
#define SPI_DEVICE_MOD_MASTER			1
#define SPI_DEVICE_MOD_SLAVE			0

//Define Bus Config
#define SPI_BUS_CONFIG_FULLDUPLEX		1
#define SPI_BUS_CONFIG_HALFDUPLEX		2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

//Define SPI_Sclk_Speed
#define SPI_SCLK_SPEED_DIV2 			0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

//Define SPI_DFF
#define SPI_DFF_8BITS 					1
#define SPI_DFF_16BITS					0

//Define CPOL
#define SPI_CPOL_HIGH 					1
#define SPI_CPOL_LOW					0

//Define CPHA
#define SPI_CPHA_HIGH 					1
#define SPI_CPHA_LOW					0

//Define SSM
#define SPI_SSM_ENABLE					1
#define SPI_SSM_DISABLE					0




/*APIs support by this driver*/

//Peripheral Clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
//Init and De-init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Handle_t *pSPIHandle);

//Data Send and Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t LenData);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t LenData);

//Peripheral Control APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//IRQ Configuration and ISR handing
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

#endif /* STM32F407XX_SPI_DRIVER_H_ */
