/*
 * stm32f407xx_spi.c
 *
 *  Created on: Oct 20, 2021
 *      Author: caoth
 */

#include "stm32f407xx_spi.h"

/**
  * @brief  SPI_PeriClockControl
  * @Descri Enable Clock for SPI Bus
  * @param  SPI pointer point SPIx_BASE address
  * @param  Enable or Disable Clock
  * @retval None
  */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PERIPH_CLK_EN();
		} else if (pSPIx == SPI2)
		{
			SPI2_PERIPH_CLK_EN();
		} else if (pSPIx == SPI3)
		{
			SPI3_PERIPH_CLK_EN();
		}
	} else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PERIPH_CLK_DI();
		} else if (pSPIx == SPI2)
		{
			SPI2_PERIPH_CLK_DI();
		} else if (pSPIx == SPI3)
		{
			SPI3_PERIPH_CLK_DI();
		}
	}
}

/**
  * @brief  Init
  * @Descri Init the SPI peripheral
  * @param  pointer SPIHandle store base address SPI peripheral and SPI Config
  * @retval None
  */
void SPI_Init(SPI_Handle_t *pSPIHandle, SPI_RegDef_t *pSPIx)
{
	//configure SPI_CR1 register
	uint32_t tempreg = 0;
	//enable SPI clock
	//1. configure device mode: Master
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_POS);
	//SET_BIT(pSPIx->CR1,SPI_CR1_MSTR);
	//2. configure SPI bus
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULLDUPLEX)
	{
		//bidi mode should be cleared
		CLEAR_BIT(tempreg,SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALFDUPLEX)
	{
		//bidi mode should be set
		SET_BIT(tempreg,SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		//Rx onle should be set
		CLEAR_BIT(tempreg,SPI_CR1_BIDIMODE);
		SET_BIT(tempreg,SPI_CR1_RXONLY);
	}

	//3. configure spi serial clock speed
	tempreg |= pSPIHandle->SPIConfig.SPI_Sclk_Speed << SPI_CR1_BR_POS;
	//4. configure DFF: 16bit
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF_POS;
	//5. configure CPOL = 1
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_POS;
	//6. configure CPHA = 1
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_POS;
	//7. configure CPHA = 1
	tempreg |= pSPIHandle->SPIConfig.SPT_SSM << SPI_CR1_SSM_POS;
	//save value of tempreg into CR1
	pSPIx->CR1 = tempreg;
}

/**
  * @brief  De-Init
  * @Descri De-Init the SPI peripheral
  * @param  SPI pointer point SPIx_BASE address
  * @param  Enable or Disable Clock
  * @retval None
  */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx==SPI1){
		SPI1_REG_RESET();
	}else if(pSPIx==SPI2){
		SPI2_REG_RESET();
	}else if(pSPIx==SPI3){
		SPI3_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**
  * @brief  SPI_SSIConfig
  * @Descri Config for SPI use Software Slave Management
  * @param  base address SPIx
  * @param  ENABLE of DISABLE
  * @note
  * @retval None
  */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			//SET_BIT(pSPIx->CR1,SPI_CR1_MSTR);
			SET_BIT(pSPIx->CR1,SPI_CR1_SSI);
		} else
		{
			CLEAR_BIT(pSPIx->CR1,SPI_CR1_SSI);
		}
}
/**
  * @brief  SPI_SSOEConfig
  * @Descri Config NSS pin as output
  * @param  base address SPIx
  * @param  ENABLE of DISABLE
  * @note
  * @retval None
  */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
			{
				SET_BIT(pSPIx->CR2,SPI_CR2_SSOE);
			} else
			{
				CLEAR_BIT(pSPIx->CR2,SPI_CR2_SSOE);
			}
}

/**
  * @brief  SPI_PeripheralControl
  * @Descri Enable SPI for comunicate
  * @param  base address SPIx
  * @param  ENABLE of DISABLE
  * @note
  * @retval None
  */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		SET_BIT(pSPIx->CR1,SPI_CR1_SPE);
	} else
	{
		CLEAR_BIT(pSPIx->CR1,SPI_CR1_SPE);
	}
}

/**
  * @brief  SPI_SendData
  * @Descri Send data
  * @param  base address SPIx
  * @param  address Tx Buffer
  * @param  Number of bytes to transmit
  * @note
  * @retval None
  */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t * pTxBuffer, uint32_t LenData)
{
	while (LenData > 0)
	{
		//1. wait until the Tx buffer empty
		/*while ( !(pSPI->SR & (1 << 1)) );*/
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
		//2. check DFF bit in CR1
		if ( pSPIx->CR1 & SPI_CR1_DFF )
		{
			//DFF 16 bit
			//1. load data from TxBuffer into DR
			pSPIx->DR |= 2;
			LenData--;
			LenData--;
			(uint16_t*)pTxBuffer++;
		} else
		{
			//DFF 8 bit
			pSPIx->DR = 0x2;
			LenData--;
			pTxBuffer++;
		}
	}
}
/**
  * @brief  SPI_ReceiveData
  * @Descri Receive data
  * @param  base address SPIs
  * @param  address Tx Buffer
  * @param  Number of bytes to receive
  * @note
  * @retval None
  */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t LenData)
{
	while (LenData > 0)
	{
		//1. wait until the Rx buffer full <=> RXNE is set
		/*while ( !(pSPI->SR & (1 << 1)) );*/
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
		//2. check DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF_POS))
		{
			//DFF 16 bit
			//1. load data from DR to RxBuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			LenData--;
			LenData--;
			(uint16_t*)pRxBuffer++;
		} else
		{
			//DFF 8 bit
			*(pRxBuffer) = pSPIx->DR;
			LenData--;
			pRxBuffer++;
		}
	}
}


