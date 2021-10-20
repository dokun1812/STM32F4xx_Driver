/*
 * spi_tx_testing.c
 *
 *  Created on: Oct 16, 2021
 *      Author: COMPUTER
 */

#include "stm32f407xx.h"
#include "string.h"

void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}

// PA7 -> SPI2_MOSI
// PA6 -> SPI2_MISO
// PA5 -> SPI2_SCLK
// PA4 -> SPI2_NSS
// Alternate Function mode: 5

void SPI_GPIOInits()
{
	GPIO_Handle SPI_Pins;

	SPI_Pins.pGPIOx = GPIOA;
	SPI_Pins.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_ALF;
	SPI_Pins.GPIO_PinConf.GPIO_PinAltFunMode = 5;
	SPI_Pins.GPIO_PinConf.GPIO_PinPuPd = GPIO_NO_PUPD;
	SPI_Pins.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SPI_Pins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_4; // NSS
	GPIO_Init(&SPI_Pins);
	SPI_Pins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_5; // SCLK
	GPIO_Init(&SPI_Pins);
//	SPI_Pins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_6; // MISO
//	GPIO_Init(&SPI_Pins);
	SPI_Pins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_7; // MOSI
	GPIO_Init(&SPI_Pins);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle GPIOBtn;

	//Btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_0;
	GPIOBtn.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConf.GPIO_PinPuPd = GPIO_PD;

	GPIO_Init(&GPIOBtn);
}

void SPI_Inits(void)
{
	SPI_Handle_t SPI1Handle;

	SPI1Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULLDUPLEX;
	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MOD_MASTER;
	SPI1Handle.SPIConfig.SPI_Sclk_Speed = SPI_SCLK_SPEED_DIV8;
	SPI1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1Handle.SPIConfig.SPT_SSM = SPI_SSM_DISABLE; // Hardware Slave Management

	SPI_Init(&SPI1Handle);

}

int main()
{
	char UserData[] = "Hello World";

	//initialize GPIO pins use for SPI1
	SPI_GPIOInits();

	//initialize SPI1
	SPI_Inits();

	//config NSS pin as output
	SPI_SSOEConfig(SPI1, ENABLE);
	while(1)
	{
		//wait until press button
		while( !(GPIO_ReadInputPin(GPIOA, GPIO_PIN_0)) );

		//delay 200ms
		delay();

		//enable SPI1
		SPI_PeripheralControl(SPI1,ENABLE);

		//send length of data first
		uint8_t LenData = strlen(UserData);
		SPI_SendData(SPI1, &LenData, 1);

		//send data at UserData
		SPI_SendData(SPI1,(uint8_t*) UserData, strlen(UserData));

		//check SPI is in rest
		while( SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG) );
		//disable SPI1
		SPI_PeripheralControl(SPI1,DISABLE);

	}

	return 0;
}

