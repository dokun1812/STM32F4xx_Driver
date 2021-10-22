/*
 * spi_tx_testing.c
 *
 *  Created on: Oct 22, 2021
 *      Author: TrungDo
 */

#include "stm32f407xx.h"
#include "string.h"

char UserData[] = "Hello World";
void delay(void);
void GPIO_LED_Init(void);
void GPIO_BTN_Init(void);
void SPI_Inits(void);
void SPI_GPIOInit(void);

int main()
{
	//Init SPI peripheral
	SPI_Inits();
	//Init GPIO use for SPI
	SPI_GPIOInit();
	//Init GPIO use for LED
	GPIO_LED_Init();
	//Enable SSI bit
	SPI_SSIConfig(SPI3, ENABLE);
	//Enble SPI
	//SPI_PeripheralControl(SPI3,ENABLE);
	//Send Data
	//SPI_SendData(SPI3, (uint8_t *)UserData, strlen(UserData));
	while(1) {
		//Enable SPI
		SPI_PeripheralControl(SPI3,ENABLE);
		//Send Data
		SPI_SendData(SPI3, (uint8_t *)UserData, strlen(UserData));
		//Check Busy flag, when spi not busy
		while( SPI_GetFlagStatus(SPI3, SPI_BSY_FLAG) );
		//Disable SPI
		SPI_PeripheralControl(SPI3,DISABLE);
	}
//	while(1);
	return 0;
}
void delay(void){
	for(uint32_t t=0;t<250000;t++);
}
void GPIO_LED_Init(void){

	GPIO_PeriphClkControl(GPIOC,ENABLE);
	GPIO_PeriphClkControl(GPIOD,ENABLE);

	GPIO_Handle LED;

	LED.pGPIOx=GPIOC;
	LED.GPIO_PinConf.GPIO_PinNumber=GPIO_PIN_0;
	LED.GPIO_PinConf.GPIO_PinMode=GPIO_MODE_OUT;
	LED.GPIO_PinConf.GPIO_PinOPType=GPIO_OUT_TYPE_PP;
	LED.GPIO_PinConf.GPIO_PinPuPd=GPIO_NO_PUPD;
	LED.GPIO_PinConf.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&LED);
	LED.GPIO_PinConf.GPIO_PinNumber=GPIO_PIN_9;
	GPIO_Init(&LED);

	LED.pGPIOx=GPIOD;
	LED.GPIO_PinConf.GPIO_PinNumber=GPIO_PIN_3;
	LED.GPIO_PinConf.GPIO_PinMode=GPIO_MODE_OUT;
	LED.GPIO_PinConf.GPIO_PinOPType=GPIO_OUT_TYPE_PP;
	LED.GPIO_PinConf.GPIO_PinPuPd=GPIO_NO_PUPD;
	LED.GPIO_PinConf.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&LED);
}
void GPIO_BTN_Init(void){

	GPIO_PeriphClkControl(GPIOA,ENABLE);
	GPIO_PeriphClkControl(GPIOF,ENABLE);

	GPIO_Handle BTN;

	BTN.pGPIOx=GPIOA;
	BTN.GPIO_PinConf.GPIO_PinNumber=GPIO_PIN_0;
	BTN.GPIO_PinConf.GPIO_PinMode=GPIO_MODE_IN;
	BTN.GPIO_PinConf.GPIO_PinPuPd=GPIO_PD;
	BTN.GPIO_PinConf.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&BTN);

	BTN.pGPIOx=GPIOF;
	BTN.GPIO_PinConf.GPIO_PinNumber=GPIO_PIN_11;
	BTN.GPIO_PinConf.GPIO_PinMode=GPIO_MODE_IN;
	BTN.GPIO_PinConf.GPIO_PinPuPd=GPIO_PD;
	BTN.GPIO_PinConf.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&BTN);
}
void SPI_GPIOInit(void)
{
	GPIO_PeriphClkControl(GPIOC,ENABLE);
	GPIO_Handle SPI_GPIO;

	SPI_GPIO.pGPIOx=GPIOC;
	SPI_GPIO.GPIO_PinConf.GPIO_PinNumber=GPIO_PIN_10;
	SPI_GPIO.GPIO_PinConf.GPIO_PinMode=GPIO_MODE_ALF;
	SPI_GPIO.GPIO_PinConf.GPIO_PinOPType=GPIO_OUT_TYPE_PP;
	SPI_GPIO.GPIO_PinConf.GPIO_PinPuPd=GPIO_NO_PUPD;
	SPI_GPIO.GPIO_PinConf.GPIO_PinSpeed=GPIO_SPEED_FAST;

	SPI_GPIO.GPIO_PinConf.GPIO_PinAltFunMode=6;
	GPIO_Init(&SPI_GPIO);

	SPI_GPIO.GPIO_PinConf.GPIO_PinNumber=GPIO_PIN_11;
	GPIO_Init(&SPI_GPIO);

	SPI_GPIO.GPIO_PinConf.GPIO_PinNumber=GPIO_PIN_12;
	GPIO_Init(&SPI_GPIO);
}
void SPI_Inits(void)
{
	SPI_PeriClockControl(SPI3, ENABLE);

	SPI_Handle_t SPI3Handle;

	SPI3Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULLDUPLEX;
	SPI3Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MOD_MASTER;
	SPI3Handle.SPIConfig.SPI_Sclk_Speed = SPI_SCLK_SPEED_DIV4;
	SPI3Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI3Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI3Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI3Handle.SPIConfig.SPT_SSM = SPI_SSM_ENABLE; // Software Slave Management

	SPI_Init(&SPI3Handle,SPI3);
}



