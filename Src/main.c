#include "stm32f407xx.h"
#include "string.h"

char UserData[] = "Hello World";
uint8_t flag;
void delay(void);
void GPIO_LED_Init(void);
void GPIO_BTN_Init(void);
void SPI_Inits(void);
void SPI_GPIOInit(void);

int main()
{

	SPI_Inits();

	//SPI_DeInit(SPI2);

	GPIO_LED_Init();
	//GPIO_BTN_Init();

	SPI_GPIOInit();

	SPI_SSOEConfig(SPI2, ENABLE);
	//SPI_SSIConfig(SPI2, ENABLE);
	GPIO_WriteOutputPin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	//GPIO_WriteOutputPin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);
//	SPI_PeripheralControl(SPI2,ENABLE);
//	SPI_SendData(SPI2,(uint8_t*) UserData, strlen(UserData));
	while(1){

		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_3);
		//delay();
		SPI_PeripheralControl(SPI2,ENABLE);
		//send length of data first

		//uint8_t LenData = strlen(UserData);
		//SPI_SendData(SPI2, &LenData, 1);
		//send data at UserData
		SPI_SendData(SPI2,(uint8_t*) UserData, strlen(UserData));
		//check SPI is in rest
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
		//disable SPI1
		SPI_PeripheralControl(SPI2,DISABLE);
	}
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

	SPI_GPIO.GPIO_PinConf.GPIO_PinAltFunMode=5;
	GPIO_Init(&SPI_GPIO);

//	SPI_GPIO.GPIO_PinConf.GPIO_PinNumber=GPIO_PIN_11;
//	GPIO_Init(&SPI_GPIO);

	SPI_GPIO.GPIO_PinConf.GPIO_PinNumber=GPIO_PIN_12;
	GPIO_Init(&SPI_GPIO);
}
void SPI_Inits(void)
{
	SPI_PeriClockControl(SPI2, ENABLE);

	SPI_Handle_t SPI2Handle;

	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULLDUPLEX;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MOD_MASTER;
	SPI2Handle.SPIConfig.SPI_Sclk_Speed = SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPT_SSM = SPI_SSM_DISABLE; // Hardware Slave Management

	SPI_Init(&SPI2Handle,SPI2);
}

