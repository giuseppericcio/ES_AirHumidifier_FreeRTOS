#include "stm32f3xx_hal.h"
#include "DHT11.h"

extern TIM_HandleTypeDef htim7;
#define DHT_TIMER &htim7

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1


void DHT_Delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(DHT_TIMER, 0);
	while ((__HAL_TIM_GET_COUNTER(DHT_TIMER))<time);
}


void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM;
int TEMP, RH;


void DHT11_Start (void)
{
	Set_Pin_Output (DHT11_PORT, DHT11_PIN);         // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	DHT_Delay (18000);   // wait for 18ms
    HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
    DHT_Delay (20);      // wait for 30us
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);           // set as input
}

uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	DHT_Delay (40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		DHT_Delay (80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
		else Response = -1;
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i = 0;
	uint8_t j = 0;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
		DHT_Delay (40);         // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))      // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));   // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go low
	}
	return i;
}

uint8_t DHT11_Get_Data (int *Temperature, int *Humidity)
{
    DHT11_Start ();
	if (DHT11_Check_Response ())
	{
		Rh_byte1 = DHT11_Read ();
		Rh_byte2 = DHT11_Read ();
		Temp_byte1 = DHT11_Read ();
		Temp_byte2 = DHT11_Read ();
		SUM = DHT11_Read();

		if (SUM == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))
		{
			TEMP = Temp_byte1;
			RH = Rh_byte1;
		}

		else return -1;
	}

	else return -1;

    *Temperature = TEMP;
    *Humidity = RH;

    return 1;
}
