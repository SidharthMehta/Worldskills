/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#define NORITAKE_VFD_HEIGHT             64
#define NORITAKE_VFD_LINES              8

#define RTC_WRITE		(0xD0)
#define RTC_READ		(0xD1)

#define RTC_SEC			(0x00)
#define RTC_MIN			(0x01)
#define RTC_HOUR		(0x02)
#define RTC_DAY			(0x03)
#define RTC_DATE		(0x04)
#define RTC_MONTH		(0x05)
#define RTC_YEAR		(0x06)
#define RTC_CONTROL	(0x07)

typedef struct
{
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
	unsigned char format;
	unsigned char ampm;
  uint8_t day;
  uint8_t date;
  uint8_t month;
  uint8_t year;  
}rtc_t;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
rtc_t DS1307;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void VFD_init(void);
void VFD_writePort(uint8_t);
void VFD_cmd(uint8_t);
void VFD_writeLine(char*);
void VFD_write(uint8_t);
void VFD_drawImageRom(unsigned, unsigned char, const unsigned char*);
void VFD_moveCursor(unsigned char, unsigned char);
void VFD_clrscr(void);
void VFD_home(void);
void VFD_scrollScreen(unsigned, unsigned, unsigned, unsigned char);

unsigned char RTC_get_SEC(void);
unsigned char RTC_get_MIN(void);
unsigned char RTC_get_HOUR(void);

unsigned char RTC_get_FORMAT(void);
unsigned char RTC_get_AMPM(void);

unsigned char RTC_get_DAY(void);
unsigned char RTC_get_DATE(void);
unsigned char RTC_get_MONTH(void);
unsigned char RTC_get_YEAR(void);

void RTC_set_SEC(uint8_t);
void RTC_set_MIN(uint8_t);
void RTC_set_HOUR(uint8_t, unsigned char, unsigned char);
void RTC_set_AMPM(unsigned char);
void RTC_set_DAY(uint8_t);
void RTC_set_DATE(uint8_t);
void RTC_set_MONTH(uint8_t);
void RTC_set_YEAR(uint8_t);

void RTC_i2c_write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData);
void RTC_i2c_read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData);
void RTC_init(void);

void project_displayTime(void);
void project_displayDay(void);
void project_displayDate(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
unsigned char img[1024]=
{
	//Insert logo here
};

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC_Init();

  /* USER CODE BEGIN 2 */
	VFD_init();
	VFD_clrscr();
	
	RTC_init();
	RTC_set_HOUR(11,'1','2'); //1 12 hour 2 24 hour //1 am 2 pm
	RTC_set_MIN(59);
	RTC_set_SEC(00);
	RTC_set_DAY(2);
	RTC_set_DATE(19);
	RTC_set_MONTH(9);
	RTC_set_YEAR(17);
	int cx=49,cy=12;
	
	VFD_drawImageRom(128,64,img);		//center
	HAL_Delay(4000);

	VFD_clrscr();
	
	int temp;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		VFD_moveCursor(0,0);
		project_displayTime();
		VFD_moveCursor(0,8);
		project_displayDay();
		VFD_moveCursor(0,16);
		project_displayDate();
		VFD_moveCursor(0,24);
		
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		temp = HAL_ADC_GetValue(&hadc);
		HAL_ADC_Stop(&hadc);
		
		VFD_writeLine("ADC Value: ");
		VFD_write( ( ( temp / 1000)	% 10) + '0' );
		VFD_write( ( ( temp / 100)	% 10) + '0' );
		VFD_write( ( ( temp / 10)		% 10) + '0' );
		VFD_write( ( ( temp / 1)		% 10) + '0' );
		VFD_home();
		
		if(HAL_GPIO_ReadPin(Check_GPIO_Port, Check_Pin) == GPIO_PIN_RESET)
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, GPIO_PIN_SET);
		else if(HAL_GPIO_ReadPin(Check_GPIO_Port, Check_Pin) == GPIO_PIN_SET)
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(Avail_GPIO_Port, Avail_Pin) == GPIO_PIN_RESET)
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, GPIO_PIN_SET);
		else if(HAL_GPIO_ReadPin(Avail_GPIO_Port, Avail_Pin) == GPIO_PIN_SET)
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, GPIO_PIN_RESET);
		
		if(HAL_GPIO_ReadPin(High_GPIO_Port, High_Pin) == GPIO_PIN_SET)
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
		else if(HAL_GPIO_ReadPin(High_GPIO_Port, High_Pin) == GPIO_PIN_RESET)
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
		if(HAL_GPIO_ReadPin(Low_GPIO_Port, Low_Pin) == GPIO_PIN_SET)
			HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_RESET);
		else if(HAL_GPIO_ReadPin(Low_GPIO_Port, Low_Pin) == GPIO_PIN_RESET)
			HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D0_Pin|D1_Pin|WR_Pin|RD_Pin 
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |B_Pin|R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L2_Pin|D2_Pin|D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : High_Pin */
  GPIO_InitStruct.Pin = High_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(High_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L1_Pin */
  GPIO_InitStruct.Pin = L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin D1_Pin WR_Pin RD_Pin 
                           D4_Pin D5_Pin D6_Pin D7_Pin 
                           B_Pin R_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D1_Pin|WR_Pin|RD_Pin 
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |B_Pin|R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L2_Pin D2_Pin D3_Pin */
  GPIO_InitStruct.Pin = L2_Pin|D2_Pin|D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Low_Pin Check_Pin */
  GPIO_InitStruct.Pin = Low_Pin|Check_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Avail_Pin */
  GPIO_InitStruct.Pin = Avail_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Avail_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void VFD_writeLine(char* data)
{
	int i=0;
	while(data[i]!='\0')
	{
		VFD_cmd(data[i++]);
	}
}

void VFD_init(void)
{
	VFD_cmd(0x1B);
	VFD_cmd('@');
}

void VFD_writePort(uint8_t data)
{
	
	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (data/128)%2?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (data/64)%2?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (data/32)%2?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (data/16)%2?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, (data/8)%2?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, (data/4)%2?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, (data/2)%2?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, (data/1)%2?GPIO_PIN_SET:GPIO_PIN_RESET);	
}

void VFD_cmd(uint8_t data)
{
	unsigned char ok;
	GPIO_InitTypeDef GPIO_InitStruct1;
	/*GPIOB-11 SET AS INPUT*/
	GPIO_InitStruct1.Pin = D7_Pin;
  GPIO_InitStruct1.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct1.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(D7_GPIO_Port, &GPIO_InitStruct1);
	
	do
	{
		HAL_GPIO_WritePin(RD_GPIO_Port,RD_Pin,GPIO_PIN_RESET);
		ok=HAL_GPIO_ReadPin(D7_GPIO_Port,D7_Pin);
		HAL_GPIO_WritePin(RD_GPIO_Port,RD_Pin,GPIO_PIN_SET);
	}while(ok);
	
	GPIO_InitStruct1.Pin =  D7_Pin;
  GPIO_InitStruct1.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct1.Pull = GPIO_NOPULL;
  GPIO_InitStruct1.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D7_GPIO_Port, &GPIO_InitStruct1);

	VFD_writePort(data);
	
	HAL_GPIO_WritePin(WR_GPIO_Port,WR_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(WR_GPIO_Port,WR_Pin,GPIO_PIN_SET);
}

void VFD_clrscr(void)
{
	VFD_cmd(0x0C);
}

void VFD_home(void)
{
	VFD_cmd(0x0B);
}

void VFD_write(uint8_t data)
{
	VFD_cmd(data);
}
void VFD_scrollScreen(unsigned x, unsigned y, unsigned times, unsigned char speed)
{
    unsigned pos = (x*NORITAKE_VFD_LINES)+(y/8);
    VFD_cmd(0x1F);
		VFD_cmd(0x28);
		VFD_cmd('a');
		VFD_cmd(0x10);
    VFD_cmd(pos);
    VFD_cmd(pos>>8);
    VFD_cmd(times);
    VFD_cmd(times>>8);
    VFD_cmd(speed);
}

void VFD_drawImageRom(unsigned width, unsigned char height, const unsigned char *data)
{
    unsigned int i;
    if (height > NORITAKE_VFD_HEIGHT) return;
    VFD_cmd(0x1F);
		VFD_cmd(0x28);
		VFD_cmd(0x66);
		VFD_cmd(0x11);
    VFD_cmd(width);
    VFD_cmd(width>>8);
    VFD_cmd(height/8);
    VFD_cmd((unsigned char) 0);
    VFD_cmd((unsigned char) 1);
    for ( i = 0; i<(height/8)*width; i++)
        VFD_cmd(data[i]);
}

void VFD_moveCursor(unsigned char x, unsigned char y)
{
		VFD_cmd(0x1f);
    VFD_cmd('$');
    VFD_cmd(x);
    VFD_cmd(x>>8);
    VFD_cmd(y/8);
    VFD_cmd(y/8>>8);
}


void RTC_i2c_read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData) 
{
  HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, 1, pData, 1, HAL_MAX_DELAY);
}

void RTC_i2c_write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t MemAddress, uint8_t *pData)
{
  HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, 1, pData, 1, HAL_MAX_DELAY);
}

unsigned char RTC_get_SEC(void)
{
	unsigned char sec;

	return sec;
}

unsigned char RTC_get_MIN(void)
{
	unsigned char min;

	return min;
}

unsigned char RTC_get_HOUR(void)
{
	unsigned char hour;

	return hour;
}

unsigned char RTC_get_FORMAT(void)		//Returns whether 1 - 12 hour or 2 - 24 hour format
{
	unsigned char hour, format;

	return format;
}

unsigned char RTC_get_AMPM(void)			//Returns 1- AM or 2 - PM if format is 12 hour
{
	unsigned char hour, ampm;
	
	return ampm;
}

unsigned char RTC_get_DAY(void)				//return 1 for monday, 2 for tuesday and so on till 7 for sunday
{
	unsigned char day;

	return day;
}

unsigned char RTC_get_DATE(void)
{
	unsigned char date;

	return date;
}

unsigned char RTC_get_MONTH(void)				//returns 1 to 12 for month
{
	unsigned char month;

	return month;
}

unsigned char RTC_get_YEAR(void)				//returns 0 to 99 for year
{
	unsigned char year;

	return year;
}

void RTC_init(void)
{
	unsigned char data = 0x00;
	RTC_i2c_write(&hi2c1, RTC_WRITE, RTC_CONTROL, &data);
	RTC_i2c_write(&hi2c1, RTC_WRITE, RTC_SEC, &data);
}

void RTC_set_SEC(uint8_t data)
{

}
void RTC_set_MIN(uint8_t data)
{

}

void RTC_set_HOUR(uint8_t data, unsigned char format, unsigned char ampm)
{

}

void RTC_set_AMPM(unsigned char ampm)
{

}
void RTC_set_DAY(uint8_t data)
{

}
void RTC_set_DATE(uint8_t data)
{

}

void RTC_set_MONTH(uint8_t data)
{

}
void RTC_set_YEAR(uint8_t data)
{

}

void project_displayTime(void)
{
	DS1307.sec = (uint8_t)RTC_get_SEC();
	DS1307.min = (uint8_t)RTC_get_MIN();
	DS1307.hour= (uint8_t)RTC_get_HOUR();
	
	DS1307.format	=	RTC_get_FORMAT();
	DS1307.ampm	=	RTC_get_AMPM();

	uint8_t hH= ((DS1307.hour)/10)%10;
	VFD_write(hH+'0');
	VFD_write(DS1307.hour%10+'0');
	VFD_write(':');
	
	uint8_t mH= ((DS1307.min)/10)%10;
	VFD_write(mH+'0');
	VFD_write(DS1307.min%10+'0');
	VFD_write(':');
	
	uint8_t sH= ((DS1307.sec)/10)%10;
	VFD_write(sH+'0');
	VFD_write(DS1307.sec%10+'0');
	
	if(DS1307.format == '1')
	{
		if(DS1307.ampm == '1')
			VFD_writeLine(" A.M.");
		else if(DS1307.ampm == '2')
			VFD_writeLine(" P.M.");
	}
}

void project_displayDay(void)
{
	DS1307.day	=	RTC_get_DAY();
	switch(DS1307.day)
	{
		case 1 : VFD_writeLine("Monday");
							break;
		case 2 : VFD_writeLine("Tuesday");
							break;
		case 3 : VFD_writeLine("Wednesday");
							break;
		case 4 : VFD_writeLine("Thursday");
							break;
		case 5 : VFD_writeLine("Friday");
							break;
		case 6 : VFD_writeLine("Saturday");
							break;
		case 7 : VFD_writeLine("Sunday");
							break;
	}
}
	
void project_displayDate(void)	
{
	DS1307.date	=	RTC_get_DATE();
	DS1307.month=	RTC_get_MONTH();
	DS1307.year	=	RTC_get_YEAR();
	
	uint8_t hD= ((DS1307.date)/10)%10;
	VFD_write(hD+'0');
	VFD_write(DS1307.date%10+'0');
	VFD_write('/');
	
	uint8_t hM= ((DS1307.month)/10)%10;
	VFD_write(hM+'0');
	VFD_write(DS1307.month%10+'0');
	VFD_write('/');
	
	uint8_t hY= ((DS1307.year)/10)%10;
	VFD_write(hY+'0');
	VFD_write(DS1307.year%10+'0');
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
