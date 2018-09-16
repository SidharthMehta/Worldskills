/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "lcd_user.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//
//**************************************************************************************
//	Global enums, structures
//**************************************************************************************
//

// 	these colors means different floors
typedef enum 
{
	BLACK, RED, GREEN, YELLOW, BLUE, VIOLET, CYAN, WHITE
} color_t;

typedef struct 
{
	uint8_t row;					// coordinates of labyrinth
	uint8_t col;					// row, col, floor
	color_t floor;
} position_t;							// type definition of position

position_t pos;							// player's position



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */

//
//****************************************************************************************
//	Global function prototypes
//****************************************************************************************
//
void delay_us(uint32_t us);
void hw_init(void);
void shift_register_reset(void);
void shift_register_clk_pulse(void);
void shift_register_load_pulse(void);
void shift_register_write(uint8_t data);
void row_counter_reset(void);
void row_counter_clk_pulse(void);
void set_row(uint8_t row);
void set_dot(uint8_t row, uint8_t col, color_t color);
void set_color(color_t color);

//
//****************************************************************************************
//	Global variables
//****************************************************************************************
//
char lcd_buff[17];										// buffer for sprintf function to print on LCD display 
char buff[17];											// buffer for sprintf function to print on USB device 

//****************************************************************************************
//	This is labyrinth data
//****************************************************************************************
#define FORWARD		0x01							// wall directions and values
#define RIGHT		0x02							// you can't go in this direction if current cell
#define BACKWARD	0x04							// contains this wall (floor and/or ceiling)
#define LEFT		0x08
#define DOWN		0x10
#define	UP			0x20

//											F  R  C											F = floor, R = row, C = colum
const uint8_t labyrinth[7][8][8] = {
	
//  col 0	col1  col2  col3  col4  col5  col6	col 7	  row 0 of array is  
//																										row 7 of labyrinth!

		0x39,	0x31,	0x37,	0x39,	0x35,	0x37,	0x39,	0x17,	// row 7	FLOOR 1 (RED)
		0x3A,	0x3E,	0x39,	0x34,	0x33,	0x3B,	0x3C,	0x33,	// row 6
		0x38,	0x31,	0x32,	0x1B,	0x38,	0x32,	0x3B,	0x3A,	// row 5
		0x3A,	0x3A,	0x3A,	0x3A,	0x3A,	0x3E,	0x3C,	0x32,	// row 4
		0x3E,	0x3E,	0x38,	0x32,	0x3A,	0x39,	0x31,	0x36,	// row 3
		0x39,	0x31,	0x36,	0x3E,	0x3A,	0x3E,	0x3A,	0x3B,	// row 2
		0x3A,	0x3A,	0x39,	0x33,	0x38,	0x35,	0x32,	0x3A,	// row 1
		0x3E,	0x3C,	0x36,	0x3E,	0x3C,	0x37,	0x3C,	0x36,	// row 0
									
		0x39,	0x31,	0x37,	0x39,	0x35,	0x37,	0x39,	0x27,	// row 7	FLOOR 2 (GREEN)
		0x3A,	0x3E,	0x39,	0x34,	0x33,	0x3B,	0x3A,	0x3B,
		0x38,	0x31,	0x32,	0x2B,	0x38,	0x32,	0x3A,	0x3A,
		0x3A,	0x3A,	0x3A,	0x3A,	0x3A,	0x3E,	0x3C,	0x32,
		0x3E,	0x3E,	0x3A,	0x3A,	0x3A,	0x39,	0x31,	0x36,
		0x39,	0x31,	0x36,	0x3E,	0x3A,	0x3E,	0x3A,	0x3B,
		0x3E,	0x3A,	0x39,	0x33,	0x38,	0x35,	0x32,	0x3A,
		0x1D,	0x34,	0x36,	0x3E,	0x3C,	0x37,	0x3C,	0x36,
									
		0x1D,	0x33,	0x39,	0x35,	0x35,	0x35,	0x35,	0x33,	// row 7	FLOOR 3 (YELLOW)
		0x3B,	0x3A,	0x3A,	0x39,	0x31,	0x35,	0x33,	0x3A,
		0x38,	0x32,	0x3A,	0x3E,	0x3A,	0x39,	0x36,	0x3A,
		0x3E,	0x3A,	0x3C,	0x17,	0x3A,	0x3C,	0x37,	0x3A,
		0x39,	0x34,	0x31,	0x35,	0x30,	0x35,	0x35,	0x32,
		0x3A,	0x3D,	0x30,	0x33,	0x3A,	0x39,	0x37,	0x3A,
		0x3C,	0x37,	0x3E,	0x3E,	0x3A,	0x3C,	0x35,	0x36,
		0x2D,	0x35,	0x35,	0x35,	0x34,	0x35,	0x35,	0x37,
									
		0x2D,	0x33,	0x39,	0x35,	0x35,	0x35,	0x35,	0x33,	// row 7	FLOOR 4 (BLUE)
		0x3B,	0x3A,	0x38,	0x35,	0x37,	0x39,	0x33,	0x3A,
		0x38,	0x32,	0x3A,	0x3D,	0x33,	0x3A,	0x3A,	0x3A,
		0x3E,	0x3A,	0x3A,	0x25,	0x36,	0x3E,	0x3A,	0x3A,
		0x39,	0x36,	0x3A,	0x39,	0x35,	0x31,	0x30,	0x32,
		0x3A,	0x3D,	0x32,	0x3A,	0x3B,	0x3E,	0x3A,	0x3A,
		0x3C,	0x17,	0x3E,	0x3E,	0x38,	0x35,	0x36,	0x3E,
		0x3D,	0x35,	0x35,	0x35,	0x34,	0x35,	0x35,	0x17,
									
		0x1D,	0x33,	0x39,	0x37,	0x3D,	0x35,	0x35,	0x33,	// row 7	FLOOR 5 (VIOLET)
		0x3B,	0x3A,	0x3A,	0x39,	0x33,	0x39,	0x35,	0x32,
		0x38,	0x32,	0x3C,	0x36,	0x3A,	0x3C,	0x37,	0x3A,
		0x3E,	0x3C,	0x33,	0x1D,	0x34,	0x35,	0x33,	0x3A,
		0x39,	0x37,	0x38,	0x35,	0x35,	0x37,	0x38,	0x36,
		0x3A,	0x3D,	0x34,	0x35,	0x31,	0x35,	0x36,	0x3B,
		0x38,	0x25,	0x35,	0x37,	0x38,	0x35,	0x35,	0x36,
		0x3C,	0x35,	0x35,	0x37,	0x3C,	0x35,	0x35,	0x27,
									
		0x2D,	0x33,	0x39,	0x35,	0x35,	0x31,	0x35,	0x33,	// row 7	FLOOR 6 (CYAN)
		0x3B,	0x3A,	0x38,	0x35,	0x37,	0x3A,	0x3B,	0x3E,
		0x38,	0x32,	0x3A,	0x3D,	0x33,	0x3C,	0x34,	0x33,
		0x3E,	0x3A,	0x38,	0x25,	0x36,	0x39,	0x37,	0x3A,
		0x39,	0x36,	0x3A,	0x3D,	0x33,	0x38,	0x31,	0x32,
		0x3A,	0x3D,	0x30,	0x35,	0x36,	0x3E,	0x3A,	0x3A,
		0x3C,	0x37,	0x3E,	0x3D,	0x31,	0x35,	0x36,	0x3E,
		0x1D,	0x35,	0x35,	0x35,	0x34,	0x35,	0x35,	0x37,
									
		0x3D,	0x33,	0x3D,	0x35,	0x31,	0x31,	0x35,	0x37,	// row 7	FLOOR 7 (WHITE)
		0x3B,	0x38,	0x35,	0x35,	0x32,	0x3A,	0x3B,	0x3B,
		0x38,	0x32,	0x39,	0x35,	0x32,	0x3C,	0x34,	0x32,
		0x3E,	0x3A,	0x38,	0x37,	0x3A,	0x39,	0x37,	0x3A,
		0x39,	0x36,	0x3A,	0x3D,	0x32,	0x38,	0x35,	0x32,
		0x3A,	0x3D,	0x30,	0x37,	0x3A,	0x3E,	0x3B,	0x3A,
		0x3C,	0x37,	0x3E,	0x3D,	0x30,	0x35,	0x36,	0x3E,
		0x2D,	0x35,	0x35,	0x35,	0x34,	0x35,	0x35,	0x37,
};


//
//***************************************************************************************
//	This is NOT an accurate delay function
//  1 -> 1,8 us, 3 -> 3,5 us, 5 -> 5,2 us, 10 -> 9,8 us, 20 -> 18 us
//***************************************************************************************
//
void delay_us(uint32_t us) 
{
	volatile uint32_t counter = 3*us;
	while(counter--);
}


//
//***************************************************************************************
//	This function does all Hardware initializations
//***************************************************************************************
//
void hw_init(void)
{	
	// 74HC595 shift register init
	HAL_GPIO_WritePin(COL_LOAD_GPIO_Port, COL_LOAD_Pin, GPIO_PIN_RESET);	// load low
	HAL_GPIO_WritePin(COL_CLK_GPIO_Port, COL_CLK_Pin, GPIO_PIN_RESET);		// clk low
	HAL_GPIO_WritePin(COL_DATA_GPIO_Port, COL_DATA_Pin, GPIO_PIN_RESET);	// data low
	shift_register_reset();
	
	HAL_GPIO_WritePin(ROW_CLK_GPIO_Port, ROW_CLK_Pin, GPIO_PIN_RESET);		// row clk low
	HAL_GPIO_WritePin(ROW_RST_GPIO_Port, ROW_RST_Pin, GPIO_PIN_SET);		// row rst high
	delay_us(5);
	HAL_GPIO_WritePin(ROW_RST_GPIO_Port, ROW_RST_Pin, GPIO_PIN_RESET);		// row rst low
	
}


//
//***************************************************************************************
//	This function reset 74HC595 shift register
//***************************************************************************************
//
void shift_register_reset(void)
{
	HAL_GPIO_WritePin(GPIOA, COL_RST_Pin, GPIO_PIN_RESET);	// reset low (= reset)
	delay_us(5);
	HAL_GPIO_WritePin(GPIOA, COL_RST_Pin, GPIO_PIN_SET);	// release reset
	delay_us(5);
}


//
//***************************************************************************************
//	This function do one clock pulse to shift register
//***************************************************************************************
//
void shift_register_clk_pulse(void)
{
	HAL_GPIO_WritePin(GPIOA, COL_CLK_Pin, GPIO_PIN_SET);
	delay_us(1);
	HAL_GPIO_WritePin(GPIOA, COL_CLK_Pin, GPIO_PIN_RESET);
}


//
//***************************************************************************************
// This function loads data of shift register to it's outputs
//***************************************************************************************
//
void shift_register_load_pulse(void)
{
	HAL_GPIO_WritePin(GPIOA, COL_LOAD_Pin, GPIO_PIN_SET);
	delay_us(1);
	HAL_GPIO_WritePin(GPIOA, COL_LOAD_Pin, GPIO_PIN_RESET);
}	


//
//***************************************************************************************
//	This function updates leds
//***************************************************************************************
void shift_register_write(uint8_t data)
{
	uint8_t i;
	
	shift_register_reset();						// reset shift register
	
	for (i=0; i<8; i++)							// 8 bit data
	{
		if (data & (0x80 >> i))					// 1000 0000 start from MSB	
			HAL_GPIO_WritePin(GPIOA, COL_DATA_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOA, COL_DATA_Pin, GPIO_PIN_RESET);
		
		shift_register_clk_pulse();
	}

	shift_register_load_pulse();			// load register 8 bit date to output latch
}




//
//***************************************************************************************
//	This function reset row counter
//***************************************************************************************
//
void row_counter_reset(void)
{
	HAL_GPIO_WritePin(ROW_RST_GPIO_Port, ROW_RST_Pin, GPIO_PIN_SET);		// row rst high
	delay_us(5);
	HAL_GPIO_WritePin(ROW_RST_GPIO_Port, ROW_RST_Pin, GPIO_PIN_RESET);		// row rst low
}


//
//***************************************************************************************
//	This function do one clock pulse to row counter
//***************************************************************************************
//
void row_counter_clk_pulse(void)
{
	HAL_GPIO_WritePin(ROW_CLK_GPIO_Port, ROW_CLK_Pin, GPIO_PIN_SET);		// row clk high
	delay_us(1);
	HAL_GPIO_WritePin(ROW_CLK_GPIO_Port, ROW_CLK_Pin, GPIO_PIN_RESET);		// row clk low
}


//
//***************************************************************************************
//	This function set row
//***************************************************************************************
//
void set_row(uint8_t row)
{
	uint8_t i;
	
	row_counter_reset();								// reset row counter
	for(i=0; i<(7-row); i++)							// do row times clock pulses to counter
  {
		row_counter_clk_pulse();
  }
}


//
//***************************************************************************************
//	This function clear all dots (whole display is blank)
//***************************************************************************************
//
void clr_all_dots(void)
{
	shift_register_write(0xFF);								// all leds off
}


//
//***************************************************************************************
//	This function set color of led and position row, col
//	Col format (24 bit), B = Blue, G = Green, R = Red
//	B7 B6 B5 B4 B3 B2 B1 B0 G7 G6 G5 G4 G3 G2 G1 G0 R7 R6 R5 R4 R3 R2 R1 R0
//***************************************************************************************
//
void set_dot(uint8_t row, uint8_t col, color_t color)
{
	shift_register_write(0xFF);								// all leds off
	set_row(row);											// set row
	shift_register_write(~(0x01 << col));						//Turn on LED corresponding to column
	set_color(color);
}

//
//***************************************************************************************
//	This function set color of led for floor
//	Col format (24 bit), B = Blue, G = Green, R = Red
//	B7 B6 B5 B4 B3 B2 B1 B0 G7 G6 G5 G4 G3 G2 G1 G0 R7 R6 R5 R4 R3 R2 R1 R0
//***************************************************************************************
//
void set_color(color_t color)
{
	if (color & RED)															// if color includes red color
		HAL_GPIO_WritePin(red_led_GPIO_Port, red_led_Pin, GPIO_PIN_SET);						// Turn on red color
	else
		HAL_GPIO_WritePin(red_led_GPIO_Port, red_led_Pin, GPIO_PIN_RESET);					// Turn off red color
	if (color & GREEN)															// if color includes green color
		HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_SET);				// Turn on green color	
	else
		HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_RESET);			// Turn off green color
	if (color & BLUE)															// if color includes blue color
		HAL_GPIO_WritePin(blue_led_GPIO_Port, blue_led_Pin, GPIO_PIN_SET);					// Turn on blue color
	else
		HAL_GPIO_WritePin(blue_led_GPIO_Port, blue_led_Pin, GPIO_PIN_RESET);					// Turn off blue color
}

//
//***************************************************************************************
//	this function gives alarm sound
//***************************************************************************************
//
void beep(void)
{
	uint8_t i;
	
	for (i=0; i<2 ; i++)
	{
		HAL_GPIO_WritePin(PZ_GPIO_Port, PZ_Pin, GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(PZ_GPIO_Port, PZ_Pin, GPIO_PIN_RESET);
		HAL_Delay(50);
	}	
}



//
//***************************************************************************************
//	This function returns value of labyrint array 	
//***************************************************************************************
//
uint8_t get_faces(position_t p)
{
	return labyrinth[p.floor-1][7-p.row][p.col];
}

//
//***************************************************************************************
//	now you finist the game
//***************************************************************************************
//
void game_over(void)
{
	uint8_t i;
	
	set_dot(7, 7, (color_t)7);
	
	for(i=0; i< 10; i++)
 	{
		beep();
		HAL_Delay(50);
	}	
	while(1)
		continue;
}

/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint16_t	steps = 0;									// how many steps you have walking
	uint16_t 	hits = 0;									// mow many times you hit a wall
	uint8_t		dly_cntr = 0;								// this for delays
	
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
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  hw_init();
  
  lcd_init(&hi2c1);						// initialize LCD display
  lcd_set_cursor_style(CURSOR_OFF);     // don't show cursor
  lcd_write_cmd(CLEAR_DISPLAY);         // clear display
	
  lcd_putstr("Worldskills");
  HAL_Delay(2000);
  lcd_write_cmd(CLEAR_DISPLAY);         // clear display
  
  pos.row = 0;
  pos.col = 0;
  pos.floor = RED;					// RED = 1 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
// Shortcut jumps
		if (flag == 1)
		{	
			pos.floor = (color_t)7;
			pos.row = 7;
			pos.col = 6;
			flag = 0;
		}

//---------------------------------------------------------------------------------------		
// 	flash LED if it possible change floor up or down
//---------------------------------------------------------------------------------------		
		if (!((get_faces(pos) & UP) && (get_faces(pos) & DOWN)))
		{
			dly_cntr++;
			dly_cntr %= 50;
			if (dly_cntr > 45)
			{
				if (get_faces(pos) & UP)
					set_dot(pos.row, pos.col, (color_t)(pos.floor-1));
				else
					set_dot(pos.row, pos.col, (color_t)(pos.floor+1));
			}	
			else
			{
				set_dot(pos.row, pos.col, (color_t)pos.floor);
			}			
		}
		else
		{
			set_dot(pos.row, pos.col, (color_t)pos.floor);
		}

//---------------------------------------------------------------------------------------		
// 	update LCD display
//---------------------------------------------------------------------------------------		
		lcd_goto_rc(0, 0);
		sprintf(lcd_buff, "F:%u R:%u C:%u  %02Xh", pos.floor, pos.row, pos.col, get_faces(pos));
		lcd_putstr(lcd_buff);
		lcd_goto_rc(1, 0);
		sprintf(lcd_buff, "S:%u", steps);
		lcd_putstr(lcd_buff);

		lcd_goto_rc(1, 8);
		sprintf(lcd_buff, "H:%u", hits);
		lcd_putstr(lcd_buff);
	
//---------------------------------------------------------------------------------------		
		if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0 ||	flag == 2)	// is FORWARD pressed?
		{
			if (!(get_faces(pos) & FORWARD)) 									// is it possible to go forward?
			{
				pos.row++;																			// move forward		
				steps++;
			}
			else
			{
				beep();
				hits++;
			}
			flag = 0;
			
			HAL_Delay(10);
			while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0)
				continue;
		}

//---------------------------------------------------------------------------------------		
		if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0 ||	flag == 3)	// is LEFT pressed?			
		{
			if (!(get_faces(pos) & LEFT))											// is it possible to go left?
			{
				pos.col--;																			// move left
				steps++;
			}
			else
			{
				beep();
				hits++;
			}			
			flag = 0;
			
			HAL_Delay(10);
			while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0)
				continue;
		}

//---------------------------------------------------------------------------------------		
		if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0 ||	flag == 5)	// is RIGHT pressed?
		{
			if (!(get_faces(pos) & RIGHT)) 										// is it possible to go right?
			{
				pos.col++;																			// move right
				steps++;
			}
			else
			{
				beep();
				hits++;
			}			
			flag = 0;
			
			HAL_Delay(10);
			while (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0)
				continue;
		}

//---------------------------------------------------------------------------------------		
		if (HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) == 0 ||	flag == 4)	// is BACKWARD pressed?			
		{
			if (!(get_faces(pos) & BACKWARD))									// is it possible to go backward?
			{
				pos.row--;																			// move backward
				steps++;
			}
			else
			{
				beep();
				hits++;
			}			
			flag = 0;
			
			HAL_Delay(10);
			while (HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) == 0)
				continue;
		}
		
//---------------------------------------------------------------------------------------		
		if (HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == 0 ||	flag == 6)	// is DOWN pressed?
		{
			if (!(get_faces(pos) & DOWN)) 										// is it possible to go down?
			{
				pos.floor--;																		// move down
				steps++;
			}
			else
			{
				beep();
				hits++;
			}			
			flag = 0;
			
			sprintf(buff, "F:%u R:%u C:%u  %02Xh", pos.floor, pos.row, pos.col, get_faces(pos));
			CDC_Transmit_FS(buff, strlen(buff));
			sprintf(buff, "S:%u", steps);
			CDC_Transmit_FS(buff, strlen(buff));
			sprintf(buff, "H:%u", hits);
			CDC_Transmit_FS(buff, strlen(buff));

			HAL_Delay(10);
			while (HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == 0)
				continue;
		}

//---------------------------------------------------------------------------------------		
		if (HAL_GPIO_ReadPin(SW6_GPIO_Port, SW6_Pin) == 0 ||	flag == 7)	// is UP pressed?			
		{
			if (!(get_faces(pos) & UP))												// is it possible to go up?
			{
				pos.floor++;																		// move up
				steps++;
			}
			else
			{
				beep();
				hits++;
			}			
			flag = 0;
			
						
			sprintf(buff, "F:%u R:%u C:%u  %02Xh", pos.floor, pos.row, pos.col, get_faces(pos));
			CDC_Transmit_FS(buff, strlen(buff));
			sprintf(buff, "S:%u", steps);
			CDC_Transmit_FS(buff, strlen(buff));
			sprintf(buff, "H:%u", hits);
			CDC_Transmit_FS(buff, strlen(buff));

			HAL_Delay(10);
			while (HAL_GPIO_ReadPin(SW6_GPIO_Port, SW6_Pin) == 0)
				continue;
		}

		if (pos.row == 7 && pos.col == 7 && pos.floor == 7)
		{
			game_over();
		}
		
		HAL_Delay(10);																			// every loops take least 10 ms
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ROW_CLK_Pin|ROW_RST_Pin|COL_RST_Pin|COL_CLK_Pin 
                          |COL_LOAD_Pin|COL_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, blue_led_Pin|green_led_Pin|red_led_Pin|PZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin SW4_Pin 
                           SW5_Pin SW6_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin|SW4_Pin 
                          |SW5_Pin|SW6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW_CLK_Pin ROW_RST_Pin COL_RST_Pin COL_CLK_Pin 
                           COL_LOAD_Pin COL_DATA_Pin */
  GPIO_InitStruct.Pin = ROW_CLK_Pin|ROW_RST_Pin|COL_RST_Pin|COL_CLK_Pin 
                          |COL_LOAD_Pin|COL_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : blue_led_Pin green_led_Pin red_led_Pin PZ_Pin */
  GPIO_InitStruct.Pin = blue_led_Pin|green_led_Pin|red_led_Pin|PZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
