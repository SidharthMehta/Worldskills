/*
 * lcd_user.c
 *
 * Created: 28.1.2015 17:34:47
 *  Author: JKo
 */ 


#include "lcd_user.h"


I2C_HandleTypeDef *hi2c;

//
//**************************************************************************************************
//  Send comman to LCD display
//**************************************************************************************************
//
void lcd_write_cmd(uint8_t cmd)
{
    uint8_t buff[4];
   
    buff[0] = (cmd & 0xF0) | 0x0C;
    buff[1] = (cmd & 0xF0) | 0x08;
    buff[2] = ((cmd & 0x0F)<<4) | 0x0C;
    buff[3] = ((cmd & 0x0F)<<4) | 0x08;	
		for(int i=0; i<sizeof(buff); i++)
		{
			HAL_I2C_Master_Transmit(hi2c, LCD_WR_ADDRESS, &buff[i], 1, HAL_MAX_DELAY);
			HAL_Delay(1);
		}
		HAL_Delay(1);
}

//
//**************************************************************************************************
//  Set Cursor style
//  NONE, ON, OFF, BLINK
//**************************************************************************************************
//
void lcd_set_cursor_style(uint8_t style)
{
    lcd_write_cmd(0x0C);                       	// kursori valinnat pois
    lcd_write_cmd(style);                       // uusi tyyli
}

//
//**************************************************************************************************
//	LCD display initialization
//**************************************************************************************************
//
void lcd_init(I2C_HandleTypeDef *h)
{
		hi2c = h;
    static uint8_t init_cmd_buffer[] =
    {
			0x2C, 0x28, 0x8C, 0x88,
			0x0C, 0x08, 0xEC, 0xE8,
			0x0C, 0x08, 0x1C, 0x18,
			0x8C, 0x88, 0x0C, 0x08
 //       0x80, 0x38, 0x80, 0x39, 0x80, 0x1C, 0x80, 0x72, 0x80, 0x57,
 //       0x80, 0x6C, 0x80, 0x0F, 0x80, 0x01, 0x80, 0x06, 0x80, 0x02,
    };

		for(int i=0; i<sizeof(init_cmd_buffer); i++)
		{
			HAL_I2C_Master_Transmit(hi2c, LCD_WR_ADDRESS, init_cmd_buffer, sizeof(init_cmd_buffer), HAL_MAX_DELAY);
			HAL_Delay(1);
		}
}

//
//**************************************************************************************************
//	Set cursor to position: ROW, COL
//**************************************************************************************************
//
void lcd_goto_rc(uint8_t row, uint8_t col)
{
    if (row == 0)
    {
        lcd_write_cmd(0x80 + col);
    }
    else
    {
        lcd_write_cmd(0xC0 + col);
    }
}


//
//**************************************************************************************************
//	Send one character to display
//**************************************************************************************************
//
void lcd_putchar(char ch)
{
    uint8_t buff[4];
    
    buff[0] = (ch & 0xF0) | 0x0D;
    buff[1] = (ch & 0xF0) | 0x09;
    buff[2] = ((ch & 0x0F)<<4) | 0x0D;
    buff[3] = ((ch & 0x0F)<<4) | 0x09;	
    for(int i=0; i<sizeof(buff); i++)
		{
			HAL_I2C_Master_Transmit(hi2c, LCD_WR_ADDRESS, &buff[i], 1, HAL_MAX_DELAY);
			HAL_Delay(1);
		}
}


//
//**************************************************************************************************
//	Send string to to display
//**************************************************************************************************
void lcd_putstr(const char *str)
{
    while (*str != 0)
    {
        lcd_putchar(*str++);
    }
}


