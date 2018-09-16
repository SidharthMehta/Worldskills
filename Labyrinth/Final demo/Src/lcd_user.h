/*
 * lcd_user.h
 *
 * Created: 28.1.2015 17:37:38
 *  Author: JKo
 */ 


#ifndef LCD_USER_H_
#define LCD_USER_H_
#include <stdint.h>
#include "stm32l0xx_hal.h"


#define LCD_WR_ADDRESS      0x7E		// I2C address of LCD display                                    

#define CLEAR_DISPLAY       0x01    // Clear display
#define CURSOR_HOME         0x02    // Set cursor to row 0 col 0 
#define NORMAL_HEIGHT       0x28    // Normal font
#define DOUBLE_HEIGHT       0x2B    // Double height font



#define CURSOR_OFF          0x0C    // hide cursor
#define CURSOR_ON           0x0E    // show cursor
#define CURSOR_OFF_BLINK    0x0D    // 
#define CURSOR_ON_BLINK     0x0F    // cursor blinks

void lcd_write_cmd(uint8_t cmd);
void lcd_set_cursor_style(uint8_t style);
void lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_goto_rc(uint8_t row, uint8_t col);
void lcd_putchar(char ch);
void lcd_putstr(const char *str);


#endif /* LCD_USER_H_ */
