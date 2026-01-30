/*
 * LCD_DISPLAY.h
 *
 *  Created on: 27-Dec-2025
 *      Author: Acer
 */

#ifndef LCD_DISPLAY_H_
#define LCD_DISPLAY_H_


#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

esp_err_t lcd_send_cmd(uint8_t cmd);
esp_err_t lcd_send_data(uint8_t data);

void LCD_INITILIZED(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_write(char *str);
void clear_lcd(void);
esp_err_t lcd_display_on(bool enable);




#endif /* MAIN_LCD_DISPLAY_H_ */





