/*
 * lcd_cmd.c
 *
 *  Created on: Sep 26, 2022
 *      Author: Ryu
 */

#include "main.h"
#include "i2c.h"
#include "stdio.h"
#include "gpio.h"


#define LCD_ADDRESS 0x7c

void lcd_cmd(uint8_t x){
	uint8_t aTxBuffer[2] = { 0x00, x };
	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)LCD_ADDRESS, (uint8_t*)aTxBuffer, 2, 1000) != HAL_OK) {
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
			Error_Handler();
		}
	}
}

void lcd_data(uint8_t x){
	uint8_t aTxBuffer[2] = { 0x40, x };
	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)LCD_ADDRESS, (uint8_t*)aTxBuffer, 2, 1000) != HAL_OK)
	{
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
			Error_Handler();
		}
	}
}

void lcd_puts(const char*s){
	while(*s) lcd_data(*s++);
}
// param://  contrast: 0 ~ 63 (最初は大きくして調整)
void lcd_init(void){// LCD initialize(裏面に記載)
	lcd_cmd(0x38); // function set
	lcd_cmd(0x39); // function set
	lcd_cmd(0x14); // interval osc
	lcd_cmd(0x70); // contrast low
	lcd_cmd(0x56); // contrast high / icon / power
	lcd_cmd(0x6c); // follower control
	HAL_Delay(300);
	lcd_cmd(0x38); // function set
	lcd_cmd(0x0c); // display on
	lcd_cmd(0x01); // clear display
	HAL_Delay(2);
}

void lcd_move(uint8_t pos){
	lcd_cmd(0x80 | pos);
}

void lcd_pos(uint8_t raw, uint8_t col) {
	lcd_cmd(0x80 | ((raw & 0x01) << 6) | col);
}

void lcd_clear() {
	lcd_cmd(0x01);
	HAL_Delay(2);
}





