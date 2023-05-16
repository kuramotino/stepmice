/*
 * lcd_cmd.h
 *
 *  Created on: Sep 26, 2022
 *      Author: Ryu
 */

#ifndef INC_LCD_CMD_H_
#define INC_LCD_CMD_H_

extern void lcd_cmd(uint8_t x);
extern void lcd_data(uint8_t x);
extern void lcd_puts(const char*s);
extern void lcd_init(void);
extern void lcd_move(uint8_t pos);
 void lcd_pos(uint8_t raw, uint8_t col);
extern void lcd_clear();

#endif /* INC_LCD_CMD_H_ */
