#ifndef __ILI9341_H
#define __ILI9341_H

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

/**
	* @brief utility used for printing debug message through serial UART.
	*/
void serial_print(char* message);

void LCD_RST_1();
void LCD_RST_0();
void LCD_CS_1();
void LCD_CS_0();
void LCD_RS_1();
void LCD_RS_0();
void LCD_RS_RISING_EDGE();
void LCD_WR_1();
void LCD_WR_0();
void LCD_WR_RISING_EDGE();
void LCD_RD_1();
void LCD_RD_0();
void LCD_RD_RISING_EDGE();
void LCD_D2_1();
void LCD_D2_0();
void LCD_D3_1();
void LCD_D3_0();
void LCD_D4_1();
void LCD_D4_0();
void LCD_D5_1();
void LCD_D5_0();
void LCD_D6_1();
void LCD_D6_0();
void LCD_D7_1();
void LCD_D7_0();
void LCD_D0_1();
void LCD_D0_0();
void LCD_D1_1();
void LCD_D1_0();

void gpio_configure_8080_datapins_input_mode();
void gpio_configure_8080_datapins_output_mode();
uint8_t gpio_8080_read_parallel_datapins();
void gpio_8080_write_parallel_datapins(uint8_t write_data);
void gpio_8080_write_command(uint8_t cmd);
void gpio_8080_write_data(uint8_t data);
void gpio_8080_read_data(uint8_t* p_read_data, uint8_t length);



void ili9341_set_power_control_a();
void ili9341_set_power_control_b();
void ili9341_set_driver_timing_control_a();
void ili9341_set_driver_timing_control_b();
void ili9341_set_poweron_sequence_control_b();
void ili9341_set_pump_ratio_control();
void ili9341_set_power_control1();
void ili9341_set_power_control2();
void ili9341_set_vcom_control1();
void ili9341_set_vcom_control2();
void ili9341_set_memory_access_control();
void ili9341_set_pixel_format_set();
void ili9341_set_frame_rate_control();
void ili9341_set_display_function_control();
void ili9341_exit_sleep_mode();
void ili9341_set_display_on();
void ili9341_set_display_brightness();
void ili9341_memory_write();
void ili9341_hard_reset();
void ili9341_read_id4(uint8_t* p_read_data);







void ili9341_init();

void Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2);
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void Rect(unsigned int x,unsigned int y,unsigned int w,unsigned int h,unsigned int c);
void Rectf(unsigned int x,unsigned int y,unsigned int w,unsigned int h,unsigned int c);
int RGB(int r,int g,int b);
void LCD_Clear(unsigned int j);

#endif





