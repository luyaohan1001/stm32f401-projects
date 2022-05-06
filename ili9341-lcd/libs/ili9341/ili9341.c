#include "ili9341.h"

/* Includes ---------------------------------------------------------------------------------------------------------------------------------------*/


/* Macro Define -----------------------------------------------------------------------------------------------------------------------------------*/



/* gpio physical layer ----------------------------------------------------------------------------------------------------------------------------*/
/* GPIO Defined on STM32F401 */
/* LCD_RST   PC1 */
/* LCD_CS    PB0 */
/* LCD_RS (D/CX)    PA4 */
/* LCD_WR    PA1 */
/* LCD_RD    PA0 */

/* LCD_D2 PA10*/
/* LCD_D3 PB3*/
/* LCD_D4 PB5*/
/* LCD_D5 PB4*/
/* LCD_D6 PB10*/
/* LCD_D7 PA8*/
/* LCD_D0 PA9*/
/* LCD_D1 PC7*/


/**
  * @brief   Print debug string through USART.
  * @param   p_message Pointer to a message string.
  * @retval  None.
  * @note    When testing on STM32F401RE Nucleo Board, the board supports virtual COM (serial) port through USB.
  *            Connecting a USB-TTL adapter such as CH340 to the 'TX/D1' on morpho connector will not receive data.
  *             In the datasheet it has been confirmed that the USART2 pins have been to multiplexed for the virtual COM feature.
  *             On the PC, look for port /dev/ttyACM0 as the virtual serial port in CuteCom / MiniCom / Screen / Putty.
  */
__inline__ void serial_print(char* p_message)
{
  /* Call STM32 HAL library function to UART, pass uart hander, string, length to UART. */
  HAL_UART_Transmit(&huart2, (uint8_t*)p_message, strlen(p_message), 100);
}


void LCD_RST_1()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);  
}

void LCD_RST_0()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);  
}
 

void LCD_CS_1()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  
}

void LCD_CS_0()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  
}

 

void LCD_RS_1()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  
}
void LCD_RS_0()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  
}


void LCD_WR_1()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  
}
void LCD_WR_0()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  
}

void LCD_WR_RISING_EDGE()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  
}


void LCD_RD_1()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  
}
void LCD_RD_0()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);  
}


void LCD_D2_1()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);  
}
void LCD_D2_0()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);  
}


void LCD_D3_1()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);  
}
void LCD_D3_0()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);  
}


void LCD_D4_1()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);  
}
void LCD_D4_0()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  
}



void LCD_D5_1()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  
}
void LCD_D5_0()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  
}


void LCD_D6_1()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  
}
void LCD_D6_0()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  
}


void LCD_D7_1()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  
}

void LCD_D7_0()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  
}



void LCD_D0_1()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);  
}

void LCD_D0_0()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);  
}


void LCD_D1_1()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);  
}
void LCD_D1_0()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);  
}


void gpio_configure_8080_datapins_input_mode()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}


void gpio_configure_8080_datapins_output_mode()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

uint8_t gpio_8080_read_parallel_datapins() 
{
  uint8_t read_data;
  read_data = \
                HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)  << 7   \
              | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) << 6   \
              | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)  << 5   \
              | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)  << 4   \
              | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)  << 3   \
              | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) << 2   \
              | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)  << 1   \
              | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
  return read_data;
}

void gpio_8080_write_parallel_datapins(uint8_t write_data) 
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,  (write_data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (write_data & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,  (write_data & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,  (write_data & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  (write_data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, (write_data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,  (write_data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,  (write_data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}




void gpio_8080_write_command(uint8_t cmd)
{
  /* Chip Select */
  LCD_CS_0();
  LCD_RS_0();  
  LCD_RD_1();

  /* Write data to data bus D7-D0. */  
  gpio_configure_8080_datapins_output_mode();
  gpio_8080_write_parallel_datapins(cmd);
  LCD_WR_RISING_EDGE();

  /* De-select Chip. */
  LCD_CS_1();
}

void gpio_8080_write_data(uint8_t data) 
{
  LCD_CS_0();
  LCD_RS_1();  
  LCD_RD_1();

  gpio_configure_8080_datapins_output_mode();
  gpio_8080_write_parallel_datapins(data);
  LCD_WR_RISING_EDGE();
  LCD_CS_1();
}

void gpio_8080_read_data(uint8_t* p_read_data, uint8_t length) 
{
  LCD_CS_0();
  LCD_RS_1();  
  LCD_WR_1();

  gpio_configure_8080_datapins_input_mode();
  for (int i = 0; i < length; ++i)
  {
    LCD_RD_0();
    p_read_data[i] = gpio_8080_read_parallel_datapins();
    LCD_RD_1();
  }

  LCD_CS_1();
}


/** 
  * @brief Set Power Control A. 
  */
void ili9341_set_power_control_a()
{
  gpio_8080_write_command(0xCB);    
  gpio_8080_write_data(0x39); 
  gpio_8080_write_data(0x2C); 
  gpio_8080_write_data(0x00); 
  gpio_8080_write_data(0x34);   //设置 Vcore=1.6V
  gpio_8080_write_data(0x02);   //设置DDVDH=5.6V
}

void ili9341_set_power_control_b()
{
  gpio_8080_write_command(0xCF);   // 指令Power Control B
  gpio_8080_write_data(0x00); 
  gpio_8080_write_data(0XC1); 
  gpio_8080_write_data(0X30); 
}


void ili9341_set_driver_timing_control_a()
{
  gpio_8080_write_command(0xE8);   //指令 Driver timing Congrol A 
  gpio_8080_write_data(0x85); 
  gpio_8080_write_data(0x00); 
  gpio_8080_write_data(0x78); 
}

void ili9341_set_driver_timing_control_b()
{
  gpio_8080_write_command(0xEA);  //指令 Driver timing Congrol B
  gpio_8080_write_data(0x00); 
  gpio_8080_write_data(0x00); 
}

void ili9341_set_poweron_sequence_control_b()
{
  gpio_8080_write_command(0xED);  //指令 Power on sequence control 
  gpio_8080_write_data(0x64); 
  gpio_8080_write_data(0x03); 
  gpio_8080_write_data(0X12); 
  gpio_8080_write_data(0X81); 
}

/**/
void ili9341_set_pump_ratio_control()
{
  gpio_8080_write_command(0xF7);  //指令Pump ratio control
  gpio_8080_write_data(0x20); //DDVDH=2*VCL
}

void ili9341_set_power_control1()
{
  gpio_8080_write_command(0xC0);    //Power control 
  gpio_8080_write_data(0x23);   //VRH[5:0]  GVDD=4.6V
}

void ili9341_set_power_control2()
{
  gpio_8080_write_command(0xC1);    //Power control 
  gpio_8080_write_data(0x10);   //SAP[2:0];BT[3:0] 
}


void ili9341_set_vcom_control1()
{
  gpio_8080_write_command(0xC5);    //VCM control  1  
  gpio_8080_write_data(0x3e);   //Contrast  VCOMH=3.45V VCOML=-1.5V
  gpio_8080_write_data(0x28); 
}

void ili9341_set_vcom_control2()
{
  gpio_8080_write_command(0xC7);    //VCM control2 
  gpio_8080_write_data(0x86);   //--
}

void ili9341_set_memory_access_control()
{
  gpio_8080_write_command(0x36);    // Memory Access Control 
  //gpio_8080_write_data(0x48);  // MX=1 Column Address Order ; BGR=1 RGB(IC)-->BGR(LCD Panel)
  gpio_8080_write_data(0x08);  // MX=0,BGR =1
}

void ili9341_set_pixel_format_set()
{
  gpio_8080_write_command(0x3A);    //指令Pixel Format Set 
  gpio_8080_write_data(0x55);   //RGB 接口和MCU接口模式的像素数据格式为16bit/pixel  
}

void ili9341_set_frame_rate_control()
{
  gpio_8080_write_command(0xB1);    //Frame Rate Control (B1h)（In Normal Mode /Full colors ）
  gpio_8080_write_data(0x00);  
  gpio_8080_write_data(0x10);  //79HZ(frame rate)
}

void ili9341_set_display_function_control()
{
  gpio_8080_write_command(0xB6);    // Display Function Control 
  gpio_8080_write_data(0x08);   // Interval Scan 
  gpio_8080_write_data(0x82);   //底背景为白屏， 5 frams Scan Cycle
  gpio_8080_write_data(0x27);   //320 line
}

void ili9341_exit_sleep_mode()
{
  gpio_8080_write_command(0x11);    //Exit Sleep 
  HAL_Delay(120);             //必须120ms的延迟
}

void ili9341_set_display_on()
{
  gpio_8080_write_command(0x29);    //Display on 

}

void ili9341_memory_write()
{
  gpio_8080_write_command(0x2c);    //Memory Write Start(2C) 或 Memory Write Continue(3ch)
}

void ili9341_hard_reset() 
{
  LCD_RST_1();
  HAL_Delay(1);
  LCD_RST_0();
  HAL_Delay(1);
  LCD_RST_1();
  HAL_Delay(1);
}

/**
	* @brief Read LCD Controller Chip (ILI9341) ID. 
	* @note  It has been tested that some display modules has all manufacturer/version set to 0.
	*        Thus a more trustworthy way to test 8080 Read is to read the controller IC (ILI9341)'s ID through ID4 register.
	*/
void ili9341_get_id4(uint8_t* p_read_data) 
{
  gpio_8080_write_command(0xD3);
  gpio_8080_read_data(p_read_data, 4);

  char msg[64];
	sprintf(msg, "- Printing ID4 register value -\n");
	serial_print(msg);
	sprintf(msg, "ILI9341 IC Version: %d \nIC Model: 0x%02x%02x\n", p_read_data[1], p_read_data[2], p_read_data[3]); /* id4[0] is dummy byte. */
	serial_print(msg);
}



void ili9341_init()
{

  ili9341_hard_reset();
  ili9341_set_power_control_a();
  ili9341_set_power_control_b();
  ili9341_set_driver_timing_control_a();
  ili9341_set_driver_timing_control_b();
  ili9341_set_poweron_sequence_control_b();
  ili9341_set_pump_ratio_control();
  ili9341_set_power_control1();
  ili9341_set_power_control2();
  ili9341_set_vcom_control1();
  ili9341_set_vcom_control2();
  ili9341_set_memory_access_control();
  ili9341_set_pixel_format_set();
  ili9341_set_frame_rate_control();
  ili9341_set_display_function_control();
  ili9341_exit_sleep_mode();
  ili9341_set_display_on();
  ili9341_memory_write();
}



void ili9341_set_column_address(uint16_t x1, uint16_t x2)
{
  gpio_8080_write_command(0x2a);
  gpio_8080_write_data(x1>>8);   //设定屏幕数据操作区域的列首地址数据，，先写入16bit数据位的高位
  gpio_8080_write_data(x1);      //写入16bit数据位的低位
  gpio_8080_write_data(x2>>8);   //设定屏幕数据操作区域的列尾地址数据，，先写入16bit数据位的高位
  gpio_8080_write_data(x2);      //写入16bit数据位的低位
}

void ili9341_set_page_address(uint16_t y1, uint16_t y2)
{
  gpio_8080_write_command(0x2b);
  gpio_8080_write_data(y1>>8);     //设定屏幕数据操作区域的行首地址数据，，先写入16bit数据位的高位
  gpio_8080_write_data(y1);         //写入16bit数据位的低位
  gpio_8080_write_data(y2>>8);      //设定屏幕数据操作区域的行尾地址数据，，先写入16bit数据位的高位
  gpio_8080_write_data(y2);         //写入16bit数据位的低位
}


/**
	* @brief Send to ILI9341 the define area of frame memory where MCU can access.
	*/
void ili9341_set_frame_address(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	ili9341_set_column_address(x1, x2);
	ili9341_set_page_address(y1, y2);
	ili9341_memory_write();
}




/**
	* @brief Draw a horizontal line.
	*/
void lcd_draw_horizontal_line(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t length, uint16_t line_color)                   
{ 
  gpio_8080_write_command(0x2C);       //write_memory_start
	uint16_t end_coordinate_x = start_coordinate_x + length;
	uint16_t end_coordinate_y = start_coordinate_y;

	/* Sets the frame memory area. */
  ili9341_set_frame_address(start_coordinate_x, start_coordinate_y, end_coordinate_x, end_coordinate_y);  

	/* Fill with line_color. */
  for(uint16_t i = 1; i<=length; ++i)
  {
    gpio_8080_write_data(line_color >> 8);  /* MSB first. */
    gpio_8080_write_data(line_color);      
  }
}

/**
	* Draw a vertical line. Fill color if necessary.
	*/
void lcd_draw_vertical_line(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t length, uint16_t line_color)                   
{ 
  
  gpio_8080_write_command(0x2c); //write_memory_start
  
	uint16_t end_coordinate_x = start_coordinate_x;
	uint16_t end_coordinate_y = start_coordinate_y + length;
	/* Sets the frame memory area. */
  ili9341_set_frame_address(start_coordinate_x, start_coordinate_y, end_coordinate_x, end_coordinate_y); 

	/* Fill with line_color. */
  for(uint16_t i=1; i<=length; ++i)
  { 
    gpio_8080_write_data(line_color>>8);   
    gpio_8080_write_data(line_color);       
  }
}

/**
	* @brief Draw an empty rectangle frame.
	* @note  This is equivalent to drawing two vertical lines plus two horizontal lines.
	*/
void lcd_draw_rectangle_unfilled(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t rect_width, uint16_t rect_height, uint16_t frame_color)
{
  lcd_draw_horizontal_line(start_coordinate_x, start_coordinate_y, rect_width, frame_color);
  lcd_draw_horizontal_line(start_coordinate_x, start_coordinate_y + rect_height, rect_width, frame_color);

  lcd_draw_vertical_line(start_coordinate_x, start_coordinate_y, rect_height, frame_color);
  lcd_draw_vertical_line(start_coordinate_x + rect_width, start_coordinate_y, rect_height, frame_color);
}

/**
	* @brief Draw a solid color-filled rectangle.
	*/
void lcd_draw_rectangle_filled(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t rect_width, uint16_t rect_height, uint16_t rect_color)
{
  for (uint16_t i = 0; i < rect_height; ++i)
  {
    lcd_draw_horizontal_line(start_coordinate_x, start_coordinate_y + i, rect_width, rect_color);
  }
}

/**
	* @brief Clear the entire screen and fill with color.
	*/
void lcd_clear_all(uint16_t fill_color)                   
{ 
	uint16_t start_coordinate_x = 0;
	uint16_t start_coordinate_y = 0;
	uint16_t end_coordinate_x = TFT_PIXEL_H_LENGTH - 1;
	uint16_t end_coordinate_y = TFT_PIXEL_V_WIDTH - 1;

  ili9341_set_frame_address(start_coordinate_x, start_coordinate_y, end_coordinate_x, end_coordinate_y);
  for(uint16_t i = 0; i < TFT_PIXEL_V_WIDTH; ++i)  
    for(uint16_t m = 0; m < TFT_PIXEL_H_LENGTH; ++m) 
    {
			/* Write color to fill. */
      gpio_8080_write_data(fill_color>>8);
      gpio_8080_write_data(fill_color);
    }
}


void lcd_draw_dot(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t dot_color) 
{
	uint16_t end_coordinate_x = start_coordinate_x + 1;
	uint16_t end_coordinate_y = start_coordinate_y + 1;
  ili9341_set_frame_address(start_coordinate_x, start_coordinate_y, end_coordinate_x, end_coordinate_y);  
  gpio_8080_write_data(dot_color >> 8); 
  gpio_8080_write_data(dot_color);      
}


void lcd_plot_char(int16_t x, int16_t y, unsigned char c,uint16_t color, uint16_t bg, uint8_t size) 
{

 
	for (int8_t i=0; i<6; i++ )
	{
		uint8_t line;

		if (i == 5)
		{
			line = 0x0;
		}
		else 
		{
			line = *((uint8_t*)(font+(c*5)+i));
		}

		for (int8_t j = 0; j<8; j++)
		{
			if (line & 0x1)
			{
				if (size == 1) // default size
				{
					lcd_draw_dot(x+i, y+j, color);
				}
				else {  // big size
					// ili9341_plot_color_block(x+(i*size), y+(j*size), size, size, color);
				} 
			} else if (bg != color)
			{
				if (size == 1) // default size
				{
					lcd_draw_dot(x+i, y+j, bg);
				}
				else 
				{  // big size
					//ili9341_plot_color_block(x+i*size, y+j*size, size, size, bg);
				}
			}

			line >>= 1;
		}
	}
}

void lcd_set_rotation(uint8_t orientation) 
{

  gpio_8080_write_command(0x36);    

	switch (orientation) 
	{
		case 0:
			gpio_8080_write_data(0x40|0x08);
			break;
		case 1:
			gpio_8080_write_data(0x20|0x08);
			break;
		case 2:
			gpio_8080_write_data(0x80|0x08);
			break;
		case 3:
			gpio_8080_write_data(0x40|0x80|0x20|0x08);
			break;
	}
}


void lcd_write_message(char* message, uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint8_t size, uint16_t text_color, uint16_t text_bg_color){
	for(uint8_t i = 0; i< strlen(message); i++){
		char single_char = message[i];
		lcd_plot_char(start_coordinate_x, start_coordinate_y, single_char, text_color, text_bg_color, 1);
		start_coordinate_x += 6;
	}
}
