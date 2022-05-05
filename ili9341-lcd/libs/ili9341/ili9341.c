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
    /*
    char msg[64];
    sprintf(msg, "D7: %d  ", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) );
    serial_print(msg);
    sprintf(msg, "D6: %d  ", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10));
    serial_print(msg);
    sprintf(msg, "D5: %d  ", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) );
    serial_print(msg);
    sprintf(msg, "D4: %d  ", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) );
    serial_print(msg);
    sprintf(msg, "D3: %d  ", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) );
    serial_print(msg);
    sprintf(msg, "D2: %d  ", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10));
    serial_print(msg);
    sprintf(msg, "D1: %d  ", HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) );
    serial_print(msg);
    sprintf(msg, "D0: %d\n", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9));
    serial_print(msg);
    */

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
  gpio_8080_write_data(0x18);  //79HZ(frame rate)
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
void ili9341_read_id4(uint8_t* p_read_data) 
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




//地址区域设置。涉及指令2Ah、2Bh
void Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{
  gpio_8080_write_command(0x2a);
  gpio_8080_write_data(x1>>8);   //设定屏幕数据操作区域的列首地址数据，，先写入16bit数据位的高位
  gpio_8080_write_data(x1);      //写入16bit数据位的低位
  gpio_8080_write_data(x2>>8);   //设定屏幕数据操作区域的列尾地址数据，，先写入16bit数据位的高位
  gpio_8080_write_data(x2);      //写入16bit数据位的低位
  gpio_8080_write_command(0x2b);
  gpio_8080_write_data(y1>>8);     //设定屏幕数据操作区域的行首地址数据，，先写入16bit数据位的高位
  gpio_8080_write_data(y1);         //写入16bit数据位的低位
  gpio_8080_write_data(y2>>8);      //设定屏幕数据操作区域的行尾地址数据，，先写入16bit数据位的高位
  gpio_8080_write_data(y2);         //写入16bit数据位的低位
  gpio_8080_write_command(0x2c);  //开启RAM数据持续写入状态。         
}




//画水平线。。设定需填色的行列起止地址范围后往里填色
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c)                   
{ 

  //x,y 为水平线的起始坐标 ，，l为水平线长度单位为像素，c为颜色参数
  unsigned int i,j;

  
  gpio_8080_write_command(0x02c);       //write_memory_start
  LCD_RS_1();
  LCD_CS_0();

  l=l+x;  //转换成终止列的X坐标  
  Address_set(x,y,l,y);   //框出要填色的区域

  j=l;                //确定要填入的像素个数
  for(i=1;i<=j;i++)
  {
    gpio_8080_write_data(c>>8);   //写入颜色数据的高8位
    gpio_8080_write_data(c);      //写入颜色数据的低8位
  }
  LCD_CS_1();
}


//画垂直线。。设定需填色的行列起止地址范围后往里填色
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c)                   
{ 
  unsigned int i,j;
  
  gpio_8080_write_command(0x02c); //write_memory_start
  LCD_RS_1();
  LCD_CS_0();
  
  l=l+y;  //转换成终止行的Y坐标
  Address_set(x,y,x,l); //框出要填色的区域
  j=l;             //确定要填入的像素个数
  for(i=1;i<=j;i++)
  { 
    gpio_8080_write_data(c>>8);    //写入颜色数据的高8位
    gpio_8080_write_data(c);       //写入颜色数据的低8位
  }
  LCD_CS_1();
}


//画空心矩形
void Rect(unsigned int x,unsigned int y,unsigned int w,unsigned int h,unsigned int c)
{
  H_line(x  , y  , w, c);
  H_line(x  , y+h, w, c);
  V_line(x  , y  , h, c);
  V_line(x+w, y  , h, c);
}


//画实心矩形
void Rectf(unsigned int x,unsigned int y,unsigned int w,unsigned int h,unsigned int c)
{
  unsigned int i;
  for(i=0;i<h;i++)
  {
    H_line(x  , y+i, w, c);
  }
}


int RGB(int r,int g,int b)
{return r << 16 | g << 8 | b;
}




//满屏填充
void LCD_Clear(unsigned int j)                   
{ 
  unsigned int i,m;

  LCD_CS_0();
  Address_set(0,0,239,319);


  for(i=0;i<320;i++)  //320个行
    for(m=0;m<240;m++) //240列
    {
      gpio_8080_write_data(j>>8);
      gpio_8080_write_data(j);

    }
  LCD_CS_1();
}

