#ifndef __NRF24_H
#define __NRF24_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>




#define TX_ADR_WIDTH  5        
#define RX_ADR_WIDTH  5       
#define TX_PLOAD_WIDTH  32  
#define RX_PLOAD_WIDTH  32  

#define R_REGISTER_MASK   0x00                   
#define W_REGISTER_MASK   0x20
#define R_RX_PAYLOAD  0x61                      
#define W_TX_PAYLOAD  0xA0                       
#define FLUSH_TX     0xE1                       
#define FLUSH_RX     0xE2                      
#define REUSE_TX_PL  0xE3                       
#define NOP          0xFF                      

// Mnemonic   Address  Description
#define CONFIG        0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA         0x01  // 自动应答功能设置
#define EN_RXADDR     0x02  // 可用信道设置
#define SETUP_AW      0x03  // 收发地址宽度设置
#define SETUP_RETR    0x04  // 自动重发功能设置
#define RF_CH         0x05  // 工作频率设置
#define RF_SETUP      0x06  // 发射速率、功耗功能设置
#define STATUS        0x07  // 状态寄存器
#define OBSERVE_TX    0x08  // 发送监测功能
#define CD            0x09  // 地址检测           
#define RX_ADDR_P0    0x0A  // 频道0接收数据地址
#define RX_ADDR_P1    0x0B  // 频道1接收数据地址
#define RX_ADDR_P2    0x0C  // 频道2接收数据地址
#define RX_ADDR_P3    0x0D  // 频道3接收数据地址
#define RX_ADDR_P4    0x0E  // 频道4接收数据地址
#define RX_ADDR_P5    0x0F  // 频道5接收数据地址
#define TX_ADDR       0x10  // 发送地址寄存器
#define RX_PW_P0      0x11  // 接收频道0接收数据长度
#define RX_PW_P1      0x12  // 接收频道0接收数据长度
#define RX_PW_P2      0x13  // 接收频道0接收数据长度
#define RX_PW_P3      0x14  // 接收频道0接收数据长度
#define RX_PW_P4      0x15  // 接收频道0接收数据长度
#define RX_PW_P5      0x16  // 接收频道0接收数据长度
#define FIFO_STATUS   0x17  // FIFO栈入栈出状态寄存器设置
#define TX_OK         0x20  //TX发送完成中断
#define MAX_TX        0x10  //达到最大发送次数中断


extern UART_HandleTypeDef huart2;

void SPI_SCK_1();
void SPI_SCK_0();
void SPI_MOSI_1();
void SPI_MOSI_0();
void SPI_CS_1();
void SPI_CS_0();
void SPI_CE_1();
void SPI_CE_0();
GPIO_PinState SPI_READ_MISO();


void spi_delay();
void gpio_clockout_8_bits(uint8_t txData);
uint8_t gpio_clockin_8_bits();
void spi_read_register(uint8_t reg, uint8_t num_bytes, uint8_t* pbuf);
void spi_write_register(uint8_t reg, uint8_t num_bytes, uint8_t* writing_data);
// nRF24 Specific
bool spi_verified_write_register(uint8_t reg, uint8_t num_bytes, uint8_t* writing_data);
uint8_t nRF24_get_STATUS();
uint8_t nRF24_get_FIFO_STATUS();
uint8_t nRF24_get_CONFIG();
bool nRF24_tx_self_test();
void nRF24_configure_tx_mode();
void nRF24_keep_sending();











#endif /* __NRF24_H */

