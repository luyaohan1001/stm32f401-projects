/**
  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********
  * @file      :     nRF24.h
  * @author    :     Luyao Han
  * @email     :     luyaohan1001@gmail.com
  * @brief     :     C library header for Nordic nRF24L01 2.4GHz wireless transceiver.
  * @date      :     04-21-2022
  * Copyright (C) 2022-2122 Luyao Han. The following code may be shared or modified for personal use / non-commercial use only.
  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********  */

/* Macro to prevent recursive inclusion ----------------------------------------------------------------------------------------*/
#ifndef __NRF24_H
#define __NRF24_H

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>


/* nRF24 SPI Commands ----------------------------------------------------------------------------------------------------------*/
#define R_REGISTER_MASK   0x00                   
#define W_REGISTER_MASK   0x20
#define R_RX_PAYLOAD  0x61                      
#define W_TX_PAYLOAD  0xA0                       
#define FLUSH_TX     0xE1                       
#define FLUSH_RX     0xE2                      
#define REUSE_TX_PL  0xE3                       
#define NOP          0xFF                      

/* nRF24 On-device Registers ----------------------------------------------------------------------------------------------------*/
#define CONFIG        0x00  // Configuration registers.
#define EN_AA         0x01  // Enable Auto Acknowledge.
#define EN_RXADDR     0x02  // Enable RX Addresses.
#define SETUP_AW      0x03  // Setup of Address Widths.
#define SETUP_RETR    0x04  // Setup of Automatic Retransmission.
#define RF_CH         0x05  // RF Channel.
#define RF_SETUP      0x06  // RF Setup Register.
#define STATUS        0x07  // Status Register.
#define OBSERVE_TX    0x08  // Transmit Lost Register.
#define RPD            0x09 // Received Power Detector.
#define RX_ADDR_P0    0x0A  // Receive address data pipe 0.
#define RX_ADDR_P1    0x0B  // Receive address data pipe 1.
#define RX_ADDR_P2    0x0C  // Receive address data pipe 2.
#define RX_ADDR_P3    0x0D  // Receive address data pipe 3.
#define RX_ADDR_P4    0x0E  // Receive address data pipe 4.
#define RX_ADDR_P5    0x0F  // Receive address data pipe 5.
#define TX_ADDR       0x10  // Transmit address.
#define RX_PW_P0      0x11  // Number of bytes in RX payload in data pipe 0.
#define RX_PW_P1      0x12  // Number of bytes in RX payload in data pipe 1.
#define RX_PW_P2      0x13  // Number of bytes in RX payload in data pipe 2.
#define RX_PW_P3      0x14  // Number of bytes in RX payload in data pipe 3.
#define RX_PW_P4      0x15  // Number of bytes in RX payload in data pipe 4.
#define RX_PW_P5      0x16  // Number of bytes in RX payload in data pipe 5.
#define FIFO_STATUS   0x17  // FIFO Status Register.

extern UART_HandleTypeDef huart2;


void SPI_SCK_1();
void SPI_SCK_0();
void SPI_MOSI_1();
void SPI_MOSI_0();
void SPI_CS_1();
void SPI_CS_0();

GPIO_PinState SPI_READ_MISO();


void nRF24_CE_1();
void nRF24_CE_0();


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

