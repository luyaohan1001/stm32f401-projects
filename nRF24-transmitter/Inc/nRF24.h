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
#define R_RX_PAYLOAD      0x61                      
#define W_TX_PAYLOAD      0xA0                       
#define FLUSH_TX          0xE1                       
#define FLUSH_RX          0xE2                      
#define REUSE_TX_PL       0xE3                       
#define NOP               0xFF                      

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
#define RPD           0x09  // Received Power Detector.
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
// Register 0x18 - 0x1B are not accesible, specified in datasheet.
#define DYNPD         0x1C  // Enable dynamic payload length.
#define FEATURE       0x1D  // Feature Register.

/* nRF24 Register Fields -----------------------------------------------------------------------------------------------------------------*/


/* The following masks are constructed for better readability in codes involving configurating nRF24 registers.
 * These masks are generically named as following:
 *
 *    <FieldName>_MASK<value>
 *
 *    The field names are found in nRF24 datasheet, Section 9 - "Register Map."
 *    MASK<value> means to "write", or to "mask on" this value to the target field.
 *    For example, MASK_RX_DR_MASK1 means to set 1 in MASK_TX_DR field of <CONFIG> register.
 *    The MASKDEFAULT means to set the field as its reset value shown in this chapter table.
 *
 *    Some fields are read-only. Their masks are named:
 *
 *    <FieldName>_READMASK
 *
 *    We can AND' these readmasks to full bytes read from SPI bus to extract these field value.
 */


/* Masks for <CONFIG> register*/
#define MASK_RX_DR_MASK1         1 << 6
#define MASK_RX_DR_MASK0         0
#define MASK_RX_DR_MASKDEFAULT   0

#define MASK_TX_DS_MASK1         1 << 5
#define MASK_TX_DS_MASK0         0
#define MASK_TX_DS_MASKDEFAULT   0

#define MASK_MAX_RT_MASK1        1 << 4
#define MASK_MAX_RT_MASK0        0
#define MASK_MAX_RT_MASKDEFAULT  0

#define EN_CRC_MASK1             1 << 3 
#define EN_CRC_MASK0             0
#define EN_CRC_MASKDEFAULT       1 << 3

#define CRCO_MASK1               1 << 2
#define CRCO_MASK0               0
#define CRCO_MASKDEFAULT         0

#define PWR_UP_MASK1             1 << 1
#define PWR_UP_MASK0             0
#define PWR_UP_MASKDEFAULT       0

#define PRIM_RX_MASK1            1 << 0
#define PRIM_RX_MASK0            0
#define PRIM_RX_MASKDEFAULT      0

/* EN_AA - Masks for enabling / disabling Auto Acknowledgement. */
#define ENAA_P5_MASK1          1 << 5
#define ENAA_P5_MASK0          0
#define ENAA_P5_MASKDEFAULT    1

#define ENAA_P4_MASK1          1 << 4
#define ENAA_P4_MASK0          0
#define ENAA_P4_MASKDEFAULT    1

#define ENAA_P3_MASK1          1 << 3
#define ENAA_P3_MASK0          0
#define ENAA_P3_MASKDEFAULT    1

#define ENAA_P2_MASK1          1 << 2
#define ENAA_P2_MASK0          0
#define ENAA_P2_MASKDEFAULT    1

#define ENAA_P1_MASK1          1 << 1
#define ENAA_P1_MASK0          0
#define ENAA_P1_MASKDEFAULT    1

#define ENAA_P0_MASK1          1 << 0
#define ENAA_P0_MASK0          0
#define ENAA_P0_MASKDEFAULT    1

/* Masks for EN_RXADDR - Enabling / Disabling RX addresses */
#define ERX_P5_MASK1           1 << 5
#define ERX_P5_MASK0           0
#define ERX_P5_MASKDEFAULT     0

#define ERX_P4_MASK1           1 << 4
#define ERX_P4_MASK0           0
#define ERX_P4_MASKDEFAULT     0

#define ERX_P3_MASK1           1 << 3
#define ERX_P3_MASK0           0
#define ERX_P3_MASKDEFAULT     0

#define ERX_P2_MASK1           1 << 2
#define ERX_P2_MASK0           0
#define ERX_P2_MASKDEFAULT     0

#define ERX_P1_MASK1           1 << 1
#define ERX_P1_MASK0           0
#define ERX_P1_MASKDEFAULT     0

#define ERX_P0_MASK1           1 << 0
#define ERX_P0_MASK0           0
#define ERX_P0_MASKDEFAULT     0

/* Masks for SETUP_AW - Setup of Address Widths */
#define AW_MASK3bytes         0b01 << 0
#define AW_MASK4bytes         0b10 << 0
#define AW_MASK5bytes         0b11 << 0
#define AW_MASKDEFAULT        0b11 << 0

/* Masks for SETUP_RETR - Setup of Automatic Retransmission */
#define ARD_MASK250us         0b0000 << 4
#define ARD_MASK500us         0b0001 << 4
#define ARD_MASK750us         0b0010 << 4
#define ARD_MASK1000us        0b0011 << 4
#define ARD_MASK1250us        0b0100 << 4
#define ARD_MASK1500us        0b0101 << 4
#define ARD_MASK1750us        0b0110 << 4
#define ARD_MASK2000us        0b0111 << 4
#define ARD_MASK2250us        0b1000 << 4
#define ARD_MASK2500us        0b1001 << 4
#define ARD_MASK2750us        0b1010 << 4
#define ARD_MASK3000us        0b1011 << 4
#define ARD_MASK3250us        0b1100 << 4
#define ARD_MASK3500us        0b1101 << 4
#define ARD_MASK3750us        0b1110 << 4
#define ARD_MASK4000us        0b1111 << 4
#define ARD_MASKDEFAULT       0b0000 << 4

#define ARC_MASK0         0b0000 << 0
#define ARC_MASK1         0b0001 << 0
#define ARC_MASK2         0b0010 << 0
#define ARC_MASK3         0b0011 << 0
#define ARC_MASK4         0b0100 << 0
#define ARC_MASK5         0b0101 << 0
#define ARC_MASK6         0b0110 << 0
#define ARC_MASK7         0b0111 << 0
#define ARC_MASK8         0b1000 << 0
#define ARC_MASK9         0b1001 << 0
#define ARC_MASK10        0b1010 << 0
#define ARC_MASK11        0b1011 << 0
#define ARC_MASK12        0b1100 << 0
#define ARC_MASK13        0b1101 << 0
#define ARC_MASK14        0b1110 << 0
#define ARC_MASK15        0b1111 << 0
#define ARC_MASKDEFAULT   0b0011 << 0

/* MASK for RF_CH - Modulation RF channel frequency : F0 = 2400 + RF_CH MHz*/
#define RF_CH_MASKDEFAULT 0b0000010 << 0

/* Mask for RF_SETUP - Settings on RF parameters, data rate, etc */
#define CONT_WAVE_MASK1        1 << 7
#define CONT_WAVE_MASK0        0
#define CONT_WAVE_MASKDEFAULT  0

#define RF_DR_LOW_MASK1        1 << 5
#define RF_DR_LOW_MASK0        0
#define RF_DR_LOW_MASKDEFAULT  0

#define RF_DR_HIGH_MASK1       1 << 3
#define RF_DR_HIGH_MASK0       0
#define RF_DR_HIGH_MASKDEFAULT 1

/* See decrption for RF_SETUP in Section 9, "Register Map", RF_SETUP, Encoding [RF_DR_LOW, RF_DR_HIGH] */
#define RF_DR_MASK1Mbps   (RF_DR_LOW_MASK0 | RF_DR_HIGH_MASK0)
#define RF_DR_MASK2Mbps   (RF_DR_LOW_MASK0 | RF_DR_HIGH_MASK1)
#define RF_DR_MASK250kbps (RF_DR_LOW_MASK1 | RF_DR_HIGH_MASK0)


#define RF_PWR_MASKNEG18dBm     0b00 << 1
#define RF_PWR_MASKNEG12dBm     0b01 << 1
#define RF_PWR_MASKNEG6dBm      0b10 << 1
#define RF_PWR_MASKNEG0dBm      0b11 << 1
#define RF_PWR_MASKDEFAULT      0b11 << 1

/* Masks for STATUS -  Register on nRF24 chip status and bits for clearning */
#define RX_DR_MASK1      1 << 6  // Write 1 to clear. No write 0 operation.
#define RX_DR_READMASK   1 << 6  //  Used for reading this bit.
#define TX_DS_MASK1      1 << 5  // Write 1 to clear. No write 0 operation.
#define TX_DS_READMASK   1 << 5  // Used for reading this bit.

#define MAX_RT_MASK1     1 << 4 // Write 1 to clear. No write 0 operation. 
#define MAX_RT_READMASK  1 << 4 // Used for reading this bit. 

#define RX_P_NO_READMASK    0b111 << 1 // Used for reading data pipe number.

#define TX_FULL_READMASK    1 << 0 // Used for reading TX FULL flag. 

/* Masks for Transmit Observe Register */ 
#define PLOS_CNT_READMASK  0b1111 << 4
#define ARC_CNT_READMASK   0b1111 << 0

/* Masks for reading FIFO_STATUS - Status on FIFO */
#define TX_REUSE_READMASK  1 << 6
// #define TX_FULL_READMASK   1 << 5  // Same as the TX_FULL field in STATUS register.
#define TX_EMPTY_READMASK  1 << 4
#define RX_FULL_READMASK   1 << 1
#define RX_EMPTY_READMASK  1 << 0

/* Function Prototypes -------------------------------------------------------------------------------------------------------*/

/* @brief husart2 Handler to UART2 peripherals on STM32F401RE */
extern UART_HandleTypeDef huart2;

/**
  * @brief Following functions operates GPIO pins on the SPI bus.
  */
void SPI_SCK_1();
void SPI_SCK_0();
void SPI_MOSI_1();
void SPI_MOSI_0();
void SPI_CS_1();
void SPI_CS_0();
GPIO_PinState SPI_READ_MISO();
void spi_delay();
void gpio_clockout_8_bits(uint8_t tx_data);
uint8_t gpio_clockin_8_bits();

/**
	* @brief utility used for printing debug message through serial UART.
	*/
void serial_print(char* message);

/**
  * @brief Following functions does spi operations.
  */
void spi_read_register(uint8_t reg, uint8_t num_bytes, uint8_t* p_read_data);
void spi_write_register(uint8_t reg, uint8_t num_bytes, uint8_t* p_writing_data);


/**
  * @brief Following functions are nRF24-specific operations.
  */
void nRF24_CE_1();
void nRF24_CE_0();
bool nRF24_verified_write_register(uint8_t reg, uint8_t num_bytes, uint8_t* p_writing_data);
uint8_t nRF24_get_STATUS();
uint8_t nRF24_get_FIFO_STATUS();
uint8_t nRF24_get_CONFIG();
bool nRF24_tx_self_test();
void nRF24_configure_tx_mode();
void nRF24_keep_sending();
void nRF24_print_all_registers();
#endif /* __NRF24_H */

