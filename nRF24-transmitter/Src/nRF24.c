/**
  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********
  * @file      :     nRF24.c
  * @author    :     Luyao Han
  * @email     :     luyaohan1001@gmail.com
  * @brief     :     C library for Nordic nRF24L01+ (or nRF24L01p) 2.4GHz wireless transceiver.
  * @date      :     04-21-2022
	* @note      :     The library nRF24.c and nRF24.h strictly follows the "nRF24L01+ Single Chip 2.4GHz Transceiver Product Specification v1.0" released by NORDIC SEMICONDUCTOR in 2008.
	                   Name Conventions:
											   <REGISTER>
												 high-level actions uses more snake cases, while lower-level GPIO / Delays uses more capitalized letter since they are closer to hardware.
  * Copyright (C) 2022-2122 Luyao Han. The following code may be shared or modified for personal use / non-commercial use only.
  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********  */

/* Includes ---------------------------------------------------------------------------------------------------------------------------------------*/
#include "nRF24.h"

/* Macro Define ---------------------------------------------------------------------------------------------------------------------------------------*/
#define NRF24_DEBUG /* When defined, debug messages are logged through UART. */
/* GPIO Physical Layer --------------------------------------------------------------------------------------------------------------------------------*/

/* SCK    PA8  */
/* MOSI   PB10 */
/* CSN    PB4  */
/* CE     PB5  */
/* MISO   PA10 */

/**
  * @brief Set high on SCK pin of SPI bus.
  * @param None
  * @retval None */
__inline__ void SPI_SCK_1()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  
}

/**
  * @brief Set low on SCK pin of SPI bus.
  * @param None
  * @retval None
  */
__inline__ void SPI_SCK_0()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  
} 

/**
  * @brief Set high on MOSI pin of SPI bus.
  * @param None
  * @retval None
  */
__inline__ void SPI_MOSI_1()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  
}

/**
  * @brief Set low on MOSI pin of SPI bus.
  * @param None
  * @retval None
  */
__inline__ void SPI_MOSI_0()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  
}

/**
  * @brief Set high on CS pin of SPI bus.
  * @param None
  * @retval None
  */
__inline__ void SPI_CS_1() 
{
    /* CS High == CSN Low */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  
}

/**
  * @brief Set low on CS pin of SPI bus.
  * @param None
  * @retval None
  */
__inline__ void SPI_CS_0()
{
    /* CS Low == CSN High */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
}


/**
  * @brief  Get pin-level on MISO pin of SPI bus.
  * @param  None
  * @retval None
  */
__inline__ GPIO_PinState SPI_READ_MISO()
{
  return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
}

/**
  * @brief   Print debug string through USART.
  * @param   p_msg Pointer to anynomous message string.
  * @retval  None.
  * @note    When testing on STM32F401RE Nucleo Board, the board supports virtual COM (serial) port through USB.
  *            Connecting a USB-TTL adapter such as CH340 to the 'TX/D1' on morpho connector will not receive data.
  *             In the datasheet it has been confirmed that the USART2 pins have been to multiplexed for the virtual COM feature.
  *            On the PC, look for port /dev/ttyACM0 as the virtual serial port.  
  */
__inline__ void serial_print(char* message)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
}

/**
  * @brief      Clock out (write) 8 bits on MOSI of SPI bus on SCK high.
  * @param[in]  tx_data One byte of data to transmit.
  * @retval     None.
  * @note  
  *   Endianess: Most Significant Bit First. Cn: Command Bits. Sn: Status Register bits. Dn: data bits.
  *
  *   Following is a diagram for time sequence:
  *   CSN ````\__________________________________________________________________________________________________/````````
  *   MOSI______|C7|__|C6|__|C5|__|C4|__|C3|__|C2|__|C1|__|C0|______|D7|__|D6|__|D5|__|D4|__|D3|__|D2|__|D1|__|D0|
  *             ^     ^     ^     ^     ^     ^     ^     ^         ^     ^     ^     ^     ^     ^     ^     ^
  *   SCK ______/``\__/``\__/``\__/``\__/``\__/``\__/``\__/``\______/``\__/``\__/``\__/``\__/``\__/``\__/``\__/``\________
  *   MISO______|S7|__|S6|__|S5|__|S4|__|S3|__|S2|__|S1|__|S0|______XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX________
  *
  *   Pulse#     1     2     3     4     5     6     7     8         9     10    11    12    13    14   15     16
  */
void gpio_clockout_8_bits(uint8_t tx_data) 
{
  spi_delay();
  for (int i = 0; i < 8; ++i) 
  {
      SPI_SCK_0();
      spi_delay();

      if(tx_data & 0x80) // MSBit first
          SPI_MOSI_1();
      else
          SPI_MOSI_0();

			/* setup time */	
      spi_delay();

      SPI_SCK_1(); // clock data
      tx_data = tx_data << 1; // load next MSB

			/* hold time */
      spi_delay();
  }
  SPI_SCK_0();
}

/**
  * @brief  Clock in (read) 8 bits from MISO of SPI bus on SCK high.
  * @param  None
  * @retval A byte of read data.
  * @note
  *   Endianess: Most Significant Bit first. Cn: Command bits. Sn: Status register bits. Dn: Data bits.
  *    
  *   Following is a diagram for time sequence:
  *   CSN ````\___________________________________________________________________________________________________/```````
  *   MOSI______|C7|__|C6|__|C5|__|C4|__|C3|__|C2|__|C1|__|C0|______|00|__|00|__|00|__|00|__|00|__|00|__|00|__|00|
  *             ^     ^     ^     ^     ^     ^     ^     ^         ^     ^     ^     ^     ^     ^     ^     ^
  *   SCK ______/``\__/``\__/``\__/``\__/``\__/``\__/``\__/``\______/``\__/``\__/``\__/``\__/``\__/``\__/``\__/``\________
  *   MISO______|S7|__|S6|__|S5|__|S4|__|S3|__|S2|__|S1|__|S0|______|D0|__|D1|__|D2|__|D3|__|D4|__|D5|__|D6|__|D7|________
  *   Pulse#     1     2     3     4     5     6     7     8         9     10    11    12    13    14   15     16
  */
uint8_t gpio_clockin_8_bits(void)
{
  uint8_t rx_data = 0;

  spi_delay();
  for (int i=0; i < 8; ++i) 
  {
      SPI_SCK_0();
      spi_delay();
      SPI_MOSI_0();

			/* setup time */
      SPI_SCK_1();
      spi_delay();
      rx_data = rx_data << 1; // Why shift first then OR'? range (0, 8) will need to shift only 7 times.
      rx_data |= SPI_READ_MISO();

			/* hold time */
      spi_delay();
  }

  SPI_SCK_0();
  return rx_data;
}
  

/* SPI Datalink Layer------------------------------------------------------------------------------------------------------------------------------------*/
void spi_delay() 
{
  HAL_Delay(1);
}


/**
  * @brief      Read data from the SPI target device register. Endianess: LSByte first.
  * @param[in]  reg SPI target device register to write to.
  * @param[in]  num_bytes Number of bytes needed to write to that address.
  * @param[out] p_read_data A pointer pointing to a memory location that can store the data read from the SPI device.
  * @retval     none.
  */
void spi_read_register(uint8_t reg, uint8_t num_bytes, uint8_t* p_read_data)
{
  /* SPI CHIP SELECT */
  SPI_CS_1();
  
  /* Clock out target register. */
  gpio_clockout_8_bits(reg);

  /* Clock in target register value. */
  for (int i = 0; i < num_bytes; ++i) 
  {
    p_read_data[i] = gpio_clockin_8_bits();
  }
  
  /* SPI CHIP DESELECT */
  SPI_CS_0();
}


/**
  * @brief      Write a number of bytes to the spi target device register.
  * @param[in]  reg spi target device register to write to.
  * @param[in]  num_bytes number of bytes needed to write to that address.
  * @param[in]  p_writing_data A pointer pointing to a memory location storing the data to write.
  * @retval     none.
  */
void spi_write_register(uint8_t reg, uint8_t num_bytes, uint8_t* p_writing_data)
{
  /* SPI CHIP SELECT */
  SPI_CS_1();

  /* Clock out target register. */
  gpio_clockout_8_bits(reg); 

  /* Clock out value bytes to the target. */
  for (int i = 0; i < num_bytes; ++i)
  {
    uint8_t writing_byte = p_writing_data[i];
    gpio_clockout_8_bits(writing_byte);
  }

  /* SPI CHIP DESELECT */
  SPI_CS_0();
}


/* nRF24 Operations -----------------------------------------------------------------------------------------------------------------------------------*/


/**
  * @brief Set high on Chip-Enable pin of nRF24L01.
  * @param None
  * @retval None
  */
void nRF24_CE_1()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);  
}


/**
  * @brief Set low on Chip-Enable pin of nRF24L01.
  * @param None
  * @retval None
  */
void nRF24_CE_0()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

/**
  * @brief      Write to a register on nRF24L01+ through SPI. Read the same registers after write to confirm that the write has been successful.
  * @param[in]  reg The target register to write value to.
  * @param[in]  num_bytes Number of bytes to write.
  * @param[in]  p_writing_data Data to write.
  * @retval     Boolean. 1 for mistakes happen. 0 for success.
  * @note       reg & ~ W_REGISTER_MASK is reverse operation of reg | W_REGISTER_MASK,
	*               essentially get rid of Write Regiter Mask and add a Read Register Mask. 
  */
bool nRF24_verified_write_register(uint8_t reg, uint8_t num_bytes, uint8_t* p_writing_data)
{

    char message[64] = {'\0'};
    uint8_t read_data[num_bytes];

		/* Write the data to target register. */
    spi_write_register(reg, num_bytes, p_writing_data); 

		/* Read from same target register to verify if data has been successfully written. */
    spi_read_register(R_REGISTER_MASK | (reg & ~W_REGISTER_MASK), num_bytes, read_data);

    for (int i = 0; i < num_bytes; ++i) 
    {
      if (read_data[i] != p_writing_data[i])  /* check mismatch between written data and read data from target register. */
      {
				#ifdef NRF24_DEBUG
        strcpy(message, "Problem writing to nRF24 register -- ");
        serial_print(message);
        sprintf(message, "writing data: <%#02x> read_data: <%#02x>\n", p_writing_data[i], read_data[i]);
        serial_print(message);
				#endif 
        return true;
      } else {
				#ifdef NRF24_DEBUG
        strcpy(message, "Success writing to nRF24 register -- ");
        serial_print(message);
        sprintf(message, "writing data: <%#02x> read_data: <%#02x>\n", p_writing_data[i], read_data[i]);
        serial_print(message);
				#endif
      }
  }
  return false;
}


/**  
 *  @brief  MVP: Minial Viable Test. Test nRF24 transmitter function without a receiver. Use this function on a fresh setup as sanity check.
 *  @param  None.
 *  @retval exit status. 0 success, 1 failure.
 *  @note  
 *      Steps: 1. Disable Auto Acknowledgement, disable Auto Retransmit.
 *             2. TX_DS (in STATUS register) is expected to be set when data has been clock into TX FIFO is set.
 *             3. Check if STATUS = 0x2E, if so we have a working TX module.
 *            
 *            After writing to W_TX_PAYLOAD, TX_EMPTY (in FIFO_STATUS register) becomes 0.
 *            
 *            What happens if sending is not successful?
 *            TX_FULL (in FIFO_STATUS register) becomes 1.
 *            TX_FULL (in STATUS register) becomes 1.
 *            TX_DS (in STATUS register) remains 0.  
 *
 *      States: 
 *            The states can be referred in 6.1.1 State diagram.
 *      
 *      STATUS register
 *                7             6           5         4         3:1         0
 *            <reserved>      RX_DR       TX_DS     MAX_RT     RX_P_NO     TX_FULL
 *            -----------------------------------------------------------------------
 *             Always 0       Receive     Transfer  Maximum    000-101 :    1: 
 *                            Data        Data      TX         Data Pipe #  TX_FULL
 *                            Ready       Sent      Transmits  110 :
 *                                                             Not Used.
 *                                                             111:
 *                                                             RX FIFO Empty.
 */
bool nRF24_mvt_tx_self_test() 
{
	char message[128];
	uint8_t writing_byte;

  #ifdef NRF24_DEBUG
  strcpy(message, "---- nrf24 tx self test. ----\n");
  serial_print(message);

  strcpy(message, "---- This test to verifies function of a tranmitter send without a receiver. ----\n");
  serial_print(message);
  #endif

  uint8_t nRF24_status = 0x00;

  // Note that if we reset the nRF connected without re-poweron the chip, initial value of registers such as STATUS or CONFIG may be different from one listed in datasheet.
  /* Current State: [Power-on reset 100 ms] */
  nRF24_CE_0();

  /* Current State: [Power-Down] (RF transmission is Power-Down, but SPI is alive.) */
  writing_byte = 0x00;
  nRF24_verified_write_register(W_REGISTER_MASK + EN_AA, 1, &writing_byte);        // disable auto acknowledgement  
  nRF24_verified_write_register(W_REGISTER_MASK + EN_RXADDR, 1, &writing_byte);    // disable RX data pipes
  nRF24_verified_write_register(W_REGISTER_MASK + SETUP_RETR, 1, &writing_byte);   // disable automatic re-transmit, ARC = 0000

  writing_byte = 0x0E;
  nRF24_verified_write_register(W_REGISTER_MASK + CONFIG, 1, &writing_byte);       // PWR_UP = 1 PRIMRX=0 (TX mode)

  /* PWR_UP=1, state transition to [Standby-I] */
  uint8_t test_payload[4] = {0xC0, 0xFE, 0xBE, 0xEF}; // clock in a payload, now TX FIFO not empty 
  spi_write_register(W_TX_PAYLOAD, 4, test_payload);
  nRF24_CE_1(); // Chip Enable. Fire the packet out on the antenna!
  
  /* TX FIFO not empty AND CE = 1, state transition to [TX MODE] */
	/* Get nRF24 <STATUS> register value. */
  spi_read_register(R_REGISTER_MASK + STATUS, 1, &nRF24_status);

  spi_delay(1);

  /* CE=0, state transition -> now return to [Standby-I]. */
  nRF24_CE_0();

  /* PWR_UP = 0, state transition to [Power Down] */
  writing_byte = 0x08; // write default value for CONFIG register (writing_byte = 0)
  nRF24_verified_write_register(W_REGISTER_MASK + CONFIG, 1, &writing_byte);       

  /* Now the chip is back to power down mode, check test result. */
  if (nRF24_status & 0x2E) 
  {
		#ifdef NRF24_DEBUG
    strcpy(message, "\n > nRF24 transmission self-test has passed. \
                       <STATUS> has value of 0x2E. \
                       TX_DS (transfer data sent) was set. \
                       RX_P_NO = 111, this means RX FIFO Empty. \n");
    serial_print(message);
    #endif
    return 0;
  } 
  else 
  {
    #ifdef NRF24_DEBUG
    sprintf(message, "\n > nRF24 transmission self-test has failed. <STATUS> is expected 0x2E. Current value: %#02x\n", nRF24_status);
    serial_print(message);
    #endif
    return 1;
  }
  
}

/**
  * @brief  MVT: Minial Viable Test. Configure nRF24 to work in TX (transmit) mode.
  * @param  None.
  * @retval None.
  * @note   After nRF24_mvt_configure_tx_mode() is called, use nRF24_mvt_keep_sending() to keep sending data.
  */
void nRF24_mvt_configure_tx_mode() 
{
    nRF24_CE_0();

    /* Set TX_ADDR for sender. On the Receiver side, set RX_ADDR_P0 with same value. */

    uint8_t TX_ADDRESS[5] = {0x10,0xDE,0x10,0x10,0x10};  // 5 byte transmit-address
    spi_write_register(W_REGISTER_MASK + TX_ADDR, 5, TX_ADDRESS);     // Write transmit-address to nRF24

    uint8_t writing_byte;

    writing_byte = 0x00;
    nRF24_verified_write_register(W_REGISTER_MASK + EN_AA, 1, &writing_byte);

    writing_byte = 0x00;
    nRF24_verified_write_register(W_REGISTER_MASK + EN_RXADDR, 1, &writing_byte);

    writing_byte = 0x00;
    nRF24_verified_write_register(W_REGISTER_MASK + SETUP_RETR, 1, &writing_byte);

    writing_byte = 40;
    nRF24_verified_write_register(W_REGISTER_MASK + RF_CH, 1, &writing_byte);

    writing_byte = 0x07;
    nRF24_verified_write_register(W_REGISTER_MASK + RF_SETUP, 1, &writing_byte);

    // PWR_UP, state transition to [Standby-I]
    writing_byte = 0x0e;
    nRF24_verified_write_register(W_REGISTER_MASK + CONFIG, 1, &writing_byte);
    spi_delay(150);

    // CE is not set to 1, nRF24 still stays in [Standby-I] Mode.
    // CE = 1 is not activated until we write to TX FIFO so stays in Standby-I mode.
}


/**
  * @brief  Make nRF24 keep sending data.
  * @param  None.
  * @retval None.
	* @note   Call nRF24_mvt_configure_tx_mode() before this test to initailize TX mode in nRF24L01+.
  */
void nRF24_mvt_keep_sending() 
{
  uint8_t payload[] = {0xBE, 0xEF, 0xCA, 0xFE}; /* clock out a four-byte payload "BEEF CAFE" */

  char debug_msg[64];
  
  spi_write_register(W_TX_PAYLOAD, 4, (uint8_t*) payload);

  /* Fire out the transmit packet */
  nRF24_CE_1(); 

	/* read <STATUS> register*/
  uint8_t status;
  spi_read_register(R_REGISTER_MASK + STATUS, 1, &status);

  #ifdef NRF24_DEBUG
  sprintf(debug_msg, "<STATUS> REGISTER : %#02x\n", status);
  serial_print(debug_msg);
  #endif

  if (status == 0x2e) /* TX_DS bit is set. */
  {
    #ifdef NRF24_DEBUG
    strcpy(debug_msg, "nRF24 mvt send status - success -\n");
    serial_print(debug_msg);
    #endif
  } 
  else 
  {
    #ifdef NRF24_DEBUG
    strcpy(debug_msg, "nRF24 mvt send status - failure -\n");
    serial_print(debug_msg);
    #endif
  }
  /* write 1 to clear TX_DS, TX_DS bit is Write-to-Clear. */
  uint8_t writing_byte = 0x20;
  spi_write_register(W_REGISTER_MASK + STATUS, 1, &writing_byte); 

  nRF24_CE_0(); /* stop transmission. Returns to [Standby-I]. */
}



/**
  * @brief  Print all registers on nRF24. 
  * @param  None.
  * @retval None.
  */
void nRF24_print_all_registers()
{
	  uint8_t read_data;
		uint8_t read_buf[4];
		char message[32];
  
    spi_read_register(R_REGISTER_MASK + CONFIG, 1, &read_data);
		sprintf(message, "CONFIG: <0x%02x>\n", read_data);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + EN_AA, 1, &read_data);
		sprintf(message, "EN_AA: <0x%02x>\n", read_data);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + EN_RXADDR, 1, &read_data);
		sprintf(message, "EN_RXADDR: <0x%02x>\n", read_data);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + SETUP_AW, 1, &read_data);
		sprintf(message, "SETUP_AW: <0x%02x>\n", read_data);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + SETUP_RETR, 1, &read_data);
		sprintf(message, "SETUP_RETR: <0x%02x>\n", read_data);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + RF_CH, 1, &read_data);
		sprintf(message, "RF_CH: <0x%02x>\n", read_data);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + RF_SETUP, 1, &read_data);
		sprintf(message, "RF_SETUP: <0x%02x>\n", read_data);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + STATUS, 1, &read_data);
		sprintf(message, "STATUS: <0x%02x>\n", read_data);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + RX_ADDR_P0, 4, read_buf);
		sprintf(message, "RX_ADDR_P0: %#02x %#02x %#02x %#02x>\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + RX_ADDR_P1, 4, read_buf);
		sprintf(message, "RX_ADDR_P1: %#02x %#02x %#02x %#02x>\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + RX_ADDR_P2, 4, read_buf);
		sprintf(message, "RX_ADDR_P2: %#02x %#02x %#02x %#02x>\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + RX_ADDR_P3, 4, read_buf);
		sprintf(message, "RX_ADDR_P3: %#02x %#02x %#02x %#02x>\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + RX_ADDR_P4, 4, read_buf);
		sprintf(message, "RX_ADDR_P4: %#02x %#02x %#02x %#02x>\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + RX_PW_P5, 4, read_buf);
		sprintf(message, "RX_ADDR_P5: %#02x %#02x %#02x %#02x>\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
    serial_print(message);

  	spi_read_register(R_REGISTER_MASK + TX_ADDR, 4, read_buf);
		sprintf(message, "TX_ADDR: %#02x %#02x %#02x %#02x>\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
    serial_print(message);

}

/* Following Function Provides High Level Access to nRF24L01+ registers. -----------------------------------------------------------------------------------*/
/* Instead of using spi_write_register() or nRF24_verified_write_register(). Following in configure function renders better code readability -----------*/

/**
	* @brief  nRF24 set <CONFIG> register
	* @param  mask_rx_dr Mask interrupt caused by RX_IDR. [ MASK_RX_DR_MASK1 | MASK_RX_DR_MASK0 | MASK_RX_DR_MASKDEFAULT ]
	* @param  mask_tx_ds Mask interrupt caused by TX_DS.  [ MASK_TX_DS_MASK1 | MASK_TX_DS_MASK0 | MASK_TX_DS_MASKDEFAULT ] 
	* @param  mask_max_rt Mask interrupt cause by MAX_RT. [ MASK_MAX_RT_MASK1 | MASK_MAX_RT_MASK0 | MASK_MAX_RT_MASKDEFAULT  ]
	* @param  en_crc Enable CRC. [ EN_CRC_MASK1 | EN_CRC_MASK0 | EN_CRC_MASKDEFAULT ]
	* @param  crco CRC encoding scheme. [ CRCO_MASK1 | CRCO_MASK0 | CRCO_MASKDEFAULT ]
	* @param  pwr_up Power Up / Down. [ PWR_UP_MASK1 | PWR_UP_MASK0 | PWR_UP_MASKDEFAULT ]
	* @param  prim_rx RX/TX control. [ PRIM_RX_MASK1 | PRIM_RX_MASK0 | PRIM_RX_MASKDEFAULT ]
	* @retval None.
	*/
void nRF24_set_CONFIG(uint8_t mask_rx_dr, uint8_t mask_tx_ds, uint8_t mask_max_rt, uint8_t en_crc, uint8_t crco, uint8_t pwr_up, uint8_t prim_rx) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= mask_rx_dr | mask_tx_ds | mask_max_rt | en_crc | crco | pwr_up | prim_rx; 
  nRF24_verified_write_register(W_REGISTER_MASK + CONFIG, 1, &writing_byte);
}


/**
  * @brief      Read <CONFIG> register from nRF24L01+.
  * @param      None.
  * @retval     <CONFIG> register value.
  */
uint8_t nRF24_get_CONFIG() 
{
  uint8_t config_reg;
  spi_read_register(R_REGISTER_MASK + CONFIG, 1, &config_reg);
  return config_reg;
}


/**
	* @brief  nRF24 set <EN_AA> register
	* @param  enaa_p5 Enable auto acknowledgement in data pipe 5. [ ENAA_P5_MASK1 | ENAA_P5_MASK0 | ENAA_P5_MASKDEFAULT ] 
	* @param  enaa_p4 Enable auto acknowledgement in data pipe 4. [ ENAA_P4_MASK1 | ENAA_P4_MASK0 | ENAA_P4_MASKDEFAULT ] 
	* @param  enaa_p3 Enable auto acknowledgement in data pipe 3. [ ENAA_P3_MASK1 | ENAA_P3_MASK0 | ENAA_P3_MASKDEFAULT ] 
	* @param  enaa_p2 Enable auto acknowledgement in data pipe 2. [ ENAA_P2_MASK1 | ENAA_P2_MASK0 | ENAA_P2_MASKDEFAULT ] 
	* @param  enaa_p1 Enable auto acknowledgement in data pipe 1. [ ENAA_P1_MASK1 | ENAA_P1_MASK0 | ENAA_P1_MASKDEFAULT ] 
	* @param  enaa_p0 Enable auto acknowledgement in data pipe 0. [ ENAA_P0_MASK1 | ENAA_P0_MASK0 | ENAA_P0_MASKDEFAULT ] 
	* @retval None.
	*/
void nRF24_set_EN_AA(uint8_t enaa_p5, uint8_t enaa_p4, uint8_t enaa_p3, uint8_t enaa_p2, uint8_t enaa_p1, uint8_t enaa_p0) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= enaa_p5 | enaa_p4 | enaa_p3 | enaa_p2 | enaa_p1 | enaa_p0;
  nRF24_verified_write_register(W_REGISTER_MASK + EN_AA, 1, &writing_byte);
}

/**
  * @brief      Read <EN_AA> register from nRF24L01+.
  * @param      None.
  * @retval     <EN_AA> register value.
  */
uint8_t nRF24_get_EN_AA() 
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + EN_AA, 1, &read_byte);
  return read_byte;
}


/**
	* @brief  nRF24 set <EN_RXADDR> register
	* @param  erx_p5 enable RX in data pipe 5.   [ ERX_P5_MASK1 | ERX_P5_MASK0 | ERX_P5_MASKDEFAULT ] 
	* @param  erx_p4 enable RX in data pipe 4.   [ ERX_P4_MASK1 | ERX_P4_MASK0 | ERX_P4_MASKDEFAULT ] 
	* @param  erx_p3 enable RX in data pipe 3.   [ ERX_P3_MASK1 | ERX_P3_MASK0 | ERX_P3_MASKDEFAULT ] 
	* @param  erx_p2 enable RX in data pipe 2.   [ ERX_P2_MASK1 | ERX_P2_MASK0 | ERX_P2_MASKDEFAULT ] 
	* @param  erx_p1 enable RX in data pipe 1.   [ ERX_P1_MASK1 | ERX_P1_MASK0 | ERX_P1_MASKDEFAULT ] 
	* @param  erx_p0 enable RX in data pipe 0.   [ ERX_P0_MASK1 | ERX_P0_MASK0 | ERX_P0_MASKDEFAULT ] 
	* @retval None.
	*/
void nRF24_set_EN_RXADDR(uint8_t erx_p5, uint8_t erx_p4, uint8_t erx_p3, uint8_t erx_p2, uint8_t erx_p1, uint8_t erx_p0) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= erx_p5 | erx_p4 | erx_p3 | erx_p2 | erx_p1 | erx_p0;
  nRF24_verified_write_register(W_REGISTER_MASK + EN_RXADDR, 1, &writing_byte);
}


/**
  * @brief      Read <EN_RXADDR> register from nRF24L01+.
  * @param      None.
  * @retval     <EN_RXADDR> register value.
  */
uint8_t nRF24_get_EN_RXADDR() 
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + EN_RXADDR, 1, &read_byte);
  return read_byte;
}

/**
	* @brief  nRF24 set <AW> register
	* @param  aw Setup of Address Widths. [ AW_MASK3bytes | AW_MASK4bytes | AW_MASK5bytes | AW_MASKDEFAULT ]
	* @retval None.
	*/
void nRF24_set_SETUP_AW(uint8_t aw) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= aw;
  nRF24_verified_write_register(W_REGISTER_MASK + SETUP_AW, 1, &writing_byte);
}

/**
  * @brief      Read <SETUP_AW> register from nRF24L01+.
  * @param      None.
  * @retval     <SETUP_AW> register value.
  */
uint8_t nRF24_get_SETUP_AW() 
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + SETUP_AW, 1, &read_byte);
  return read_byte;
}

/**
	* @brief  nRF24 configure auto-retransmit delay and auto retransmit count.
	* @param  ard Auto-Retransmit Delay
  *   [ ARD_MASK250us  | ARD_MASK500us  | ARD_MASK750us  | ARD_MASK1000us | ARD_MASK1250us        
  *   | ARD_MASK1500us | ARD_MASK1750us | ARD_MASK2000us | ARD_MASK2250us | ARD_MASK2500us        
  *   | ARD_MASK2750us | ARD_MASK3000us | ARD_MASK3250us | ARD_MASK3500us | ARD_MASK3750us        
  *   | ARD_MASK4000us | ARD_MASKDEFAULT ]
	* @param  arc Auto-Retransmit Count
  *		[	ARC_MASK0 | ARC_MASK1 | ARC_MASK2  | ARC_MASK3  | ARC_MASK4  | ARC_MASK5   | ARC_MASK6  | ARC_MASK7 
	*   | ARC_MASK8 | ARC_MASK9 | ARC_MASK10 | ARC_MASK11 | ARC_MASK12 |	ARC_MASK13 | ARC_MASK14 | ARC_MASK15 
	*   | ARC_MASKDEFAULT ]
	* @retval None.
	*/
void nRF24_set_SETUP_RETR(uint8_t ard, uint8_t arc)
{
  uint8_t writing_byte = 0x00;
	writing_byte |= ard | arc;
  nRF24_verified_write_register(W_REGISTER_MASK + SETUP_RETR, 1, &writing_byte);
}


uint8_t nRF24_get_SETUP_RETR(uint8_t ARD, uint8_t ARC)
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + SETUP_RETR, 1, &read_byte);
  return read_byte;
}



/**
	* @brief  nRF24 sets the frequency channel of nRF24L01+ operates on.
	* @param  ch Channel. It is basically frequency in MHz. [ RF_CH_MASKDEFAULT ]
	* @retval None.
	* @note   frequency = (2400 + ch) MHz
	*/
void nRF24_set_RF_CH(uint8_t ch)
{
  uint8_t writing_byte = 0x00;
	writing_byte |= ch;
  nRF24_verified_write_register(W_REGISTER_MASK + RF_CH, 1, &writing_byte);
}

uint8_t nRF24_get_RF_CH()
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + RF_CH, 1, &read_byte);
  return read_byte;
}

/**
	* @brief nRF24 RF SETUP register configuration. 
	* @param cont_wave Enables continuous carrier transmit when high. [ CONT_WAVE_MASK1 | CONT_WAVE_MASK0 | CONT_WAVE_MASKDEFAULT ]
	* @param rf_dr_low Sets RF data rate to 250 kbps. [ RF_DR_LOW_MASK1 | RF_DR_LOW_MASK0 | RF_DR_LOW_MASKDEFAULT ]
	* @param pll_lock (!) TEST-ONLY SIGNAL for Nordic's internal testing purpose. Forces PLL lock signal.
	* @param rf_dr_high Select between high speed data rates. This bit is don't care if RF_DR_LOW bit is set.
	*  
	*    [ RF_DR_HIGH_MASK1 | RF_DR_HIGH_MASK0 | RF_DR_HIGH_MASKDEFAULT ]
	*
	* @note Use RF_DR_HIGH_MASKx and RF_DR_LOW_MASKx or you can use the following masks that combines the two:
	*
	*     Encoding for RF data rate: 
	*     {RF_DR_LOW, RF_DR_HIGH}   Data-Rate
	*          0    ,     0            1   Mbps
	*          0    ,     1            2   Mbps
	*          1    ,     0            250 kbps
	*          1    ,     1            Reserved
  *  
	*    [ RF_DR_MASK1Mbps | RF_DR_MASK2Mbps | RF_DR_MASK250kbps ]
  *
	* @param rf_pwr RF Output power in TX mode. [ RF_PWR_MASKNEG18dBm | RF_PWR_MASKNEG12dBm | RF_PWR_MASKNEG6dBm | RF_PWR_MASKNEG0dBm | RF_PWR_MASKDEFAULT ]
	*                Power
	*         11   -18 dBm
	*         01   -12 dBm
	*         10    -6 dBm
	*         11     0 dBm
	* @retval None.
	*/
void nRF24_set_RF_SETUP(uint8_t cont_wave, uint8_t rf_dr_low, uint8_t pll_lock, uint8_t rf_dr_high, uint8_t rf_pwr)
{
  uint8_t writing_byte = 0x00;
	writing_byte |= cont_wave | rf_dr_low | pll_lock | rf_dr_high | rf_pwr;
  nRF24_verified_write_register(W_REGISTER_MASK + RF_SETUP, 1, &writing_byte);
}

uint8_t nRF24_get_RF_SETUP()
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + RF_SETUP, 1, &read_byte);
  return read_byte;
}




/**
  * @brief      Clear flag bits in 'STATUS' register.
  * @param      rx_dr Data Ready RX FIFO interrupt. Asserted when new data arrives RX FIFO.
  *               Write 1 to clear bit [ RX_DR_MASK1 ]
	* @param      tx_ds Data Sent TX FIFO interrupt. Asserted when packet transmitted on TX. 
	*               If AUTO_ACK is activated, this bit is set high only when ACK is received. Write 1 to clear bit. [ TX_DS_MASK1 ]
	* @param      max_rt Maximum number of TX retransmits interrupt. Write 1 to clear bit. 
	*               If MAX_RT is asserted it must be cleared to enable further communication. [ MAX_RT_MASK1 ]
  * @retval     None.
  */
void nRF24_clear_STATUS(uint8_t rx_dr, uint8_t tx_ds, uint8_t max_rt) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= rx_dr | tx_ds | max_rt;
  nRF24_verified_write_register(W_REGISTER_MASK + STATUS, 1, &writing_byte);
}

/**
  * @brief      Read <STATUS> register from nRF24.
  * @param      None.
  * @retval     <STATUS> register value.
	* @note    Following masks can be used:
  *   RX_DR_READMASK   
  *   TX_DS_READMASK   
  *   MAX_RT_READMASK  
  *   RX_P_NO_READMASK    
  *   TX_FULL_READMASK    
  */
uint8_t nRF24_get_STATUS(void) 
{
  uint8_t status;
  spi_read_register(R_REGISTER_MASK + STATUS, 1, &status);
  return status;
}



/**
  * @brief      Read <OBSERVE_TX> register from nRF24.
  * @param      None.
  * @retval     <OBSERVE_TX> register value.
  */
uint8_t nRF24_get_OBSERVE_TX()
{
	#define PLOS_CNT_READMASK  0b1111 << 4
	#define ARC_CNT_READMASK   0b1111 << 0
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + OBSERVE_TX, 1, &read_byte);
  return read_byte;
}

/**
  * @brief      Read <RPD> register from nRF24.
  * @param      None.
  * @retval     <RPD> register value.
  */
uint8_t nRF24_get_RPD() 
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + RPD, 1, &read_byte);
  return read_byte;
}


/**
	* @brief  Set RX (receive) address for data pipe 0. Least Significant Byte written first.
	* @param  rx_addr_width The width of RX address. Needs to be consistent with SET_AW.
	* @param  p_rx_addr_p0 Pointer to an array storing receive address for data pipe 0. 
	*           The array pointed should have a length of either 3, 4, or 5 bytes. 
	*           The SET_AW register determines the length of the address array. Default length is 5.
	* @retval None.
	*/
void nRF24_set_RX_ADDR_P0(uint8_t rx_addr_width, uint8_t* p_rx_addr_p0) 
{
  nRF24_verified_write_register(W_REGISTER_MASK + RX_ADDR_P0, rx_addr_width, p_rx_addr_p0);
}


/**
  * @brief      Read <RX_ADDR_P0> register from nRF24.
  * @param[in]  rx_addr_width The width of RX address. Needs to be consistent with SET_AW.
	* @param[out] p_read_buffer Pointer to an array that's used to store the read RX address on Pipe 0.
  * @retval     None.
  */
void nRF24_get_RX_ADDR_P0(uint8_t rx_addr_width, uint8_t* p_read_buffer) 
{
  spi_read_register(R_REGISTER_MASK + RX_ADDR_P0, rx_addr_width, p_read_buffer);
}


/**
	* @brief  Set RX (receive) address for data pipe 1. Least Significant Byte written first.
	* @param  rx_addr_width The width of RX address. Needs to be consistent with SET_AW.
	* @param  p_rx_addr_p1 Pointer to an array storing receive address for data pipe 1. 
	*           The array pointed should have a length of either 3, 4, or 5 bytes. 
	*           The SET_AW register determines the length of the address array. Default length is 5.
	* @note   The RX data pipe 0 and pipe 1 and have totally different addresses. 
	*           However, for RX pipe 2,3,4,5 the [39:8] addresses byte need to be same as RX pipe 1.
	*           Given an example,
	*           RX_ADDR_P0 = 0xE7E7E7E7E7
	*           RX_ADDR_P1 = 0xC2C2C2C2C2
	*           RX_ADDR_P2 = 0x--------C3 = 0xC2C2C2C2C3
	*           RX_ADDR_P3 = 0x--------C4 = 0xC2C2C2C2C4
	*           RX_ADDR_P4 = 0x--------C5 = 0xC2C2C2C2C5
	*           RX_ADDR_P5 = 0x--------C6 = 0xC2C2C2C2C6
	* @retval None.
	*/
void nRF24_set_RX_ADDR_P1(uint8_t rx_addr_width, uint8_t* p_rx_addr_p1) 
{
  nRF24_verified_write_register(W_REGISTER_MASK + RX_ADDR_P1, rx_addr_width, p_rx_addr_p1);
}


/**
  * @brief      Read <RX_ADDR_P1> register from nRF24.
  * @param[in]  rx_addr_width The width of RX address. Needs to be consistent with SET_AW.
	* @param[out] p_read_buffer Pointer to an array that's used to store the read RX address on Pipe 1.
  * @retval     None.
  */
void nRF24_get_RX_ADDR_P1(uint8_t rx_addr_width, uint8_t* p_read_buffer) 
{
  spi_read_register(R_REGISTER_MASK + RX_ADDR_P1, rx_addr_width, p_read_buffer);
}

/**
	* @brief  Set RX (receive) address for data pipe 2. 
	* @param  rx_addr_p2 Byte storing receive address for data pipe 2. 
	*           rx_addr_p2 overwrite the Least Significant Byte on RX_ADDR_P1
	*           Given an example,
	*           RX_ADDR_P1 = 0xC2C2C2C2C2
	*           RX_ADDR_P2 = 0x--------C3 = 0xC2C2C2C2C3
	* @retval None.
	*/
void nRF24_set_RX_ADDR_P2(uint8_t rx_addr_p2) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= rx_addr_p2;
  nRF24_verified_write_register(W_REGISTER_MASK + RX_ADDR_P2, 1, &writing_byte);
}



/**
  * @brief      Read <RX_ADDR_P2> register from nRF24.
  * @param[in]  rx_addr_width The width of RX address. Needs to be consistent with SET_AW.
	* @param[out] p_read_buffer Pointer to an array that's used to store the read RX address on Pipe 2.
  * @retval     None.
  */
void nRF24_get_RX_ADDR_P2(uint8_t rx_addr_width, uint8_t* p_read_buffer) 
{
  spi_read_register(R_REGISTER_MASK + RX_ADDR_P2, rx_addr_width, p_read_buffer);
}


/**
	* @brief  Set RX (receive) address for data pipe 3. 
	* @param  rx_addr_p3 Byte storing receive address for data pipe 3. 
	*           rx_addr_p3 overwrite the Least Significant Byte on RX_ADDR_P1
	*           Given an example,
	*           RX_ADDR_P1 = 0xC2C2C2C2C2
	*           RX_ADDR_P3 = 0x--------C4 = 0xC2C2C2C2C4
	* @retval None.
	*/
void nRF24_set_RX_ADDR_P3(uint8_t rx_addr_p3) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= rx_addr_p3;
  nRF24_verified_write_register(W_REGISTER_MASK + RX_ADDR_P3, 1, &writing_byte);
}

/**
  * @brief      Read <RX_ADDR_P3> register from nRF24.
  * @param[in]  rx_addr_width The width of RX address. Needs to be consistent with SET_AW.
	* @param[out] p_read_buffer Pointer to an array that's used to store the read RX address on Pipe 3.
  * @retval     None.
  */
void nRF24_get_RX_ADDR_P3(uint8_t rx_addr_width, uint8_t* p_read_buffer) 
{
  spi_read_register(R_REGISTER_MASK + RX_ADDR_P3, rx_addr_width, p_read_buffer);
}

/**
	* @brief  Set RX (receive) address for data pipe 4.
	* @param  rx_addr_p4 Byte storing receive address for data pipe 4. 
	*           rx_addr_p4 overwrite the Least Significant Byte on RX_ADDR_P1
	*           Given an example,
	*           RX_ADDR_P1 = 0xC2C2C2C2C2
	*           RX_ADDR_P4 = 0x--------C5 = 0xC2C2C2C2C5
	* @retval None.
	*/
void nRF24_set_RX_ADDR_P4(uint8_t rx_addr_p4) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= rx_addr_p4;
  nRF24_verified_write_register(W_REGISTER_MASK + RX_ADDR_P4, 1, &writing_byte);
}

/**
  * @brief      Read <RX_ADDR_P4> register from nRF24.
  * @param[in]  rx_addr_width The width of RX address. Needs to be consistent with SET_AW.
	* @param[out] p_read_buffer Pointer to an array that's used to store the read RX address on Pipe 4.
  * @retval     None.
  */
void nRF24_get_RX_ADDR_P4(uint8_t rx_addr_width, uint8_t* p_read_buffer) 
{
  spi_read_register(R_REGISTER_MASK + RX_ADDR_P4, rx_addr_width, p_read_buffer);
}

/**
	* @brief  Set RX (receive) address for data pipe 5.
	* @param  rx_addr_p5 Byte storing receive address for data pipe 5. 
	*           rx_addr_p5 overwrite the Least Significant Byte on RX_ADDR_P1
	*           Given an example,
	*           RX_ADDR_P1 = 0xC2C2C2C2C2
	*           RX_ADDR_P5 = 0x--------C6 = 0xC2C2C2C2C6
	* @retval None.
	*/
void nRF24_set_RX_ADDR_P5(uint8_t rx_addr_p5) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= rx_addr_p5;
  nRF24_verified_write_register(W_REGISTER_MASK + RX_ADDR_P5, 1, &writing_byte);
}


/**
  * @brief      Read <RX_ADDR_P5> register from nRF24.
  * @param[in]  rx_addr_width The width of RX address. Needs to be consistent with SET_AW.
	* @param[out] p_read_buffer Pointer to an array that's used to store the read RX address on Pipe 5.
  * @retval     None.
  */
void nRF24_get_RX_ADDR_P5(uint8_t rx_addr_width, uint8_t* p_read_buffer) 
{
  spi_read_register(R_REGISTER_MASK + RX_ADDR_P5, rx_addr_width, p_read_buffer);
}

/**
	* @brief  Set TX (transmit) address. Least Significant Byte written first.
	* @param  tx_addr_width The width of TX address. Needs to be consistent with SET_AW.
	* @param  p_tx_addr Pointer to an array storing the transmit address.
	* @retval None.
	*/
void nRF24_set_TX_ADDR(uint8_t tx_addr_width, uint8_t* p_tx_addr) 
{
  nRF24_verified_write_register(W_REGISTER_MASK + TX_ADDR, tx_addr_width, p_tx_addr);
}


/**
  * @brief      Read <TX_ADDR> register from nRF24.
  * @param[in]  tx_addr_width The width of RX address. Needs to be consistent with SET_AW.
	* @param[out] p_read_buffer Pointer to an array that's used to store the read TX address.
  * @retval     None.
  */
void nRF24_get_TX_ADDR(uint8_t tx_addr_width, uint8_t* p_read_buffer) 
{
  spi_read_register(R_REGISTER_MASK + RX_ADDR_P5, tx_addr_width, p_read_buffer);
}

/**
	* @brief Set number of bytes in RX payload in data pipe 0.
	* @param rx_pw_p0 Number of bytes. 
	*          0 = pipe not used.
	*          1 = 1 byte.
	*          2 = 2 bytes.
	*          ...
	*          32 = 32 bytes.
	*/
void nRF24_set_RX_PW_P0(uint8_t rx_pw_p0) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= rx_pw_p0;
  nRF24_verified_write_register(W_REGISTER_MASK + RX_PW_P0, 1, &writing_byte);
}


/**
  * @brief      Read <RX_PW_P0> register from nRF24.
  * @param      None.
  * @retval     <RX_PW_P0> register value.
  */
uint8_t nRF24_get_RX_PW_P0() 
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + RX_PW_P0, 1, &read_byte);
  return read_byte;
}


/**
	* @brief Set number of bytes in RX payload in data pipe 1.
	* @param rx_pw_p1 Number of bytes. 
	*          0 = pipe not used.
	*          1 = 1 byte.
	*          2 = 2 bytes.
	*          ...
	*          32 = 32 bytes.
	* @retval None.
	*/
void nRF24_set_RX_PW_P1(uint8_t rx_pw_p1) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= rx_pw_p1;
  nRF24_verified_write_register(W_REGISTER_MASK + RX_PW_P1, 1, &writing_byte);
}


/**
  * @brief      Read <RX_PW_P1> register from nRF24.
  * @param      None.
  * @retval     <RX_PW_P1> register value.
  */
uint8_t nRF24_get_RX_PW_P1() 
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + RX_PW_P1, 1, &read_byte);
  return read_byte;
}


/**
	* @brief Set number of bytes in RX payload in data pipe 2.
	* @param rx_pw_p2 Number of bytes. 
	*          0 = pipe not used.
	*          1 = 1 byte.
	*          2 = 2 bytes.
	*          ...
	*          32 = 32 bytes.
	* @retval None.
	*/
void nRF24_set_RX_PW_P2(uint8_t rx_pw_p2) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= rx_pw_p2;
  nRF24_verified_write_register(W_REGISTER_MASK + RX_PW_P2, 1, &writing_byte);
}


/**
  * @brief      Read <RX_PW_P2> register from nRF24.
  * @param      None.
  * @retval     <RX_PW_P2> register value.
  */
uint8_t nRF24_get_RX_PW_P2() 
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + RX_PW_P2, 1, &read_byte);
  return read_byte;
}


/**
	* @brief Set number of bytes in RX payload in data pipe 3.
	* @param rx_pw_p3 Number of bytes. 
	*          0 = pipe not used.
	*          1 = 1 byte.
	*          2 = 2 bytes.
	*          ...
	*          32 = 32 bytes.
	* @retval None.
	*/
void nRF24_set_RX_PW_P3(uint8_t rx_pw_p3) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= rx_pw_p3;
  nRF24_verified_write_register(W_REGISTER_MASK + RX_PW_P3, 1, &writing_byte);
}

/**
  * @brief      Read <RX_PW_P3> register from nRF24.
  * @param      None.
  * @retval     <RX_PW_P3> register value.
  */
uint8_t nRF24_get_RX_PW_P3() 
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + RX_PW_P3, 1, &read_byte);
  return read_byte;
}


/**
	* @brief Set number of bytes in RX payload in data pipe 4.
	* @param rx_pw_p4 Number of bytes. 
	*          0 = pipe not used.
	*          1 = 1 byte.
	*          2 = 2 bytes.
	*          ...
	*          32 = 32 bytes.
	* @retval None.
	*/
void nRF24_set_RX_PW_P4(uint8_t rx_pw_p4) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= rx_pw_p4;
  nRF24_verified_write_register(W_REGISTER_MASK + RX_PW_P4, 1, &writing_byte);
}

/**
  * @brief      Read <RX_PW_P4> register from nRF24.
  * @param      None.
  * @retval     <RX_PW_P4> register value.
  */
uint8_t nRF24_get_RX_PW_P4() 
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + RX_PW_P4, 1, &read_byte);
  return read_byte;
}

/**
	* @brief Set number of bytes in RX payload in data pipe 5.
	* @param rx_pw_p5 Number of bytes. 
	*          0 = pipe not used.
	*          1 = 1 byte.
	*          2 = 2 bytes.
	*          ...
	*          32 = 32 bytes.
	* @retval None.
	*/
void nRF24_set_RX_PW_P5(uint8_t rx_pw_p5) 
{
  uint8_t writing_byte = 0x00;
	writing_byte |= rx_pw_p5;
  nRF24_verified_write_register(W_REGISTER_MASK + RX_PW_P5, 1, &writing_byte);
}

/**
  * @brief      Read <RX_PW_P5> register from nRF24.
  * @param      None.
  * @retval     <RX_PW_P5> register value.
  */
uint8_t nRF24_get_RX_PW_P5() 
{
  uint8_t read_byte;
  spi_read_register(R_REGISTER_MASK + RX_PW_P5, 1, &read_byte);
  return read_byte;
}


/**
  * @brief      Read 'FIFO_STATUS' register from nRF24.
  * @param      None.
  * @retval     FIFO_STATUS register value.
	* @note  Following masks can be used:
	*  TX_REUSE_READMASK  
	*  TX_EMPTY_READMASK  
	*  RX_FULL_READMASK 
	*  RX_EMPTY_READMASK 
  */
uint8_t nRF24_get_FIFO_STATUS() 
{
  uint8_t fifo_status;
  spi_read_register(R_REGISTER_MASK + FIFO_STATUS, 1, &fifo_status);

  return fifo_status;
}

