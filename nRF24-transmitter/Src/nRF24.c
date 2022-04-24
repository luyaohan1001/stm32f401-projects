/**
  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********
  * @file      :     nRF24.c
  * @author    :     Luyao Han
  * @email     :     luyaohan1001@gmail.com
  * @brief     :     C library for Nordic nRF24L01 2.4GHz wireless transceiver.
  * @date      :     04-21-2022
  * Copyright (C) 2022-2122 Luyao Han. The following code may be shared or modified for personal use / non-commercial use only.
  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********  */

/* Includes ------------------------------------------------------------------- */
#include "nRF24.h"

/* GPIO Operations --------------------------------------------------------*/

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
  

/* SPI Operations -------------------------------------------------------------------------------*/
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


/* nRF24 Operations --------------------------------------------------------------------------------------------*/


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
  * @brief      Write to a register on target device through SPI. Read the same registers after write to confirm that the write has been successful.
  * @param[in]  reg The device register to write value to.
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
      if (read_data[i] != p_writing_data[i])  // check mismatch between written data and read data from target register.
      {
        strcpy(message, "Problem writing to SPI register -- ");
        serial_print(message);
        sprintf(message, "writing data: <%#02x> read_data: <%#02x>\n", p_writing_data[i], read_data[i]);
        serial_print(message);
        return true;
      } else {
        strcpy(message, "Success writing to SPI register -- ");
        serial_print(message);
        sprintf(message, "writing data: <%#02x> read_data: <%#02x>\n", p_writing_data[i], read_data[i]);
        serial_print(message);
      }
  }
  return false;
}


/**  
 *  @brief:  Test nRF24 transmitter function without a receiver.
 *  @note:  
 *      
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
bool nRF24_tx_self_test() 
{
	char message[128];
	uint8_t writing_byte;

  strcpy(message, "---- nrf24 tx self test. ----\n");
  serial_print(message);

  strcpy(message, "---- This test to verifies function of a tranmitter send without a receiver. ----\n");
  serial_print(message);

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
  nRF24_status = nRF24_get_STATUS();
  spi_delay(1);

  /* CE=0, state transition -> now return to [Standby-I]. */
  nRF24_CE_0();

  /* PWR_UP = 0, state transition to [Power Down] */
  writing_byte = 0x08; // write default value for CONFIG register (writing_byte = 0)
  nRF24_verified_write_register(W_REGISTER_MASK + CONFIG, 1, &writing_byte);       

  /* Now the chip is back to power down mode, check test result. */
  if (nRF24_status & 0x2E) 
  {
    strcpy(message, "\n > nRF24 transmission self-test has passed. \
                       STATUS has value of 0x2E. \
                       TX_DS (transfer data sent) was set. \
                       RX_P_NO = 111, means RX FIFO Empty. \n");
    serial_print(message);
    return true;
  } 
  else 
  {
    strcpy(message, "\n > nRF24 transmission self-test has failed. STATUS is expected 0x2E.");
    serial_print(message);
    return false;
  }
  
}

/**
  * @brief  Configure nRF24 to work in TX (transmit) mode.
  * @param  None.
  * @retval None.
  * @note   After nRF24_configure_tx_mode() is called, use nRF24_keep_sending() to keep sending data.
  */
void nRF24_configure_tx_mode() 
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
  */
void nRF24_keep_sending() 
{
  uint8_t payload[] = {0xBE, 0xEF, 0xCA, 0xFE}; // clock in a payload, TX FIFO not empty 

  char debug_msg[64];
  
  spi_write_register(W_TX_PAYLOAD, 4, (uint8_t*) payload);

  /* Fire out the transmit packet */
  nRF24_CE_1(); 

  uint8_t stat = nRF24_get_STATUS();

  sprintf(debug_msg, "<STATUS> REGISTER : %x\n", stat);
  serial_print(debug_msg);

  if (stat == 0x2e) // TX_DS bit is set.
  {
    strcpy(debug_msg, "nRF24 send successful.\n");
    serial_print(debug_msg);
  } 
  else 
  {
    strcpy(debug_msg, "nRF24 send failed.\n");
    serial_print(debug_msg);
  }
  // write 1 to clear TX_DS, TX_DS bit is Write-to-Clear.
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




/**
  * @brief      Read 'STATUS' register from nRF24.
  * @param      None.
  * @retval     STATUS register value.
  */
uint8_t nRF24_get_STATUS(void) 
{
  uint8_t status;
  spi_read_register(R_REGISTER_MASK + STATUS, 1, &status);
  return status;
}

/**
  * @brief      Read 'FIFO_STATUS' register from nRF24.
  * @param      None.
  * @retval     FIFO_STATUS register value.
  */
uint8_t nRF24_get_FIFO_STATUS() 
{
  uint8_t fifo_status;
  spi_read_register(R_REGISTER_MASK + FIFO_STATUS, 1, &fifo_status);
  return fifo_status;
}

/**
  * @brief      Read 'CONFIG' register from nRF24.
  * @param      None.
  * @retval     CONFIG register value.
  */
uint8_t nRF24_get_CONFIG() 
{
  uint8_t config_reg;
  spi_read_register(R_REGISTER_MASK + CONFIG, 1, &config_reg);
  return config_reg;
}



/**
	* @brief  nRF24 set <CONFIG> register
	*
	*
void nRF24_set_CONFIG(uint8_t MASK_RX_DR, uint8_t MASK_TX_DS, uint8_t MASK_MAX_RT, uint8_t EN_CRC, uint8_t CRCO, uint8_t PWR_UP, uint8_t PRIM_RX) 
{
	
}
*/











