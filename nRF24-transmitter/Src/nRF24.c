/**
  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********
  * @file      :     nRF24.c
  * @author    :     Luyao Han
  * @email     :     luyaohan1001@gmail.com
  * @brief     :     C library for Nordic nRF24L01 2.4GHz wireless transceiver.
  * @date      :     04-21-2022
  * Copyright (C) 2022-2122 Luyao Han. The following code may be shared or modified for personal use / non-commercial use only.
  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********  */

/* Includes -------------------------------------------------------------------*/
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
	* @retval None
	*/
void SPI_SCK_1(){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);	
}

/**
	* @brief Set low on SCK pin of SPI bus.
	* @param None
	* @retval None
	*/
void SPI_SCK_0(){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);	
} 

/**
	* @brief Set high on MOSI pin of SPI bus.
	* @param None
	* @retval None
	*/
void SPI_MOSI_1(){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	
}

/**
	* @brief Set low on MOSI pin of SPI bus.
	* @param None
	* @retval None
	*/
void SPI_MOSI_0(){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);	
}

/**
	* @brief Set high on CS pin of SPI bus.
	* @param None
	* @retval None
	*/
void SPI_CS_1() {
		/* CS High == CSN Low */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	
}

/**
	* @brief Set low on CS pin of SPI bus.
	* @param None
	* @retval None
	*/
void SPI_CS_0(){
		/* CS Low == CSN High */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
}

/**
	* @brief Set high on Chip-Enable pin of nRF24L01.
	* @param None
	* @retval None
	*/
void SPI_CE_1(){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);	
}


/**
	* @brief Set low on Chip-Enable pin of nRF24L01.
	* @param None
	* @retval None
	*/
void SPI_CE_0(){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}
    
GPIO_PinState SPI_READ_MISO(){
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
}


/* SPI Operations -------------------------------------------------------------------------------*/
void spi_delay() {
	HAL_Delay(1);
}


void gpio_clockout_8_bits(uint8_t txData) {
  spi_delay();
  for (int i = 0; i < 8; ++i) {
      SPI_SCK_0();
      spi_delay();
      if(txData & 0x80) // MSBit first
          SPI_MOSI_1();
      else
          SPI_MOSI_0();
      SPI_SCK_1(); // clock data
      txData = txData << 1; // load next MSB
      spi_delay();
  }
  SPI_SCK_0();
}

uint8_t gpio_clockin_8_bits(){
  uint8_t rxData = 0;
  spi_delay();
  for (int i=0; i < 8; ++i) {
      SPI_SCK_0();
      spi_delay();
      SPI_MOSI_0();
      SPI_SCK_1();
      spi_delay();
      rxData = rxData << 1; // Why shift first then OR'? range (0, 8) will need to shift only 7 times.
      rxData |= SPI_READ_MISO();
      spi_delay();
  }
  SPI_SCK_0();
  return rxData;
}
  

void spi_read_register(uint8_t reg, uint8_t num_bytes, uint8_t* pbuf){
  // Select chip
  SPI_CS_1();
  
  // Write register address to read.
  gpio_clockout_8_bits(reg);
  // Read value
  for (int i = 0; i < num_bytes; ++i) {
    pbuf[i] = gpio_clockin_8_bits();
  }
  
  // Deselect chip
  SPI_CS_0();
}


void spi_write_register(uint8_t reg, uint8_t num_bytes, uint8_t* writing_data){
  // Select chip (CSN LOW)
  SPI_CS_1();

  // Write chip register 
  gpio_clockout_8_bits(reg);  // W_REGISTER_MASK is specifc to nRF24.
  // Write value
  for (int i = 0; i < num_bytes; ++i){
    uint8_t writing_byte = writing_data[i];
    gpio_clockout_8_bits(writing_byte);
  }

  // Deselect chip (CSN HIGH)
  SPI_CS_0();
}


// nRF24 Specific
bool spi_verified_write_register(uint8_t reg, uint8_t num_bytes, uint8_t* writing_data){

    char message[64] = {'\0'};

    spi_write_register(reg, num_bytes, writing_data); 

    uint8_t read_data[num_bytes];

    // reg & ~ W_REGISTER_MASK is a reverse operation of reg | W_REGISTER_MASK
    spi_read_register(R_REGISTER_MASK | (reg & ~W_REGISTER_MASK), num_bytes, read_data);
    for (int i = 0; i < num_bytes; ++i) {
      // if there's any mismatch between written data and read data from the register.
      if (read_data[i] != writing_data[i]) {
				strcpy(message, "Problem writing to SPI register -- ");
				HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
				sprintf(message, "writing_data: <%02x> read_data: <%02x>\n", writing_data[i], read_data[i]);
				HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
        return true;
      } else {
				strcpy(message, "Success writing to SPI register -- ");
				HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
				sprintf(message, "writing_data: <%02x> read_data: <%02x>\n", writing_data[i], read_data[i]);
				HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
  		}
	}
  return false;
}


uint8_t nRF24_get_STATUS() {
  uint8_t stat;
  spi_read_register(R_REGISTER_MASK + STATUS, 1, &stat);
  // Serial.print("- STATUS: "); Serial.println(stat,HEX);
  return stat;
}

uint8_t nRF24_get_FIFO_STATUS() {
  uint8_t fifo_status;
  spi_read_register(R_REGISTER_MASK + FIFO_STATUS, 1, &fifo_status);
  // Serial.print("- FIFO STATUS: "); Serial.println(fifo_status,HEX);
  return fifo_status;
}

uint8_t nRF24_get_CONFIG() {
  uint8_t config_reg;
  spi_read_register(R_REGISTER_MASK + CONFIG, 1, &config_reg);
  // Serial.print("- CONFIG: "); Serial.println(config_reg,HEX);
  return config_reg;
}


/**  
 *  Brief: We need to make sure the Transmitter work before we have a Receiver working.
 *  
 *  How to know TX payload is loaded?
 *  
 *  Steps: 1. Disable Auto Acknowledgement, Auto Retransmit. TX_DS will be be set if these two are not turn-off.
 *         2. TX_DS (in STATUS register) is expected to be set when data in TX FIFO is set.
 *         STATUS = 0x2E after firing means we have a working TX module.
 *        
 *        After writing to W_TX_PAYLOAD, TX_EMPTY (in FIFO_STATUS register) becomes 0.
 *        
 *        What happens if sending is not successful?
 *        TX_FULL (in FIFO_STATUS register) becomes 1.
 *        TX_FULL (in STATUS register) becomes 1.
 *        TX_DS (in STATUS register) remains 0.  
 *  States: 
 *        The states can be referred in 6.1.1 State diagram.
 *  
 *  STATUS register
 *            7             6           5         4         3:1         0
 *        <reserved>      RX_DR       TX_DS     MAX_RT     RX_P_NO     TX_FULL
 *        -----------------------------------------------------------------------
 *         Always 0       Receive     Transfer  Maximum    000-101 :    1: 
 *                        Data        Data      TX         Data Pipe #  TX_FULL
 *                        Ready       Sent      Transmits  110 :
 *                                                         Not Used.
 *                                                         111:
 *                                                         RX FIFO Empty.
 */
bool nRF24_tx_self_test() {

  char message1[] = "---- nrf24 tx self test. ----\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)message1, strlen(message1), 100);

  char message2[] = "---- This test to verifies function of a tranmitter send without a receiver. ----\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)message2, strlen(message2), 100);

  uint8_t nRF24_status = 0x00;

  // Note that if we reset the Arduino without re-poweron the chip, initial value of registers such as STATUS or CONFIG may be different from one listed in datasheet.
  // [Current State: Power-on reset 100 ms] 
  SPI_CE_0();
  // [Current State: (RF transmission is) Power Down (But SPI is alive.)]
  uint8_t writing_byte = 0x00;
  spi_verified_write_register(W_REGISTER_MASK + EN_AA, 1, &writing_byte);        // disable auto acknowledgement  
  spi_verified_write_register(W_REGISTER_MASK + EN_RXADDR, 1, &writing_byte);    // disable RX data pipes
  spi_verified_write_register(W_REGISTER_MASK + SETUP_RETR, 1, &writing_byte);   // disable automatic re-transmit, ARC = 0000
  writing_byte = 0x0E;
  spi_verified_write_register(W_REGISTER_MASK + CONFIG, 1, &writing_byte);       // PWR_UP = 1 PRIMRX=0 (TX mode)

  // PWR_UP=1, state transition -> [Current State: Standby-I]
  uint8_t test_payload[4] = {0xDE, 0xAD, 0xBE, 0xEF}; // clock in a payload, now TX FIFO not empty 
  spi_write_register(W_TX_PAYLOAD, 4, test_payload);
  SPI_CE_1(); // Chip Enable. Fire the packet out on the antenna!
  
  // TX FIFO not empty AND CE = 1, state transition -> [Current State: TX MODE]
  nRF24_status = nRF24_get_STATUS();
  spi_delay(1);

  // CE=0, state transition -> now return to [State: Standby-I]. 
  SPI_CE_0();
  // PWR_UP = 0, state transition -> now return to [State: Power Down]
  writing_byte = 0x08; // write default value for CONFIG register (writing_byte = 0)
  spi_verified_write_register(W_REGISTER_MASK + CONFIG, 1, &writing_byte);       

  // Now the chip is back to power down mode, check test result. 
  if (nRF24_status & 0x2E) { // TX_DS bit is set.
  	char message3[] = "\n > nRF24 transmission self-test has passed. STATUS has value of 0x2E, TX_DS (transfer data sent) was set, RX_P_NO = 111, means RX FIFO Empty.\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)message3, strlen(message3), 100);
    return true;
  } else {
  	char message4[] = "\n > nRF24 transmission self-test has failed. STATUS is expected 0x2E.";
		HAL_UART_Transmit(&huart2, (uint8_t*)message4, strlen(message4), 100);
    return false;
  }
  
}


void nRF24_configure_tx_mode() {
    SPI_CE_0();

    // Set TX_ADDR for sender. On the Receiver side, set RX_ADDR_P0 with same value.
    unsigned char TX_ADDRESS[5] = {0x10,0xDE,0x10,0x10,0x10};  // 定义一个静态发送地址
    spi_write_register(W_REGISTER_MASK + TX_ADDR, 5, TX_ADDRESS);     // 写入发送地址      

    uint8_t writing_byte;
    writing_byte = 0x00;
    spi_verified_write_register(W_REGISTER_MASK + EN_AA, 1, &writing_byte);

    writing_byte = 0x00;
    spi_verified_write_register(W_REGISTER_MASK + EN_RXADDR, 1, &writing_byte);

    writing_byte = 0x00;
    spi_verified_write_register(W_REGISTER_MASK + SETUP_RETR, 1, &writing_byte);

    writing_byte = 40;
    spi_verified_write_register(W_REGISTER_MASK + RF_CH, 1, &writing_byte);

    writing_byte = 0x07;
    spi_verified_write_register(W_REGISTER_MASK + RF_SETUP, 1, &writing_byte);

    // PWR_UP, state transition -> Standby-I
    writing_byte = 0x0e;
    spi_verified_write_register(W_REGISTER_MASK + CONFIG, 1, &writing_byte);
    spi_delay(150);

    // CE = 1 is not activated until we write to TX FIFO so stays in Standby-I mode.
}


void nRF24_keep_sending() {
  uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF}; // clock in a payload, TX FIFO not empty 
  
  spi_write_register(W_TX_PAYLOAD, 4, (uint8_t*) payload);
  SPI_CE_1(); // fire out the transmit packet

  uint8_t stat = nRF24_get_STATUS();
	char message[64];
	sprintf(message, "Status is: %x\n", stat);
	HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);

  if (stat == 0x2e) { // TX_DS bit is set.
		strcpy(message, "nRF24 send successful.\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
  } else {
		strcpy(message, "nRF24 send failed.\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
  }
  // write 1 to clear TX_DS
  uint8_t writing_byte = 0x20;
  spi_write_register(W_REGISTER_MASK + STATUS, 1, &writing_byte); 
  SPI_CE_0(); // stop transmission. return to Standby-I state.
  // Return to [State: Standby-I]
}






