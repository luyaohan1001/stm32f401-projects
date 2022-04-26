#include "ble.h"




const uint8_t channel[3]   = {37,38,39};  // logical BTLE channel number (37-39)
const uint8_t frequency[3] = { 2,26,80};  // physical frequency (2400+x MHz)


// add data segment
int ble_addChunk(ble_struct *ble,uint8_t chunk_type, uint8_t buflen, const void* buf)
{
	if (ble->buffer.payload_size + buflen + 2 > 21 + 6) // (buflen+2) is how much this chunk will take, 21 is payload size without crc and 6 is MAC size
		return -1;
	
	struct btle_pdu_chunk* chunk = (struct btle_pdu_chunk*) (ble->buffer.payload+ble->buffer.payload_size-6);
	chunk->type = chunk_type;
	for (uint8_t i = 0; i < buflen; i++)
		chunk->data[i] = ((uint8_t*)buf)[i];
	chunk->size = buflen + 1;
	ble->buffer.payload_size += buflen + 2;
	return 0;
}



void ble_hopChannel(ble_struct *ble) 
{
	ble->current++;
	if (ble->current >= sizeof(channel)) ble->current = 0;

	uint8_t writing_byte;
  writing_byte = frequency[ble->current];
  nRF24_verified_write_register(W_REGISTER_MASK + RF_CH, 1, &writing_byte);
}




// Advertise a ble packet.
int ble_advertise( ble_struct *ble,uint8_t data_type, void* buf, uint8_t buflen ) 
{
	ble_preparePacket(ble);
	
	// add custom data, if applicable
	if (buflen > 0) {
		int success = ble_addChunk(ble,data_type, buflen, buf);
		if (0!=success) {
			return -1;
		}
	}
	
	ble_transmitPacket(ble);
	return 0;
}



void ble_preparePacket(ble_struct *ble) 
{

	// This is a rather convoluted hack to extract the month number from the build date in
	// the __DATE__ macro using a small hash function + lookup table. Since all inputs are
	// const, this can be fully resolved by the compiler and saves over 200 bytes of code.

#define month(m) month_lookup[ (( ((( (m[0] % 24) * 13) + m[1]) % 24) * 13) + m[2]) % 24 ]
	const uint8_t month_lookup[24] = { 0,6,0,4,0,1,0,17,0,8,0,0,3,0,0,0,18,2,16,5,9,0,1,7 };
	// Pseudo-random MAC address
	// ble->buffer.mac[0] = ((__TIME__[6]-0x30) << 4) | (__TIME__[7]-0x30);
	// ble->buffer.mac[1] = ((__TIME__[3]-0x30) << 4) | (__TIME__[4]-0x30);
	// ble->buffer.mac[2] = ((__TIME__[0]-0x30) << 4) | (__TIME__[1]-0x30);
	// ble->buffer.mac[3] = ((__DATE__[4]-0x30) << 4) | (__DATE__[5]-0x30);
	// ble->buffer.mac[4] = month(__DATE__);
	// ble->buffer.mac[5] = ((__DATE__[9]-0x30) << 4) | (__DATE__[10]-0x30) | 0xC0; // static random address should have two topmost bits set

	// set MAC address of bluetooth	
	ble->buffer.mac[0] = 0x00;
	ble->buffer.mac[1] = 0x11;
	ble->buffer.mac[2] = 0x22;
	ble->buffer.mac[3] = 0x33;
	ble->buffer.mac[4] = 0x44;
	ble->buffer.mac[5] = 0x55;
	
	// ble->buffer.pdu_type = 0x42;    // PDU type: ADV_NONCONN_IND, TX address is random
	ble->buffer.pdu_type = 0x02;
	ble->buffer.payload_size = 6; //including MAC
	
	// add device descriptor chunk
	uint8_t flags = 0x05;
	ble_addChunk(ble, 0x01, 1, &flags);
	
	// add "complete name" chunk
	if (strlen(ble->name) > 0) {
		ble_addChunk(ble,0x09, strlen(ble->name), ble->name);
	}

}



/**
	* @brief Transmit packet(s) through Bluetooth Low Energy.
	* @note Assume that the nRF24 has already been set TX MODE in <CONFIG> and CE = 1 already enabled. 
	*            nRF24 remains in [Standby-I] state, waiting for data written into TX FIFO.
	*            Once TX FIFO non-empty, nRF24 transfer to [TX Mode] state and start GFSK modulation and fire out the data.
	*/
void ble_transmitPacket(ble_struct *ble) 
{
	uint8_t pls = ble->buffer.payload_size - 6;

	// calculate CRC over header+MAC+payload, append after payload
	uint8_t* outbuf = (uint8_t*)&ble->buffer;
	ble_crc(ble, pls + 8, outbuf + pls + 8 );
	
	// whiten header+MAC+payload+CRC, swap bit order
	ble_whiten(ble, pls + 11);
	ble_swapbuf(ble, pls + 11);
	
	char msg[64];
	serial_print("printing outbuf: ");
	for (int i = 0; i < pls+11; ++i) {
		sprintf(msg, "%#02x", outbuf[i]);
		serial_print(msg);
	}
	serial_print("\n");

  spi_write_register(W_TX_PAYLOAD, 32, outbuf);
  // nRF24_CE_1(); // fire out the packet

}






// change buffer contents to "wire bit order"
void ble_swapbuf(ble_struct *ble, uint8_t len) 
{

	uint8_t* buf = (uint8_t*)&ble->buffer;

	while (len--) 
	{
		uint8_t a = *buf;
		uint8_t v = 0;

		if (a & 0x80) v |= 0x01;
		if (a & 0x40) v |= 0x02;
		if (a & 0x20) v |= 0x04;
		if (a & 0x10) v |= 0x08;
		if (a & 0x08) v |= 0x10;
		if (a & 0x04) v |= 0x20;
		if (a & 0x02) v |= 0x40;
		if (a & 0x01) v |= 0x80;

		*(buf++) = v;
	}
}






// see BT Core Spec 4.0, Section 6.B.3.2
void ble_whiten(ble_struct *ble, uint8_t len) 
{

	uint8_t* buf = (uint8_t*)&ble->buffer;

	// initialize LFSR with current channel, set bit 6
	uint8_t lfsr = channel[ble->current] | 0x40;

	while (len--) {
		uint8_t res = 0;
		// LFSR in "wire bit order"
		for (uint8_t i = 1; i; i <<= 1) {
			if (lfsr & 0x01) {
				lfsr ^= 0x88;
				res |= i;
			}
			lfsr >>= 1;
		}
		*(buf++) ^= res;
	}
}



void ble_crc( ble_struct *ble,uint8_t len, uint8_t* dst ) 
{

	uint8_t* buf = (uint8_t*)&ble->buffer;

	// initialize 24-bit shift register in "wire bit order"
	// dst[0] = bits 23-16, dst[1] = bits 15-8, dst[2] = bits 7-0
	dst[0] = 0xAA;
	dst[1] = 0xAA;
	dst[2] = 0xAA;

	while (len--) {

		uint8_t d = *(buf++);

		for (uint8_t i = 1; i; i <<= 1, d >>= 1) {

			// save bit 23 (highest-value), left-shift the entire register by one

			uint8_t t = dst[0] & 0x01;         
			
			dst[0] >>= 1;

			if (dst[1] & 0x01) 
			{
				dst[0] |= 0x80; 
			}
			
			dst[1] >>= 1;

			if (dst[2] & 0x01) 
			{
				dst[1] |= 0x80; 
			}
			
			dst[2] >>= 1;

			// if the bit just shifted out (former bit 23) and the incoming data
			// bit are not equal (i.e. bit_out ^ bit_in == 1) => toggle tap bits
			if (t != (d & 1)) {
				// toggle register tap bits (=XOR with 1) according to CRC polynom
				dst[2] ^= 0xDA; // 0xDA = 0b11011010 inv. = 0b01011011 ^= x^6 + x^4 + x^3 + x + 1
				dst[1] ^= 0x60; // 0x60 = 0b01100000 inv. = 0b00000110 ^= x^10+x^9
			}
		}
	}
}



void ble_begin( ble_struct *ble,char* _name ) 
{

	ble->name = _name;


	nRF24_CE_0();
		
	uint8_t writing_byte;
	
  writing_byte = 0x00;
  nRF24_verified_write_register(W_REGISTER_MASK + EN_AA, 1, &writing_byte);

  writing_byte = 0b00000111; // '00' - 1Mbps '11' - 0dBm
  nRF24_verified_write_register(W_REGISTER_MASK + RF_SETUP, 1, &writing_byte);

  writing_byte = frequency[ble->current];
  nRF24_verified_write_register(W_REGISTER_MASK + RF_CH, 1, &writing_byte);

  writing_byte = 0b00000000; // Wait 250us, disable auto retransmit
  nRF24_verified_write_register(W_REGISTER_MASK + SETUP_RETR, 1, &writing_byte);

  writing_byte = 0b00000010; // 4 bytes '10' on AW bits
  nRF24_verified_write_register(W_REGISTER_MASK + SETUP_AW, 1, &writing_byte);

  writing_byte = 0x01; // 4 bytes '10' on AW bits
  nRF24_verified_write_register(W_REGISTER_MASK + EN_RXADDR, 1, &writing_byte);

  // Set TX_ADDR for sender. On the Receiver side, set RX_ADDR_P0 with same value.
  uint8_t TX_ADDRESS[4] = {0x71,0x91,0x7D,0x6B};  // 4 byte transmit-address
  nRF24_verified_write_register(W_REGISTER_MASK + TX_ADDR, 4, TX_ADDRESS);     // Write transmit-address to nRF24
  nRF24_verified_write_register(W_REGISTER_MASK + RX_ADDR_P0, 4, TX_ADDRESS);     // Write transmit-address to nRF24

  // PWR_UP, state transition to [Standby-I]
  writing_byte = 0x06;
  nRF24_verified_write_register(W_REGISTER_MASK + CONFIG, 1, &writing_byte);
  spi_delay(150);
  
	nRF24_CE_1();
}


