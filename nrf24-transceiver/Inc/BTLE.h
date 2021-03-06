/*
 * Copyright (C) 2013 Florian Echtler <floe@butterbrot.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation.
*/

#ifndef _BTLE_H_
#define _BTLE_H_


#include "stm32f4xx_hal.h"
#include "nRF24.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// float as used on the nRF8001 and nRF51822 platforms
// and on the "nRF Master Control Panel" and "nRF Temp 2.0" apps.
// This float representation has the first 8 bits as a base-10 exponent
// and the last 24 bits as the mantissa.
typedef int32_t nRF_Float;

// Service UUIDs used on the nRF8001 and nRF51822 platforms
#define NRF_TEMPERATURE_SERVICE_UUID            0x1809
#define NRF_BATTERY_SERVICE_UUID                0x180F
#define NRF_DEVICE_INFORMATION_SERVICE_UUID     0x180A
#define NRF_EDDYSTONE_SERVICE_UUID              0xFEAA

#define EDDYSTONE_UID_FRAME_TYPE      0x00
#define EDDYSTONE_URL_FRAME_TYPE      0x10
#define EDDYSTONE_TLM_FRAME_TYPE      0x20
#define EDDYSTONE_EID_FRAME_TYPE      0x30

#define EDDYSTONE_HTTPWWW_URL_SCHEME     0x00  //stands for "http://www."
#define EDDYSTONE_HTTPSWWW_URL_SCHEME    0x01  //stands for "https://www."
#define EDDYSTONE_HTTP_URL_SCHEME        0x02  //stands for "http://"
#define EDDYSTONE_HTTPS_URL_SCHEME       0x03  //stands for "https://"


// helper struct for sending temperature as BT service data
struct nrf_service_data {
  int16_t   service_uuid;
  nRF_Float value;
};


#define EDDYSTONE_URL_HEADER_LENGTH (5)

struct eddystone_url_service_data {
  uint16_t service_uuid;
  uint8_t frame_type;
  int8_t tx_power; // take -20 if unsure
  uint8_t url_scheme;
  uint8_t encoded_url[11]; //11 bytes at max with current implementation. see https://github.com/google/eddystone/tree/master/eddystone-url#eddystone-url-http-url-encoding
};

// advertisement PDU
struct btle_adv_pdu {

  // packet header
  uint8_t pdu_type; // PDU type
  uint8_t pl_size;  // payload size

  // MAC address
  uint8_t mac[6];

  // payload (including 3 bytes for CRC)
  uint8_t payload[24];
};



// payload chunk in advertisement PDU payload
struct btle_pdu_chunk {
  uint8_t size;
  uint8_t type;
  uint8_t data[];
};

// helper macro to access chunk at specific offset
#define chunk(x,y) ((struct btle_pdu_chunk*)(x.payload+y))



// BTLE( RF24* _radio ); // constructor 

static nRF_Float to_nRF_Float(float t); // convert an arduino float to a nRF_Float

void begin( const char* _name ); // set BTLE-compatible radio parameters & name

void setChannel( uint8_t num ); // set the current channel (from 36 to 38)
void hopChannel();              // hop to the next channel

// Broadcast an advertisement packet with a specific data type
// Standardized data types can be seen here: 
// https://www.bluetooth.org/en-us/specification/assigned-numbers/generic-access-profile
bool advertise( uint8_t data_type, void* buf, uint8_t len ); 

// Broadcast an advertisement packet with optional payload
// Data type will be 0xFF (Manufacturer Specific Data)
bool advertise_short( void* buf, uint8_t len ); 

bool listen( int timeout);         // listen for advertisement packets (if true: result = buffer)

struct btle_adv_pdu buffer;  // buffer for received BTLE packet (also used for outgoing!)

void preparePacket();
bool addChunk(uint8_t chunk_type, uint8_t buflen, const void* buf);
void transmitPacket();

void whiten( uint8_t len );
void swapbuf( uint8_t len );
void crc( uint8_t len, uint8_t* dst );

// RF24* radio;       // pointer to the RF24 object managing the radio
uint8_t current;   // current channel index
const char* name;  // name of local device

#endif // _BTLE_H_

