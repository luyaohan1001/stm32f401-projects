
#include "stm32f4xx_hal.h"
#include "nRF24.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// advertisement PDU
__packed struct btle_adv_pdu {

	// packet header
	uint8_t pdu_type; // PDU type
	uint8_t pl_size;  // payload size 负载大小，包括6字节的mac

	// MAC address
	uint8_t mac[6];

	// payload (including 3 bytes for CRC)
	uint8_t payload[24];
};

// payload chunk in advertisement PDU payload
__packed struct btle_pdu_chunk {
	uint8_t size;
	uint8_t type;
	uint8_t data[];
};






typedef struct
{
	struct btle_adv_pdu buffer;
	char *name;
	uint8_t current;   // current channel index
}ble_struct;




//把无线模块初始化为蓝牙兼容的形式
void ble_begin( ble_struct *ble,char* _name ) ;

//发送广播数据
int ble_advertise( ble_struct *ble,uint8_t data_type, void* buf, uint8_t buflen ) ;

//改变信道
void ble_hopChannel(ble_struct *ble) ;





