#pragma once

#include <AP_HAL/AP_HAL.h>


#define XBEEMAXLEN 300
#define NEIGHBOUR_NUM 10

enum Neighbours_Addr : uint16_t {
	NEI0 = 0xE0E0,
	NEI1 = 0xE1E1,
	NEI2 = 0xE2E2,
	NEI3 = 0xE3E3,
	NEI4 = 0xE4E4,
	NEI5 = 0xE5E5,
	NEI6 = 0xE6E6,
	NEI7 = 0xE7E7,
	NEI8 = 0xE8E8,
	NEI9 = 0xE9E9
};

class Xbee_Protocol{
public:		

	Xbee_Protocol(uint16_t tag_adr = 0xFFFF):target_adr(tag_adr){}
	
	void init_xbee(AP_HAL::UARTDriver* uart){
		_uart = uart;
		if (_uart != nullptr)
        	_uart->begin(57600);
        memset(xbee_data, 0, sizeof(xbee_data));
        memset(xbee_data_len, 0, sizeof(xbee_data_len));
	}

	void update_receive(void);
	void update_send(void);

private:

	int8_t get_nei_index(uint16_t addr);

	uint16_t send_protocol(const uint8_t *message, uint16_t len);
	int8_t receive_protocol(void);
	uint16_t xbee_write(void);

	AP_HAL::UARTDriver* _uart;
	uint16_t target_adr;
	
	uint8_t send_buf[XBEEMAXLEN];
	uint16_t send_buf_len;
	
	uint8_t receive_num;
	
	uint8_t xbee_data[NEIGHBOUR_NUM][XBEEMAXLEN];
	uint16_t xbee_data_len[NEIGHBOUR_NUM];
	uint16_t xbee_nei_mask = 0;
	
};

extern Xbee_Protocol xbee;

//void print_buf(xbee_protocol& x);
