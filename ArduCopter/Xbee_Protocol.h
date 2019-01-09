#pragma once

#include <AP_HAL/AP_HAL.h>


#define XBEEMAXLEN 300

class Xbee_Protocol{
public:

	Xbee_Protocol(uint16_t tag_adr = 0xFFFF):target_adr(tag_adr){}
	
	void init_xbee(AP_HAL::UARTDriver* uart){
		_uart = uart;
		if (_uart != nullptr)
        	_uart->begin(57600);
	}

	uint16_t send_protocol(const uint8_t *message, uint16_t len);
	int16_t receive_protocol(uint8_t *message, uint16_t *address);
	
	uint16_t xbee_write(void);
	
	void update_receive(void);
	void update_send(void);

private:

	AP_HAL::UARTDriver* _uart;

	uint16_t target_adr;
	uint8_t send_buf[XBEEMAXLEN];
	uint16_t send_buf_len;
	uint8_t receive_buf[XBEEMAXLEN];
	uint16_t receive_buf_len;
	uint8_t reveive_msg[XBEEMAXLEN];
	uint16_t receive_len;
	uint16_t receive_adr;
	
};

extern Xbee_Protocol xbee;

//void print_buf(xbee_protocol& x);
