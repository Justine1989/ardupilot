#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

//#ifdef XBEE_CONNECT2

#define XBEEMAXLEN 512

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

const uint16_t Neighbours_Frame_Len = sizeof(Neighbours_Pos);

union Neighbours_cor{
	Neighbours_Pos Pose;
	uint8_t cor_data[Neighbours_Frame_Len];
};


class Xbee_Protocol{
public:

	Xbee_Protocol(void){}
	
	void init_xbee(AP_HAL::UARTDriver* uart){
		_uart = uart;
		if (_uart != nullptr)
        	_uart->begin(57600, 512, 512);
        //memset(xbee_data, 0, sizeof(xbee_data));
        memset(xbee_data_len, 0, sizeof(xbee_data_len));
        memset(xbee_data_num, 0, sizeof(xbee_data_num));
	}

	void update_receive(void);
	void update_send(void);

private:

	int8_t get_nei_index(uint16_t addr);

	uint16_t send_protocol(const uint16_t addr, const uint8_t *message, uint16_t len);
	uint8_t receive_protocol(void);
	uint16_t xbee_write(void);

	AP_HAL::UARTDriver* _uart;
	
	uint8_t send_buf[XBEEMAXLEN];
	uint16_t send_buf_len;
	
	//uint8_t xbee_data[NEIGHBOUR_NUM][XBEEMAXLEN];//The datas of the newest frame.
	uint16_t xbee_data_len[NEIGHBOUR_NUM];//The length of the newest frame.
	uint16_t xbee_data_num[NEIGHBOUR_NUM];//The number of all frames which have received so far.
	uint16_t xbee_nei_mask = 0;//Check which neighbours of UAV that have sent the data and been reveived.
	uint8_t *xbee_data;
};

//extern Xbee_Protocol xbee;
//#endif
