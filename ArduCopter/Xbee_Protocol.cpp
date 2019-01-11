#include "Xbee_Protocol.h"
#include "Copter.h"

uint16_t Xbee_Protocol::send_protocol(const uint16_t addr, const uint8_t *message, uint16_t len)
{
	uint8_t sum = 0;
	uint16_t i;
	
	send_buf[0] = 0x7E;
	send_buf[1] = (5+len) >> 8;
	send_buf[2] = (5+len) & 0xFF;
	send_buf[3] = 0x01;
	sum += send_buf[3];
	send_buf[4] = 0x00;
	sum += send_buf[4];
	send_buf[5] = addr >> 8;
	sum += send_buf[5];
	send_buf[6] = addr & 0xFF;
	sum += send_buf[6];
	send_buf[7] = 0x00;
	sum += send_buf[7];
	for(i = 8; i<len+8; i++){
		send_buf[i] = message[i-8];
		sum += send_buf[i];
	}
	send_buf[i] = 0xFF-sum;
	send_buf_len = i+1;
	
	return send_buf_len;
}

uint8_t Xbee_Protocol::receive_protocol(void)
{
	//uint32_t start_time = AP_HAL::micros();
	uint8_t frame_num{0};
	uint8_t sum{0}, receive_buf[8];
	uint16_t receive_len{0}, receive_adr{0};
	
	while(_uart->available()>8){
		receive_buf[0] = _uart->read();
		if(receive_buf[0] == 0x7E){
			for(uint8_t i = 1; i < 4; i++)
            	receive_buf[i] = _uart->read();
            if(receive_buf[3] == 0x81){
            	receive_len = receive_buf[1] * 256 + receive_buf[2] - 5;
            	sum = receive_buf[3];
            	for(uint16_t i = 4; i < 8; i++){
                	receive_buf[i] = _uart->read();
                	sum += receive_buf[i];
                }
        		receive_adr = receive_buf[4] * 256 + receive_buf[5];
        		int8_t nei_index = get_nei_index(receive_adr);
        		if(nei_index>=0){
        			xbee_data_len[nei_index] = receive_len;
        			xbee_nei_mask |= (1U<<nei_index);
        		}else
        			continue;
            	for(uint16_t i = 0; i < receive_len; i++){
            		xbee_data[nei_index][i] = _uart->read();
            		sum += xbee_data[nei_index][i];
            	}
            	if(0xFF-sum == _uart->read()){
            		frame_num++;
            		xbee_data_num[nei_index]++;
            	}
            }
        }
	}
	return frame_num;
}

uint16_t Xbee_Protocol::xbee_write(void)
{
	return _uart->write(send_buf, send_buf_len);
}

int8_t Xbee_Protocol::get_nei_index(uint16_t addr)
{
	switch((enum Neighbours_Addr)addr){
		case NEI0:
			return 0;
		case NEI1:
			return 1;
		case NEI2:
			return 2;
		case NEI3:
			return 3;
		case NEI4:
			return 4;
		case NEI5:
			return 5;
		case NEI6:
			return 6;
		case NEI7:
			return 7;
		case NEI8:
			return 8;
		case NEI9:
			return 9;
		default:
			return -1;
	}
}


void Xbee_Protocol::update_receive(void)
{
	//static uint32_t time_{0};
	//uint32_t start_t = AP_HAL::micros();
	
	receive_protocol();
	
	//time_ = AP_HAL::micros() - start_t;
	
}

void Xbee_Protocol::update_send(void)
{
	uint8_t send_msg[2];
	static uint16_t old_num[2]{0,0};
	if(old_num[0]<xbee_data_num[0]){
		send_msg[0] = (uint8_t)(xbee_data_num[0]/256);
		send_msg[1] = (uint8_t)(xbee_data_num[0]&0xFF);
		send_protocol(0xE0E0, send_msg, 2);
		xbee_write();
	}
	if(old_num[1]<xbee_data_num[2]){
		send_msg[0] = (uint8_t)(xbee_data_num[2]/256);
		send_msg[1] = (uint8_t)(xbee_data_num[2]&0xFF);
		send_protocol(0xE2E2, send_msg, 2);
		xbee_write();
	}
	old_num[0] = xbee_data_num[0];
	old_num[1] = xbee_data_num[2];
}

Xbee_Protocol xbee;


