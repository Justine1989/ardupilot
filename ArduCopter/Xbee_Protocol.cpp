#include "Xbee_Protocol.h"
#include "Copter.h"

uint16_t Xbee_Protocol::send_protocol(const uint8_t *message, uint16_t len)
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
	send_buf[5] = target_adr >> 8;
	sum += send_buf[5];
	send_buf[6] = target_adr & 0xFF;
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

int16_t Xbee_Protocol::receive_protocol(uint8_t *message, uint16_t *address)
{
	//uint32_t start_time = AP_HAL::micros();
	
	receive_buf_len = _uart->available();
	uint8_t sum = 0;
	
	while(_uart->available() > 8){
		receive_buf[0] = _uart->read();
		if(receive_buf[0] == 0x7E){
			for(uint16_t i = 1; i < 4; i++)
            	receive_buf[i] = _uart->read();
            if(receive_buf[3] == 0x81){
            	sum += receive_buf[3];
            	for(uint16_t i = 4; i < 8; i++){
            		sum += receive_buf[i];
                	receive_buf[i] = _uart->read();
                }
        		receive_adr = receive_buf[4] * 256 + receive_buf[5];
            	receive_len = receive_buf[1] * 256 + receive_buf[2] - 5;
            	for(uint16_t i = 8; i < receive_len + 8; i++){
            		sum += receive_buf[i];
            		receive_buf[i] = _uart->read();
            	}
            	if(0xFF-sum == _uart->read())
            		return receive_len;
            	else
            		return -1;
            }
        }
	}
	return 0;
}

uint16_t Xbee_Protocol::xbee_write(void)
{
	return _uart->write(send_buf, send_buf_len);
}


void Xbee_Protocol::update_receive(void)
{
	static uint32_t time_{0};
	uint32_t start_t = AP_HAL::micros();
	
	int16_t rev_len = receive_protocol(reveive_msg, &receive_adr);
	if(rev_len>0)
		receive_len = (uint16_t)rev_len;
		
	time_ = AP_HAL::micros() - start_t;
		
	if(rev_len>0){
		uint8_t send_time[2];
		send_time[0] = (uint8_t)time_>>8;
		send_time[1] = (uint8_t)time_&0xFF;
		send_protocol(send_time, 2);
		xbee_write();
	}
}

void Xbee_Protocol::update_send(void)
{
	uint8_t send_msg[]{'b','a','f','s'};
	send_protocol(send_msg, sizeof(send_msg));
	xbee_write();
}

Xbee_Protocol xbee;

/*void print_buf(xbee_protocol& x)
{
	for(uint8_t i = 0; i<x.send_buf_len; i++)
		hal.uartA->printf("%u ", x.send_buf[i]);
	hal.uartA->printf("\r\n");
}*/

