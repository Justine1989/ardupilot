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
	receive_buf_len = _uart->available();
	uint8_t buf, sum = 0;
	enum STATUS{
		START,
		LENGTH,
		FRAME,
		ADDRESS,
		RSSI,
		OPTIONS,
		DATA,
		CHECKSUM,
		END,
		ERROR
	} status = ERROR;
	uint16_t message_len{4};
	for(size_t i = 0; i < receive_buf_len; i++){
		buf = _uart->read();
		if(buf==0x7E)
			status = START;
		switch(status){
			case START:
				status = LENGTH;
				break;
			case LENGTH:
				message_len = buf;
				message_len <<= 8;
				buf = _uart->read();
				message_len |= buf;
				status = FRAME;
				break;
			case FRAME:
				if(buf==0x81){
					sum += buf;
					status = ADDRESS;
				}
				break;
			case ADDRESS:
				sum += buf;
				*address = buf;
				*address <<= 8;
				buf = _uart->read();
				sum += buf;
				*address |= buf;
				status = RSSI;
				break;
			case RSSI:
				sum += buf;
				status = OPTIONS;
				break;
			case OPTIONS:
				sum += buf;
				status = DATA;
				break;
			case DATA:
				sum += buf;
				message[0] = buf;
				for(size_t j = 1; j<(message_len-5); j++){
					buf = _uart->read();
					sum += buf;
					message[j] = buf;
				}
				status = CHECKSUM;
				break;
			case CHECKSUM:
				if(0xFF-sum==buf)
					status = END;
				else
					status = ERROR;
				break;
			default:
				break;
		}
		if(status==END)
			break;
	}
	if(status==END)
		return (int16_t)message_len-5;
	else
		return -1;
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
		reveive_msg[receive_len+1] = (uint8_t)time_&0xFF;
		reveive_msg[receive_len] = (uint8_t)time_>>8;
		send_protocol(reveive_msg, receive_len+2);
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

