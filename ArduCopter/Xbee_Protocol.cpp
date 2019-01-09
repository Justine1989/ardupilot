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

int16_t Xbee_Protocol::receive_protocol(uint8_t *buf, uint16_t len, uint8_t *message, uint16_t *address)
{
	uint8_t sum = 0;
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
	uint16_t message_len{4}, j{0};
	for(size_t i = 0; i < len; i++){
		if(buf[i]==0x7E)
			status = START;
		switch(status){
			case START:
				status = LENGTH;
				break;
			case LENGTH:
				message_len = buf[i++];
				message_len <<= 8;
				message_len |= buf[i];
				status = FRAME;
				break;
			case FRAME:
				if(buf[i]==0x81){
					sum += buf[i];
					status = ADDRESS;
				}
				break;
			case ADDRESS:
				sum += buf[i];
				*address = buf[i++];
				*address <<= 8;
				sum += buf[i];
				*address |= buf[i];
				status = RSSI;
				break;
			case RSSI:
				sum += buf[i];
				status = OPTIONS;
				break;
			case OPTIONS:
				sum += buf[i];
				status = DATA;
				break;
			case DATA:
				sum += buf[i];
				message[j++] = buf[i];
				if(j>=message_len-5)
					status = CHECKSUM;
				break;
			case CHECKSUM:
				if(0xFF-sum==buf[i])
					status = END;
				else
					status = ERROR;
				break;
			default:
				break;
		}
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
	receive_buf_len = _uart->available();
	
	for(size_t i = 0; i < receive_buf_len; i++)
		receive_buf[i] = _uart->read();
	
	int16_t rev_len = receive_protocol(receive_buf, receive_buf_len, reveive_msg, &receive_adr);
	if(rev_len>0)
		receive_len = (uint16_t)rev_len;
}

Xbee_Protocol xbee;

/*void print_buf(xbee_protocol& x)
{
	for(uint8_t i = 0; i<x.send_buf_len; i++)
		hal.uartA->printf("%u ", x.send_buf[i]);
	hal.uartA->printf("\r\n");
}*/

