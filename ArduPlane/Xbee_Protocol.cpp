#include "Xbee_Protocol.h"
#include "Plane.h"

//#ifdef XBEE_CONNECT2
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
            	if(receive_len != Neighbours_Frame_Len)
            		continue;
            	sum = receive_buf[3];
            	for(uint16_t i = 4; i < 8; i++){
                	receive_buf[i] = _uart->read();
                	sum += receive_buf[i];
                }
        		receive_adr = receive_buf[4] * 256 + receive_buf[5];
        		int8_t nei_index = get_nei_index(receive_adr);
        		if(nei_index<0)
        			continue;
        		xbee_data_len[nei_index] = receive_len;
        		xbee_nei_mask |= (1U<<nei_index);
        		plane.gcs().update_neighbours_mask(xbee_nei_mask);
        		xbee_data = (uint8_t*)plane.gcs().update_neighbours_pose(nei_index);
            	for(uint16_t i = 0; i < receive_len; i++){
            		xbee_data[i] = _uart->read();
            		sum += xbee_data[i];
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
//	comm_send_lock((mavlink_channel_t)4);
	uint16_t send_len = _uart->write(send_buf, send_buf_len);
	//uint16_t send_len = mavlink_comm_port[4]->write(send_buf, send_buf_len);
//	comm_send_unlock((mavlink_channel_t)4);
	return send_len;
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
	xbee_nei_mask = 0;
	/*uint8_t rev_len = */receive_protocol();
	
	/*if(rev_len>0){
		send_protocol(0xE1E1, xbee_data[1], xbee_data_len[1]);
		xbee_write();
	}*/
	
	//time_ = AP_HAL::micros() - start_t;
	
}

void Xbee_Protocol::update_send(void)
{
    union Neighbours_cor xbee_send_data;
    xbee_send_data.Pose.time_boot_ms = AP_HAL::millis();
    xbee_send_data.Pose.lat = plane.current_loc.lat;
    xbee_send_data.Pose.lon = plane.current_loc.lng;
    xbee_send_data.Pose.alt = plane.current_loc.alt*10UL;
    xbee_send_data.Pose.relative_alt = plane.relative_altitude*1000.0f;
	const Vector3f &vel = plane.gps.velocity();
    xbee_send_data.Pose.vx = vel.x * 100;
    xbee_send_data.Pose.vy = vel.y * 100;
    xbee_send_data.Pose.vz = vel.z * 100;
    xbee_send_data.Pose.hdg = plane.ahrs.yaw_sensor;
    
    send_protocol(0xFFFF, xbee_send_data.cor_data, Neighbours_Frame_Len);
    xbee_write();
    
	/*uint8_t send_msg[2];
	static uint16_t old_num[2]{0,0};
	if(old_num[0]<xbee_data_num[0]){
		send_msg[0] = (uint8_t)(xbee_data_num[0]/256);
		send_msg[1] = (uint8_t)(xbee_data_num[0]&0xFF);
		send_protocol(0xE0E0, send_msg, 2);
		xbee_write();
	}
	if(old_num[1]<xbee_data_num[1]){
		send_msg[0] = (uint8_t)(xbee_data_num[1]/256);
		send_msg[1] = (uint8_t)(xbee_data_num[1]&0xFF);
		send_protocol(0xE1E1, send_msg, 2);
		xbee_write();
	}
	old_num[0] = xbee_data_num[0];
	old_num[1] = xbee_data_num[1];*/
}

//Xbee_Protocol xbee;
//#endif

