#include "communicate.h"
#include "sys.h"

void send_to_server(position_sensor_data data){
	
	unsigned char buffer[36];
	int i = 0;
	uint16_t checksum;
	
	buffer[i++] = 254;
	buffer[i++] = 28;
	buffer[i++] = 1;
	buffer[i++] = 255;
	buffer[i++] = 190;
	buffer[i++] = 33;
	buffer[i++] = data.time_boot_ms >> 0;
	buffer[i++] = data.time_boot_ms >> 8;
	buffer[i++] = data.time_boot_ms >> 16;
	buffer[i++] = data.time_boot_ms >> 24;
	buffer[i++] = data.lat >> 0;
	buffer[i++] = data.lat >> 8;
	buffer[i++] = data.lat >> 16;
	buffer[i++] = data.lat >> 24;
	buffer[i++] = data.lon >> 0;
	buffer[i++] = data.lon >> 8;
	buffer[i++] = data.lon >> 16;
	buffer[i++] = data.lon >> 24;
	buffer[i++] = data.alt >> 0;
	buffer[i++] = data.alt >> 8;
	buffer[i++] = data.alt >> 16;
	buffer[i++] = data.alt >> 24;
	buffer[i++] = data.relative_lat >> 0;
	buffer[i++] = data.relative_lat >> 8;
	buffer[i++] = data.relative_lat >> 16;
	buffer[i++] = data.relative_lat >> 24;
	buffer[i++] = data.vx >> 0;
	buffer[i++] = data.vx >> 8;
	buffer[i++] = data.vy >> 0;
	buffer[i++] = data.vy >> 8;
	buffer[i++] = data.vz >> 0;
	buffer[i++] = data.vz >> 8;
	buffer[i++] = data.vx >> 0;
	buffer[i++] = data.hdg >> 0;
	buffer[i++] = data.hdg >> 8;
	
	const char *packet = &buffer[6];
	checksum = crc_calculate((const uint8_t*)&buffer[1], 5);
	crc_accumulate_buffer(&checksum,  packet,28);
	crc_accumulate(104, &checksum);
	buffer[i++] = (uint8_t)(checksum & 0xFF);
	buffer[i++] = (uint8_t)(checksum >> 8);
	
	

}