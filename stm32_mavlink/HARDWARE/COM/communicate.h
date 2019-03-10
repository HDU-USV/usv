#include <stdint.h>
//#include "stdafx.h"
#include <stdlib.h>
//
#include "sys.h"
void send_to_server(position_sensor_data data);
#define X25_INIT_CRC 0xffff
/** 
 * Calculating CRC-16 in 'C' 
 * @para addr, start of data 
 * @para num, length of data 
 * @para crc, incoming CRC 
 */  
void send_to_server();
static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
        /*Accumulate one byte of data into the CRC*/
        uint8_t tmp;

        tmp = data ^ (uint8_t)(*crcAccum &0xff);
        tmp ^= (tmp<<4);
        *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}

static inline void crc_init(uint16_t* crcAccum)
{
        *crcAccum = X25_INIT_CRC;
}

static inline uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
        uint16_t crcTmp;
        crc_init(&crcTmp);
    while (length--) {
                crc_accumulate(*pBuffer++, &crcTmp);
        }
        return crcTmp;
}

static inline void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
    const uint8_t *p = (const uint8_t *)pBuffer;
    while (length--) {
                crc_accumulate(*p++, crcAccum);
        }
}
