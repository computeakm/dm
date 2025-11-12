#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "main.h"

#ifndef CRSF_H
#define CRSF_H

	typedef struct __attribute__((packed))
	{//length 0x18, type 0x16
		uint8_t type;
		unsigned ch0 : 11;
		unsigned ch1 : 11;
		unsigned ch2 : 11;
		unsigned ch3 : 11;
		unsigned ch4 : 11;
		unsigned ch5 : 11;
		unsigned ch6 : 11;
		unsigned ch7 : 11;
		unsigned ch8 : 11;
		unsigned ch9 : 11;
		unsigned ch10 : 11;
		unsigned ch11 : 11;
		unsigned ch12 : 11;
		unsigned ch13 : 11;
		unsigned ch14 : 11;
		unsigned ch15 : 11;
		uint8_t crc;
	} crsf_channels_t;

	typedef struct __attribute__((packed))
	{//length 0x0C, type 0x14
		uint8_t type;
	    uint8_t uplink_RSSI_1;
	    uint8_t uplink_RSSI_2;
	    uint8_t uplink_Link_quality;
	    int8_t uplink_SNR;
	    uint8_t active_antenna;
	    uint8_t rf_Mode;
	    uint8_t uplink_TX_Power;
	    uint8_t downlink_RSSI;
	    uint8_t downlink_Link_quality;
	    int8_t downlink_SNR;
	    uint8_t crc;
	} crsfLinkStatistics_t;

	typedef struct __attribute__((packed))
	{//length 0x0A, type 0x08
		uint8_t type;
	    unsigned voltage : 16;  // mv * 100 BigEndian
	    unsigned current : 16;  // ma * 100
	    unsigned capacity : 24; // mah
	    unsigned remaining : 8; // %
	    uint8_t crc;
	} crsf_sensor_battery_t;

	typedef struct __attribute__((packed))
	{//length 0x11, type 0x02
		uint8_t type;
		unsigned latitude : 32;		// ( degree / 10`000`000 )
		unsigned longitude : 32;  	// ( degree / 10`000`000 )
		unsigned groundspeed : 16; 	// ( km/h / 10 )
		unsigned heading : 16; 		// ( degree / 100 )
		unsigned altitude : 16; 	// ( meter ­1000m offset )
		uint8_t satellites;
		uint8_t crc;
	} crsf_sensor_gps_t;

	bool CRSF_RC(uint8_t *in, crsf_channels_t *rcDataAddr);
	bool CRSF_LQ(uint8_t *in, crsfLinkStatistics_t *lqDataAddr);

#endif
