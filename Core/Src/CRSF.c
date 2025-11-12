#include "CRSF.h"

extern CRC_HandleTypeDef hcrc;

const uint8_t head_rc[3] = {0xC8, 0x18, 0x16};
const uint8_t head_lq[3] = {0xC8, 0x0C, 0x14};

bool CRSF_RC(uint8_t *in, crsf_channels_t *rcDataAddr)
{
	int8_t lastpos = -1;//init lastpos
	for (uint8_t i = 0; i <= 38; i ++)
	{//looking for head,先比对，由于只有64缓存，到38字节还没有说明没了，下一帧再看吧
		if (memcmp(&head_rc, &in[i], 3) == 0)
		{//如果能和头对上，说明对的
			lastpos = i;//save last head postion,记录最后一次发现帧头的节号
		}
	}
	
	
	if (lastpos >= 0)
	{//if lastpos >= 0, has head data
		memcpy(rcDataAddr, &in[lastpos + 2], 24);//copy to rcData
			memset(&in[lastpos], 0, 26);//reset input array
//		if (in[lastpos + 25] == HAL_CRC_Calculate(&hcrc, &in[lastpos + 2], 23))
//		{//check crc
//			memcpy(rcDataAddr, &in[lastpos + 2], 24);//copy to rcData
//			memset(&in[lastpos], 0, 26);//reset input array
//			return 1;//has data return 1
//		}
	}
	return 0;//if there is no rc data, return 0
}

bool CRSF_LQ(uint8_t *in, crsfLinkStatistics_t *lqDataAddr)
{
	int8_t lastpos = -1;//init lastpos
	for (uint8_t i = 0; i <= 50; i ++)
	{//先比对，由于只有64缓存，到50字节还没有说明没了，下一帧再看吧
		if (memcmp(&head_lq, &in[i], 3) == 0)
		{//如果能和头对上，说明对的
			lastpos = i;//记录当前字节号
		}
	}
	if (lastpos >= 0)
	{//如果大于0，说明的确有数据
		
		
		memcpy(lqDataAddr, &in[lastpos + 2], 12);//拷贝到RC数据中
			memset(&in[lastpos], 0, 14);//清空这一帧数据地址下的内存
		
//		if (in[lastpos + 13] == HAL_CRC_Calculate(&hcrc, &in[lastpos + 2], 11))
//		{//如果能对上CRC
//			memcpy(lqDataAddr, &in[lastpos + 2], 12);//拷贝到RC数据中
//			memset(&in[lastpos], 0, 14);//清空这一帧数据地址下的内存
//			return 1;//返回1，说明有了
//		}
	}
	return 0;//if there is no rc data, return 0
}

void crsf_data_reset(uint8_t *in)
{
	memset(in, 0, 64);
}