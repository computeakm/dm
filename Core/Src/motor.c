#include "tim.h"
#include "usart.h"
#include "motor.h"
#include "stdio.h"
#include "string.h"
char escstr[50];

uint16_t Dshot_600_CRC(uint16_t data)
{
	uint16_t crc;
	uint16_t data1;
	crc=(data^(data>>4)^(data >>8))&0x0f;
	data1=data+crc;
	return data1;
}

void Dshot_600_H(TIM_HandleTypeDef * TIM,uint32_t CHANNEL)
{
	__HAL_TIM_SET_COMPARE(TIM,CHANNEL,12);
}

void Dshot_600_L(TIM_HandleTypeDef * TIM,uint32_t CHANNEL)
{
	__HAL_TIM_SET_COMPARE(TIM,CHANNEL,6);
}

void Dshot_600_SET_DATA_END(TIM_HandleTypeDef * TIM,uint32_t CHANNEL)
{
	__HAL_TIM_SET_COMPARE(TIM,CHANNEL,0);
}

void Dshot_600_SET_DATA(uint16_t SPEED,TIM_HandleTypeDef * TIM,uint32_t CHANNEL,uint8_t TELEMETRY)
{
	
	uint16_t DATA=0;
	uint16_t SPEED_DATA=SPEED;
	if(SPEED_DATA>2047)
	{
		SPEED_DATA=2047;
	}
	SPEED_DATA=SPEED_DATA<<1;
	if(TELEMETRY)
	{
		SPEED_DATA=SPEED_DATA+1;
	}
	SPEED_DATA=SPEED_DATA<<4;
	DATA=Dshot_600_CRC(SPEED_DATA);
	for(int i=15;i>=0;i--)
	{
		if((DATA>>i)&0x0001)
		{
			Dshot_600_H(TIM,CHANNEL);
			
		}else
		{
			Dshot_600_L(TIM,CHANNEL);
			
		}
//		sprintf(escstr,"i: %X\r\n",(DATA>>i)&0x01);
//		HAL_UART_Transmit(&huart1, (uint8_t*)escstr, strlen(escstr), 0xff);
	}
	Dshot_600_SET_DATA_END(TIM,CHANNEL);
	sprintf(escstr,"DATA: %X\r\n",DATA);
	HAL_UART_Transmit(&huart1, (uint8_t*)escstr, strlen(escstr), 0xff);
}

/*
void ESC_inte(void)
{
	HAL_TIM_PWM_Start(&htim1,Motor_CH1);
	HAL_TIM_PWM_Start(&htim1,Motor_CH2);
	HAL_TIM_PWM_Start(&htim2,Motor_CH3);
	HAL_TIM_PWM_Start(&htim2,Motor_CH4);
	Dshot_600_SET_DATA(0,&htim1,Motor_CH1,0);
	Dshot_600_SET_DATA(0,&htim1,Motor_CH2,0);
	Dshot_600_SET_DATA(0,&htim2,Motor_CH3,0);
	Dshot_600_SET_DATA(0,&htim2,Motor_CH4,0);
	HAL_Delay(1000);
}


	
		void ESC_ON(void)
{
	
	Dshot_600_SET_DATA(0,&htim1,Motor_CH1,0);
	Dshot_600_SET_DATA(0,&htim1,Motor_CH2,0);
	Dshot_600_SET_DATA(0,&htim2,Motor_CH3,0);
	Dshot_600_SET_DATA(0,&htim2,Motor_CH4,0);
	
	HAL_Delay(200);
	Dshot_600_SET_DATA(48,&htim2,Motor_CH4,0);
	HAL_Delay(200);
	Dshot_600_SET_DATA(0,&htim2,Motor_CH4,0);
	
	
}
*/

void ESC_inte(void)
{
	HAL_TIM_PWM_Start(&htim1,Motor_CH1);
	HAL_TIM_PWM_Start(&htim1,Motor_CH2);
	HAL_TIM_PWM_Start(&htim2,Motor_CH3);
	HAL_TIM_PWM_Start(&htim2,Motor_CH4);
	__HAL_TIM_SET_COMPARE(&htim1, Motor_CH1,2000);//BBB
	__HAL_TIM_SET_COMPARE(&htim1, Motor_CH2,2000);//BBB
	__HAL_TIM_SET_COMPARE(&htim2, Motor_CH3,2000);//BBB
	__HAL_TIM_SET_COMPARE(&htim2, Motor_CH4,2000);//BBB
	HAL_Delay(10000);
//	
	
	
	__HAL_TIM_SET_COMPARE(&htim1, Motor_CH1,0);//BBB
	__HAL_TIM_SET_COMPARE(&htim1, Motor_CH2,0);//BBB
	__HAL_TIM_SET_COMPARE(&htim2, Motor_CH3,0);//BBB
	__HAL_TIM_SET_COMPARE(&htim2, Motor_CH4,0);//BBB
	HAL_Delay(10000);
	__HAL_TIM_SET_COMPARE(&htim1, Motor_CH1,1000);//BB
	__HAL_TIM_SET_COMPARE(&htim1, Motor_CH2,1000);//BB
	__HAL_TIM_SET_COMPARE(&htim2, Motor_CH3,1000);//BB
	__HAL_TIM_SET_COMPARE(&htim2, Motor_CH4,1000);//BB
		HAL_Delay(10000);
}


void ESC_ON(void)
{
	
	__HAL_TIM_SET_COMPARE(&htim1, Motor_CH1,2000);//BBB
	__HAL_TIM_SET_COMPARE(&htim1, Motor_CH2,2000);//BBB
	__HAL_TIM_SET_COMPARE(&htim2, Motor_CH3,2000);//BBB
	__HAL_TIM_SET_COMPARE(&htim2, Motor_CH4,2000);//BBB
	HAL_Delay(200);
}

void ESC_STE_Speed(int speed_CH1,int speed_CH2,int speed_CH3,int speed_CH4)
{
	int speed[4];
	
	
	
	speed[0]=MIN_SPEED+speed_CH1;
	speed[1]=MIN_SPEED+speed_CH2;
	speed[2]=MIN_SPEED+speed_CH3;
	speed[3]=MIN_SPEED+speed_CH4;
	
	if(speed[0]>MAX_SPEED)
	{
		speed[0]=MAX_SPEED;
	}else if(speed[0]<MIN_SPEED)
	{
		speed[0]=MIN_SPEED;
	}
	
		if(speed[1]>MAX_SPEED)
	{
		speed[1]=MAX_SPEED;
	}else if(speed[1]<MIN_SPEED)
	{
		speed[1]=MIN_SPEED;
	}
	
		if(speed[2]>MAX_SPEED)
	{
		speed[2]=MAX_SPEED;
	}else if(speed[2]<MIN_SPEED)
	{
		speed[2]=MIN_SPEED;
	}
	
		if(speed[3]>MAX_SPEED)
	{
		speed[3]=MAX_SPEED;
	}else if(speed[3]<MIN_SPEED)
	{
		speed[3]=MIN_SPEED;
	}
	
	
	
	__HAL_TIM_SetCompare(&htim1,Motor_CH1,speed[0]);
	__HAL_TIM_SetCompare(&htim1,Motor_CH2,speed[1]);
	__HAL_TIM_SetCompare(&htim2,Motor_CH3,speed[2]);
	__HAL_TIM_SetCompare(&htim2,Motor_CH4,speed[3]);
}


void ESC_OFF(void)
{
	
	__HAL_TIM_SetCompare(&htim1,Motor_CH1,0);
	__HAL_TIM_SetCompare(&htim1,Motor_CH2,0);
	__HAL_TIM_SetCompare(&htim2,Motor_CH3,0);
	__HAL_TIM_SetCompare(&htim2,Motor_CH4,0);
		HAL_Delay(200);
}



void ESC_MAIN(int *speed,uint8_t* motor_flag)
{
	if (*motor_flag==2)
	{
		ESC_STE_Speed(speed[0],speed[1],speed[2],speed[3]);
	}else if (*motor_flag==1)
	{
		ESC_OFF();
	}else if(*motor_flag==0)
	{
		ESC_ON();
		*motor_flag=2;
		
	}
}

 
 
//定时器 4分频72/4=18mhz ;分频不固定，可自行调整
//pwm波周期 1.67us ；对应的pwm分辨率 1.67us /（1/18）= 30；
// 0.625us/(1/18) = 11; 0的占空比11/30
// 1.250us/(1/18) = 22; 1的占空比22/30
 
#define ESC_BIT_0     7
#define ESC_BIT_1     13  
#define ESC_CMD_BUF_LEN 18 
uint16_t ESC_CMD[ESC_CMD_BUF_LEN]={0};
 
 
//定时器 1分频32/1=32mhz ;分频不固定，可自行调整  记一次数就是1/32 us
//pwm波周期 1.67us ；对应的pwm分辨率 1.67us /（1/32）= 53.44；
// 0.625us/(1/32) = 20; 0的占空比20/53
// 1.250us/(1/32) = 40; 1的占空比40/53
 
// #define ESC_BIT_0     20
// #define ESC_BIT_1     40  
 
/*************************************************************
** Function name:       prepareDshotPacket
** Descriptions:        CRC校验以及要不要回传信息
** Input parameters:    value: 油门大小 注意取值范围, requestTelemetry 是否请求回传数据
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
static uint16_t prepareDshotPacket(const uint16_t value, int requestTelemetry)
{
    // 油门大小为11位  所以这里先左移一位 添加上请求回传标志共12位
    uint16_t packet = (value << 1) | (requestTelemetry ? 1 : 0);
 
    // 将12位数据分为3组 每组4位, 进行异或
    // compute checksum
    int csum = 0;
    int csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    //取最后四位 其他的不要 
    csum &= 0xf;
    // append checksum 将CRC添加到后四位
    packet = (packet << 4) | csum;
    return packet;
}
 
 
/*************************************************************
** Function name:       pwmWriteDigital
** Descriptions:        根据输入 填充esc_cmd,填充的数值代表每一位的高低电平
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void pwmWriteDigital(uint16_t *esc_cmd, uint16_t value)
{
	value = ( (value > 2047) ? 2047 : value );
	value = prepareDshotPacket(value, 0);
    esc_cmd[0]  = (value & 0x8000) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[1]  = (value & 0x4000) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[2]  = (value & 0x2000) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[3]  = (value & 0x1000) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[4]  = (value & 0x0800) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[5]  = (value & 0x0400) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[6]  = (value & 0x0200) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[7]  = (value & 0x0100) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[8]  = (value & 0x0080) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[9]  = (value & 0x0040) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[10] = (value & 0x0020) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[11] = (value & 0x0010) ? ESC_BIT_1 : ESC_BIT_0; 	
    esc_cmd[12] = (value & 0x8) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[13] = (value & 0x4) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[14] = (value & 0x2) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[15] = (value & 0x1) ? ESC_BIT_1 : ESC_BIT_0;
 
    HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_3,(uint32_t *)esc_cmd,ESC_CMD_BUF_LEN);
}
 
 
/*************************************************************
** Function name:       HAL_TIM_PWM_PulseFinishedCallback
** Descriptions:        每次DMA发送完后会进这个回调函数，可以做标记。要记得关闭DMA
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
