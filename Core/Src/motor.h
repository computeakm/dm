#ifndef MOTOR_H
#define MOTOR_H


#include "main.h"




#define Motor_Tim htim1 
#define Motor_CH1 TIM_CHANNEL_1
#define Motor_CH2 TIM_CHANNEL_3
#define Motor_CH3 TIM_CHANNEL_1
#define Motor_CH4 TIM_CHANNEL_3
#define CUNDTER_DATA 20000
#define ON_TIME   1000
#define OFF_TIME  0
#define MAX_SPEED 2500
#define MIN_SPEED 1500


void ESC_inte(void);
void ESC_ON(void);
void ESC_STE_Speed(int speed_CH1,int speed_CH2,int speed_CH3,int speed_CH4);
void ESC_OFF(void);
void ESC_MAIN(int *speed,uint8_t* motor_flag);
void pwmWriteDigital(uint16_t *esc_cmd, uint16_t value);

#endif
