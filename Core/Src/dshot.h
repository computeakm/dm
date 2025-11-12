/*
 * dshot.h
 *
 *  Created on: 2021. 1. 27.
 *      Author: mokhwasomssi
 */


#ifndef __DSHOT_H__
#define __DSHOT_H__


#include "tim.h"    	// header from stm32cubemx code generate
#include <stdbool.h>	
#include <math.h>		// lrintf


/* User Configuration */
// Timer Clock
#define TIMER_CLOCK				270000000	// 550MHz

// MOTOR 1 (PA3) - TIM5 Channel 4, DMA1 Stream 0
#define MOTOR_1_TIM             (&htim1)
#define MOTOR_1_TIM_CHANNEL     TIM_CHANNEL_1

// MOTOR 2 (PA2) - TIM2 Channel 3, DMA1 Stream 1
#define MOTOR_2_TIM             (&htim1)
#define MOTOR_2_TIM_CHANNEL     TIM_CHANNEL_3

// MOTOR 3 (PA0) - TIM2 Channel 1, DMA1 Stream 2
#define MOTOR_3_TIM             (&htim2)
#define MOTOR_3_TIM_CHANNEL     TIM_CHANNEL_1

// MOTOR 4 (PA1) - TIM5 Channel 2, DMA1 Stream 3
#define MOTOR_4_TIM             (&htim2)
#define MOTOR_4_TIM_CHANNEL     TIM_CHANNEL_3


/* Definition */
#define MHZ_TO_HZ(x) 			((x) * 1000000)

#define DSHOT600_HZ     		MHZ_TO_HZ(12)
#define DSHOT300_HZ     		MHZ_TO_HZ(60)
#define DSHOT150_HZ     		MHZ_TO_HZ(30)
#define PWM_2_5MS           MHZ_TO_HZ(5)
#define ONESHOUT_125        MHZ_TO_HZ(2)

#define MOTOR_BIT_0            	66
#define MOTOR_BIT_1            	132
#define MOTOR_BITLENGTH        	200
#define PWM_MOTOR_BITLENGTH     12500
#define ONE_SHOT_125            1000  // 0.5ms



#define ONE_SHOT_125_MIN        250   //   0.125ms
#define ONE_SHOT_125_MAX        500   //   0.25ms


#define PWM_MOTOR_MIN         4500    // 0.9ms
#define PWM_MOTOR_MAX         10000   // 2ms

       


#define DSHOT_FRAME_SIZE       	16
#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */

#define DSHOT_MIN_THROTTLE      48
#define DSHOT_MAX_THROTTLE     	2047
#define DSHOT_RANGE 			(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)


/* Enumeration */
typedef enum
{
    DSHOT150,
    DSHOT300,
    DSHOT600,
		PWM,
    ONESHOT125
		
} dshot_type_e;


/* Functions */
void dshot_init(dshot_type_e dshot_type);
void dshot_write(uint16_t* motor_value);
void esc_init(dshot_type_e dshot_type);
void esc_set_speed(uint16_t* motor_value);
/* Static functions */



#endif /* __DSHOT_H__ */