#include "buzzer.h"
#include "tim.h"

void buzzer_mod(int flag)
{
    switch (flag)
    {
    case 1:
      __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,200);
      break;
    case 2:
    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,50);
    break;
    
    case 0:
    __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);
    break;

    default:
      break;
    }
  
}


void buzzer_init(void)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    buzzer_mod(0);
}

void buzzer_deinit(void)
{
    HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
}

void buzzer_test(void)
{
    buzzer_mod(1);
    HAL_Delay(500);
    buzzer_mod(0);
    HAL_Delay(500);
    buzzer_mod(2);
    HAL_Delay(500);
    buzzer_mod(0);
    HAL_Delay(500);
}
/* USER CODE END 1 */