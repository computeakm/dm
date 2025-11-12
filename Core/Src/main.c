/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "fdcan.h"
#include "memorymap.h"
#include "octospi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "PID.h"
#include "BMI088Middleware.h"
#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include <stdio.h>
#include "CRSF.h"
#include "motor.h"
#include "dshot.h"
#include "ws2812.h"
#include "buzzer.h"
#include "ekf_attitude.h"
#include "usbd_cdc_if.h"
#include "filter_rc_bandpass.h"
#include "mtf01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int cotrol_mode=1; //0:sleep 1:speed 2:position
extern float gyro[3], accel[3],temperate;
extern float accel_x, accel_y, accel_z;
extern float gyro_x, gyro_y, gyro_z;
extern float temperate;
char str[200];
uint16_t EMC[18]={0};
uint8_t uartRx[16];
extern uint8_t uartTemp[64];
extern crsf_channels_t rcData;
extern crsfLinkStatistics_t lqData;
extern int  uartUpdate;
PIDController x_speed_pid,x_Angle,y_speed_pid,y_Angle,z_speed_pid,z_Angle,hight_speed_pid,hight_position_pid,x_position_pid,y_position_pid;
PIDController_Angle pid_angle_speed;
PIDController_Angle pid_angle_position;
PIDController pid_temp;
MahonyAHRS mahonyAHRS;
int bim088_init_state = 0;
int flag=0;
float ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9,ch10,ch11,ch12,ch13,ch14,ch15;
extern float q_gworld[4];
extern float v_gworld[4];  
extern float v_body[3];
extern float v_body_lpf[3];
extern uint8_t tx_sreed[11];
int num;
extern uint32_t motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
extern uint32_t motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
extern uint32_t motor3_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
extern uint32_t motor4_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
int Buzzer_hz;
AccelCalibParam accelCalibParam;
extern float a[3];
extern int losstime;
extern uint16_t speed[4];
uint16_t err_speed[4]={0,0,0,0};
uint16_t test_speed[4]={4500,4500,4500,4500};
uint16_t err2_speed[4]={48,48,48,48};
extern bool dshot;
uint16_t speed1[4]; 
uint8_t u10_rx_data[30];
extern  MICOLINK_PAYLOAD_RANGE_SENSOR_t mtf_01;
extern MICOLINK_MSG_t msg;
extern float angle[3];
FilterRCBandpass bandpass_filter_ax;
FilterRCBandpass bandpass_filter_ay;
FilterRCBandpass bandpass_filter_az;
FilterRCBandpass bandpass_filter_gx;
FilterRCBandpass bandpass_filter_gy;
FilterRCBandpass bandpass_filter_gz;

FilterRCBandstop bandstop_filter_gx;
FilterRCBandstop bandstop_filter_gy;
FilterRCBandstop bandstop_filter_gz;
FilterRCBandstop bandstop_filter_ax;
FilterRCBandstop bandstop_filter_ay;
FilterRCBandstop bandstop_filter_az;

FilterRCBandstop bandstop_filter_gx_2;
FilterRCBandstop bandstop_filter_gy_2;
FilterRCBandstop bandstop_filter_gz_2;
FilterRCBandstop bandstop_filter_ax_2;
FilterRCBandstop bandstop_filter_ay_2;
FilterRCBandstop bandstop_filter_az_2;


FilterRCLowpass lowpass_filter_accel_x;
FilterRCLowpass lowpass_filter_accel_y;
FilterRCLowpass lowpass_filter_accel_z;
FilterRCLowpass lowpass_filter_gyro_x;
FilterRCLowpass lowpass_filter_gyro_y;
FilterRCLowpass lowpass_filter_gyro_z;
FilterRCLowpass lowpass_filter_speed_x;
FilterRCLowpass lowpass_filter_speed_y;
FilterRCLowpass lowpass_filter_speed_z;
MovAvgState mov_avg_speed_x;
MovAvgState mov_avg_speed_y;
MovAvgState mov_avg_speed_z;
float mov_avg_speed_x_buffer[10];
float mov_avg_speed_y_buffer[10];
float mov_avg_speed_z_buffer[10];
MovAvgState msg_speed_x_filter;
MovAvgState msg_speed_y_filter;
MovAvgState msg_speed_z_filter;
float msg_speed_x_filter_buffer[10];
float msg_speed_y_filter_buffer[10];
float msg_speed_z_filter_buffer[10];

extern UAV_xyz word_speed;
extern  UAV_xyz body_speed;
extern UAV_xyz mtf_01_speed;
extern UAV_xyz word_speed2;
extern UAV_xyz word_accle;
extern UAV_xyz body_accle;
extern UAV_xyz UAV_position;
extern UAV_xyz body_speed2;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN3_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_UART7_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_OCTOSPI2_Init();
  MX_SPI6_Init();
  MX_USB_DEVICE_Init();
  MX_USART10_UART_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
	WS2812_Ctrl(10,0,0);
	float k=0.5;
  PIDController_Init(&x_speed_pid,0.2f, k*14.2f, k*0.0f, k*0.0f, 0.02f, -50.0f, 50.0f, -50.0f, 50.0f, 0.001f);
  PIDController_Init(&y_speed_pid,0.2f, k*17.2f, k*0.0f, k*0.0f, 0.02f, -50.0f, 50.0f, -50.0f, 50.0f, 0.001f);
  PIDController_Init(&z_speed_pid,0.5f, 35.0f, 0.0f, 0.0f, 0.02f, -50.0f, 50.0f, -5.0f, 5.0f, 0.001f);
	
  PIDController_Init(&x_Angle,7.0f, 6.0f, 0.5f, 0.5f, 0.01f, -10.0f, 10.0f, -2.5f, 2.5f, 0.001f);
  PIDController_Init(&y_Angle,7.0f, 6.0f, 0.5f, 0.5f, 0.01f, -10.0f, 10.0f, -2.5f, 2.5f, 0.001f);
  PIDController_Init(&z_Angle,0.0f, 0.0f, 0.0f, 0.0f, 0.01f, -10.0f, 10.0f, -3.0f, 3.0f, 0.001f);

  PIDTemp_Init(&pid_temp, 70.0f, 0.01f, 0.0f, 0.01f, 0.0f, 100.0f, 0.0f, 100.0f, 0.001f);

  PIDController_Init(&hight_position_pid,1000.0f, 0.0f, 0.0f, 0.0001f, 0.02f, -50.0f, 50.0f,-50.0f, 50.0f, 0.001f);
  PIDController_Init(&hight_speed_pid,0.5f, 15.2f, 0.0f, 0.0f, 0.02f, -50.0f, 50.0f, -5.0f, 5.0f, 0.001f);
  PIDController_Init(&x_position_pid,0.2f, 0.0f, 0.0f, 0.0f, 0.02f, -20.0f, 20.0f, -5.0f, 5.0f, 0.001f);
  PIDController_Init(&y_position_pid,0.2f, 0.0f, 0.0f, 0.0f, 0.02f, -20.0f, 20.0f, -5.0f, 5.0f, 0.001f);

  filter_rc_bandstop_init(&bandstop_filter_ax, 1500.0f, 100.0f, 200.0f);
  filter_rc_bandstop_init(&bandstop_filter_ay, 1500.0f, 100.0f, 200.0f);
  filter_rc_bandstop_init(&bandstop_filter_az, 1500.0f, 100.0f, 200.0f);
  filter_rc_bandstop_init(&bandstop_filter_ax_2, 1500.0f, 600.0f, 700.0f);
  filter_rc_bandstop_init(&bandstop_filter_ay_2, 1500.0f, 600.0f, 700.0f);
  filter_rc_bandstop_init(&bandstop_filter_az_2, 1500.0f, 600.0f, 700.0f);
  filter_rc_bandstop_init(&bandstop_filter_gx, 1500.0f, 100.0f, 200.0f);
  filter_rc_bandstop_init(&bandstop_filter_gy, 1500.0f, 100.0f, 200.0f);
  filter_rc_bandstop_init(&bandstop_filter_gz, 1500.0f, 100.0f, 200.0f);
                                                                                                      
  filter_rc_lowpass_init(&lowpass_filter_accel_x, 1500.0f, 800.0f);                                                                                                           
  filter_rc_lowpass_init(&lowpass_filter_accel_y, 1500.0f, 800.0f);
  filter_rc_lowpass_init(&lowpass_filter_accel_z, 1500.0f, 800.0f);
  filter_rc_lowpass_init(&lowpass_filter_gyro_x, 1500.0f, 850.0f);
  filter_rc_lowpass_init(&lowpass_filter_gyro_y, 1500.0f, 850.0f);
  filter_rc_lowpass_init(&lowpass_filter_gyro_z, 1500.0f, 850.0f);
  filter_rc_lowpass_init(&lowpass_filter_speed_x, 1200.0f, 1000.0f);
  filter_rc_lowpass_init(&lowpass_filter_speed_y, 1200.0f, 1000.0f);
  filter_rc_lowpass_init(&lowpass_filter_speed_z, 1200.0f, 1000.0f);
  movavg_init(&mov_avg_speed_x, mov_avg_speed_x_buffer, 10);
  movavg_init(&mov_avg_speed_y, mov_avg_speed_y_buffer, 10);
  movavg_init(&mov_avg_speed_z, mov_avg_speed_z_buffer, 10);
  movavg_init(&msg_speed_x_filter, msg_speed_x_filter_buffer, 3);
  movavg_init(&msg_speed_y_filter, msg_speed_y_filter_buffer, 3);
  movavg_init(&msg_speed_z_filter, msg_speed_z_filter_buffer, 3);
  MahonyAHRS_init(&mahonyAHRS);
	bim088_init_state=1;
 while(bim088_init_state)
    {
        bim088_init_state=BMI088_init();
    }

//	 calibrate_accel_six_position_test(&accelCalibParam);

//dshot_init(DSHOT300);		
HAL_TIM_Base_Init(&htim16);
HAL_TIM_Base_Start_IT(&htim16);
HAL_TIM_Base_Init(&htim17);
HAL_TIM_Base_Start_IT(&htim17);
HAL_TIM_Base_Init(&htim15);
HAL_TIM_Base_Start_IT(&htim15);
HAL_UART_Receive_IT(&huart7, uartRx, 16);
HAL_UART_Receive_IT(&huart10, u10_rx_data,1);
//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
esc_init(PWM);
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
WS2812_Ctrl(10,0,10);

//  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
		

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
				if (uartUpdate)
	{
		uartUpdate = 0;
		//crsfLinkStatistics_t lqData;
		if (CRSF_RC(uartTemp, &rcData))
		{
		}
    if (CRSF_LQ(uartTemp, &lqData))
		{
		}
	}
		
		
		if (lqData.uplink_Link_quality>10&&losstime<90)
		{
      flag=1;
      ch0=((int)rcData.ch0-174)*100/(1811-174);
      ch1=((int)rcData.ch1-174)*100/(1811-174);
			ch2=((int)rcData.ch2-174)*100/(1811-174);
      ch3=((int)rcData.ch3-174)*100/(1811-174);
      ch4=((int)rcData.ch4-174)*100/(1811-174);
      ch5=((int)rcData.ch5-174)*100/(1811-174);
      ch6=((int)rcData.ch6-174)*100/(1811-174);
      ch7=((int)rcData.ch7-174)*100/(1811-174); 
      ch8=((int)rcData.ch8-174)*100/(1811-174);
      ch9=((int)rcData.ch9-174)*100/(1811-174);
      ch10=((int)rcData.ch10-174)*100/(1811-174);
      ch11=((int)rcData.ch11-174)*100/(1811-174);
      ch12=((int)rcData.ch12-174)*100/(1811-174);
      ch13=((int)rcData.ch13-174)*100/(1811-174);
      ch14=((int)rcData.ch14-174)*100/(1811-174);
      ch15=((int)rcData.ch15-174)*100/(1811-174); 
      if(ch7>90)
      {
        cotrol_mode=2;
      }else if(ch7<90&&ch7>10)
      {
        cotrol_mode=0;
      }else
      {
        cotrol_mode=1;
      }

      if(ch6>90)
      { 
//				if(ch4>90)
//			{WS2812_Ctrl(10,10,10);
//				if(dshot==false)
//				{
//					test_speed[0]=1200;
//				esc_set_speed(test_speed);
//				HAL_Delay(1000);
//				test_speed[0]=900;
//				esc_set_speed(test_speed);
//				HAL_Delay(1000);
//				test_speed[1]=1200;
//				esc_set_speed(test_speed);
//				HAL_Delay(1000);
//				test_speed[1]=900;
//				esc_set_speed(test_speed);
//				HAL_Delay(1000);
//				test_speed[2]=1200;
//				esc_set_speed(test_speed);
//				HAL_Delay(1000);
//				test_speed[2]=900;
//				esc_set_speed(test_speed);
//				HAL_Delay(1000);
//				test_speed[3]=1200;
//				esc_set_speed(test_speed);
//				HAL_Delay(1000);
//				test_speed[3]=900;
//				esc_set_speed(test_speed);
//				HAL_Delay(1000);
//        PID_Reset(&x_speed_pid);
//        PID_Reset(&y_speed_pid);
//        PID_Reset(&z_speed_pid);
//				}
				
				
		
				
//			}else
//			{
//				WS2812_Ctrl(0,10,0);
//				if(dshot==false)
//				{
//					esc_set_speed(speed);
//				}else
//				{
//					
//					dshot_write(speed);
//				}
        
//			}
			
			WS2812_Ctrl(0,10,0);
				if(dshot==false)
				{
					esc_set_speed(speed);
				}else
				{
					
					dshot_write(speed);
				}	
			
      }
			else 
      {
        WS2812_Ctrl(10,10,0);
        PID_Reset(&x_speed_pid);
        PID_Reset(&y_speed_pid);
        PID_Reset(&z_speed_pid);
				 PID_Reset(&x_speed_pid);
   PID_Reset(&y_speed_pid);
    PID_Reset(&z_speed_pid);
		  PID_Reset(&x_Angle);
   PID_Reset(&y_Angle);
    PID_Reset(&z_Angle);
       
				if(dshot==false)
				{
					esc_set_speed(err_speed);
          
				}else
				{
					
					dshot_write(err2_speed);
				}
        
      }
		
		  
		}else if(losstime>450)
		{
	  WS2812_Ctrl(0,0,10);
    ch0=50;
    ch1=50;
    ch2=0;
    ch3=50;
    ch4=0;
    ch5=0;
    ch6=0;
    ch7=0;
    ch8=0;
    ch9=0;
    ch10=0;
    ch11=0;
    ch12=0;
    ch13=0;
    ch14=0;
    ch15=0;

HAL_UART_Receive_IT(&huart7, uartRx, 16);
    PID_Reset(&x_speed_pid);
   PID_Reset(&y_speed_pid);
    PID_Reset(&z_speed_pid);
		  PID_Reset(&x_Angle);
   PID_Reset(&y_Angle);
    PID_Reset(&z_Angle);
			if(dshot==false)
				{
					esc_set_speed(err_speed);
				}else
				{
					
					dshot_write(err2_speed);
				}
        
		
    
		}
		
			
		
	


                                                                                               
//		HAL_UART_Transmit(&huart1, (uint8_t*)uartTemp, 64, 0xff);
//				sprintf(str,"------------------------\r\n");
//				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);

//				sprintf(str,"samples:%.6f,%.6f,%.6f\n",accel_x, accel_y, accel_z);
//				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
			 
//				sprintf(str, "samples:%.6f,%.6f,%.6f\n", gyro_x, gyro_y, gyro_z);
//				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//				sprintf(str, "temperate: %.2f\r\n", temperate);
//				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//    sprintf(str,"------------------------\r\n");
//    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);

//    sprintf(str, "samples::%.6f,%.6f,%.6f\n", mahonyAHRS.roll, mahonyAHRS.pitch, mahonyAHRS.yaw);
//    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//    sprintf(str,"q0: %.2f, q1: %.2f, q2: %.2f, q3: %.2f\r\n", mahonyAHRS.q0, mahonyAHRS.q1, mahonyAHRS.q2, mahonyAHRS.q3);
//    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//    sprintf(str, "temperate: %.2f pid_temp: %.2f\r\n", temperate, pid_temp.out);
//    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//    sprintf(str,"------------------------\r\n");
		
//    sprintf(str,"LQ: %d, RSSI1: %d, RSSI2: %d\r\n", lqData.uplink_Link_quality, lqData.uplink_RSSI_1, lqData.uplink_RSSI_2);
//    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//    sprintf(str,"bim088_init_state: %d\r\n", bim088_init_state);
//    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
    
//		sprintf(str,"pid_angle_speed: %.2f, %.2f, %.2f\r\n", pid_angle_speed.x_out, pid_angle_speed.y_out, pid_angle_speed.z_out);
//    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);s
//		
//			sprintf(str,"ch1: %d, ch2: %d, ch3: %d, ch4: %d\r\n", rcData.ch0, rcData.ch1, rcData.ch2, rcData.ch3);
//  		HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);

//    sprintf(str,"gx: %f, gy: %f, gz: %f\r\n", v_gworld[0], v_gworld[1], v_gworld[2]);
//    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//     sprintf(str,"gx: %f, gy: %f, gz: %f\r\n", v_body_lpf[0], v_body_lpf[1], v_body_lpf[2]);
//     HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//    sprintf(str,"vx: %f, vy: %f, vz: %f\r\n", v_body[0], v_body[1], v_body[2]);
//    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//    HAL_UART_Transmit(&huart1, (uint8_t*)tx_sreed, 12, 0xff);

//      sprintf(str,"losstime: %d\r\n", losstime);
//      HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//      sprintf(str,"speed::%d,%d,%d,%d\n", speed[0], speed[1], speed[2], speed[3]);
//      HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//      sprintf(str,"ch: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n", (ch0-50.0f)/10.0f,ch1,ch2,ch3,ch4,ch5,ch6,ch7);
//      HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);  
//      sprintf(str,"ch: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n", ch8,ch9,ch10,ch11,ch12,ch13,ch14,ch15);
//      HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//sprintf(str,"accel_correct: %.2f, %.2f, %.2f\r\n", a[0], a[1], a[2]);
//HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//sprintf(str,"pid_speed::%.6f,%.6f,%.6f\n", x_speed_pid.out, y_speed_pid.out, z_speed_pid.out);
//HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
sprintf(str,"data:%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%f,%f,%f,%f,%f\n",speed[0], speed[1], speed[2], speed[3],accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,mahonyAHRS.roll,mahonyAHRS.pitch, mahonyAHRS.yaw,angle[0],angle[1],angle[2],mahonyAHRS.q0,mahonyAHRS.q1,mahonyAHRS.q2,mahonyAHRS.q3,y_Angle.out,x_Angle.out);
//HAL_UART_Transmit(&huart10, (uint8_t*)str, strlen(str), 0xff); 
//sprintf(str,"data:%d,%d,%d,%d,%x,%d,%d\angle,mtf_01.distance,mtf_01.flow_vel_x,mtf_01.flow_vel_y,msg.status,u10_rx_data[0],msg.len,mtf_01.time_ms);
//sprintf(str,"data:,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
//sprintf(str,"data:,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,word_speed.x,word_speed.y,word_speed.z,mtf_01_speed.x,mtf_01_speed.y,mtf_01_speed.z);
//sprintf(str,"accel_correct: %.2f, %.2f, %.2f\r\n", angle[0], angle[1], angle[2]);

//  sprintf(str,"accel_correct:%f,%f,%f\r\n",word_accle.x,word_accle.y,word_accle.z);
 //sprintf(str,"accel_correct:%f,%f,%f,%d,%d,%f,%f,%f,%f,%f,%f\n",word_speed2.x*100,word_speed2.y*100,word_speed2.z*100,mtf_01.strength,mtf_01.flow_quality,UAV_position.x,UAV_position.y,UAV_position.z,body_speed2.x*100,body_speed2.y*100,body_speed2.z*100);
HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
//CDC_Transmit_HS((uint8_t*)str, strlen(str));

}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 33;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6|RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 1;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
