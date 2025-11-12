//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>
#include "usart.h"
#include "stdio.h"
#include "bmi088driver.h"
#include "buzzer.h"
#include "string.h"
//---------------------------------------------------------------------------------------------------
// Definitions


#define twoKpDef	(2.0f * 0.8f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.1f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions


//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update



// 输入六个静止位置的原始数据
void calibrate_accel_six_position(const float axp, const float axn,
                                 const float ayp, const float ayn,
                                 const float azp, const float azn,
                                 float g, AccelCalibParam* param)
{
    param->kx = (axp - axn) / (2.0f * g);
    param->bx = (axp + axn) / 2.0f;
    param->ky = (ayp - ayn) / (2.0f * g);
    param->by = (ayp + ayn) / 2.0f;
    param->kz = (azp - azn) / (2.0f * g);
    param->bz = (azp + azn) / 2.0f;
}

// 标定后数据修正
void accel_correct(const float raw[3], const AccelCalibParam* param, float corrected[3])
{
    corrected[0] = (raw[0] - param->bx) / param->kx;
    corrected[1] = (raw[1] - param->by) / param->ky;
    corrected[2] = (raw[2] - param->bz) / param->kz;
}











void calibrate_accel_six_position_test(AccelCalibParam *param)
{
    // 这里的数值需要根据实际测量结果填写
    float axp;   // x轴正方向朝下时的加速度值
    float axn;  // x轴负方向朝下时的加速度值
    float ayp;   // y轴正方向朝下时的加速度值
    float ayn ;  // y轴负方向朝下时的加速度值
    float azp ;   // z轴正方向朝下时的加速度值
    float azn ;  // z轴负方向朝下时的加速度值
    float gyro[3], accel[3], temperate;
    float data1[2],data2[2],data3[2];
   float old_data;
   float data;
    int flag,num;
    char str[100];
    flag=1;// 等待用户放置传感器
	num=0;
    while(flag==1)
    { BMI088_read(gyro, accel, &temperate);
      data1[0]=accel[0];
      data2[0]=accel[1];
      data3[0]=accel[2];
      // 检测X+方向
      
      if(accel[0]<10.5f&&accel[0]>9.4f&&data1[0]-data1[1]<0.05f&&data1[1]-data1[0]<0.05f&&data2[0]-data2[1]<0.05f&&data2[1]-data2[0]<0.05f&&data3[0]-data3[1]<0.05f&&data3[1]-data3[0]<0.05f)
      {
        
        data=accel[0];

        flag=2;
        
    
        buzzer_mod(1);
			  HAL_Delay(10);
			  buzzer_mod(0);
			  HAL_Delay(10);
      }
      if(flag==2)
      {
        old_data=data;
        axp=(old_data+data)/2;
        num++;
        flag=1;
      }
      if(num>10)
      {
        
        flag=0;
       num=0;
      }

        HAL_Delay(100);
        data1[1]=data1[0];
        data2[1]=data2[0];
        data3[1]=data3[0];
        sprintf(str, "Place the sensor with X+ down. Current Accel: X=%.2f, Y=%.2f, Z=%.2f\r\n", accel[0], accel[1], accel[2]);
        HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
    }
    
    flag=1;
    while(flag==1)
    { BMI088_read(gyro, accel, &temperate);
      data1[0]=accel[0];
      data2[0]=accel[1];
      data3[0]=accel[2];
      // 检测X-方向
      if(accel[0]>-10.5f&&accel[0]<-9.4f&&data1[0]-data1[1]<0.05f&&data1[1]-data1[0]<0.05f&&data2[0]-data2[1]<0.05f&&data2[1]-data2[0]<0.05f&&data3[0]-data3[1]<0.05f&&data3[1]-data3[0]<0.05f)
      {
        
        data=accel[0];

        flag=2;
        
    
        buzzer_mod(1);
        HAL_Delay(10);
        buzzer_mod(0);
        HAL_Delay(10);
      }
      if(flag==2)
      {
        old_data=data;
        axn=(old_data+data)/2;
        num++;
        flag=1;
      }
      if(num>10)
      {
        
        flag=0;
       num=0;
      }

        HAL_Delay(100);
        data1[1]=data1[0];
        data2[1]=data2[0];
        data3[1]=data3[0];
        sprintf(str, "Place the sensor with X- down. Current Accel: X=%.2f, Y=%.2f, Z=%.2f\r\n", accel[0], accel[1], accel[2]);
        HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
    }


    flag=1;
    while(flag==1)
    { BMI088_read(gyro, accel, &temperate);
      data1[0]=accel[0];
      data2[0]=accel[1];
      data3[0]=accel[2];
      // 检测Y+方向
      if(accel[1]<10.5f&&accel[1]>9.4f&&data1[0]-data1[1]<0.05f&&data1[1]-data1[0]<0.05f&&data2[0]-data2[1]<0.05f&&data2[1]-data2[0]<0.05f&&data3[0]-data3[1]<0.05f&&data3[1]-data3[0]<0.05f)
      {
        
        data=accel[1];

        flag=2;
        
    
        buzzer_mod(1);
        HAL_Delay(10);
        buzzer_mod(0);
        HAL_Delay(10);
      }
      if(flag==2)
      {
        old_data=data;
        ayp=(old_data+data)/2;
        num++;
        flag=1;
      }
      if(num>10)
      {
        
        flag=0;
       num=0;
      }

        HAL_Delay(100);
        data1[1]=data1[0];
        data2[1]=data2[0];
        data3[1]=data3[0];
        sprintf(str, "Place the sensor with Y+ down. Current Accel: X=%.2f, Y=%.2f, Z=%.2f\r\n", accel[0], accel[1], accel[2]);
        HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
    }

    flag=1;
    while(flag==1)
    { BMI088_read(gyro, accel, &temperate);
      data1[0]=accel[0];
      data2[0]=accel[1];
      data3[0]=accel[2];
      // 检测Y-方向
      if(accel[1]>-10.5f&&accel[1]<-9.4f&&data1[0]-data1[1]<0.05f&&data1[1]-data1[0]<0.05f&&data2[0]-data2[1]<0.05f&&data2[1]-data2[0]<0.05f&&data3[0]-data3[1]<0.05f&&data3[1]-data3[0]<0.05f)
      {
        
        data=accel[1];

        flag=2;
        
    
        buzzer_mod(1);
        HAL_Delay(10);
        buzzer_mod(0);
        HAL_Delay(10);
      }
      if(flag==2)
      {
        old_data=data;
        ayn=(old_data+data)/2;
        num++;
        flag=1;
      }
      if(num>10)
      {
        
        flag=0;
       num=0;
      }

        HAL_Delay(100);
        data1[1]=data1[0];
        data2[1]=data2[0];
        data3[1]=data3[0];
        sprintf(str, "Place the sensor with Y- down. Current Accel: X=%.2f, Y=%.2f, Z=%.2f\r\n", accel[0], accel[1], accel[2]);
        HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
    }

    flag=1;
    while(flag==1)
    { BMI088_read(gyro, accel, &temperate);
      data1[0]=accel[0];
      data2[0]=accel[1];
      data3[0]=accel[2];
      // 检测Z+方向
      if(accel[2]<10.5f&&accel[2]>9.4f&&data1[0]-data1[1]<0.05f&&data1[1]-data1[0]<0.05f&&data2[0]-data2[1]<0.05f&&data2[1]-data2[0]<0.05f&&data3[0]-data3[1]<0.05f&&data3[1]-data3[0]<0.05f)
      {
        
        data=accel[2];

        flag=2;
        
    
        buzzer_mod(1);
        HAL_Delay(10);
        buzzer_mod(0);
        HAL_Delay(10);
      }
      if(flag==2)
      {
        old_data=data;
        azp=(old_data+data)/2;
        num++;
        flag=1;
      }
      if(num>10)
      {
        
        flag=0;
       num=0;
      }

        HAL_Delay(100);
        data1[1]=data1[0];
        data2[1]=data2[0];
        data3[1]=data3[0];
        sprintf(str, "Place the sensor with Z+ down. Current Accel: X=%.2f, Y=%.2f, Z=%.2f\r\n", accel[0], accel[1], accel[2]);
        HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
    }

    flag=1;
    while(flag==1)
    { BMI088_read(gyro, accel, &temperate);
      data1[0]=accel[0];
      data2[0]=accel[1];
      data3[0]=accel[2];
      // 检测Z-方向
      if(accel[2]>-10.5f&&accel[2]<-9.4f&&data1[0]-data1[1]<0.05f&&data1[1]-data1[0]<0.05f&&data2[0]-data2[1]<0.05f&&data2[1]-data2[0]<0.05f&&data3[0]-data3[1]<0.05f&&data3[1]-data3[0]<0.05f)
      {
        
        data=accel[2];

        flag=2;
        
    
        buzzer_mod(1);
        HAL_Delay(10);
        buzzer_mod(0);
        HAL_Delay(10);
      }
      if(flag==2)
      {
        old_data=data;
        azn=(old_data+data)/2;
        num++;
        flag=1;
      }
      if(num>10)
      {
        
        flag=0;
       num=0;
      }

        HAL_Delay(100);
        data1[1]=data1[0];
        data2[1]=data2[0];
        data3[1]=data3[0];
        sprintf(str, "Place the sensor with Z- down. Current Accel: X=%.2f, Y=%.2f, Z=%.2f\r\n", accel[0], accel[1], accel[2]);
        HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
    }
    
    // 进行标定计算
    
  

    calibrate_accel_six_position(axp, axn, ayp, ayn, azp, azn, 9.81f, param);

    // 测试校准效果
    BMI088_read(gyro, accel, &temperate);
    float raw[3]; // 假设这是从传感器读取的原始数据
    raw[0] = accel[0];
    raw[1] = accel[1];
    raw[2] = accel[2];
    float corrected[3];
    accel_correct(raw, param, corrected);
   
    // 输出校准后的数据
    sprintf(str, "Corrected Accel: X=%.2f, Y=%.2f, Z=%.2f\r\n", corrected[0], corrected[1], corrected[2]);
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 0xff);
}



void MahonyAHRS_init(MahonyAHRS* ahrs)
{
	ahrs->q0=1.0f;
	ahrs->q1=0.0f;
	ahrs->q2=0.0f;
	ahrs->q3=0.0f;
	ahrs->twoKp=1.0f;
	ahrs->twoKi=0.0f;
	ahrs->integralFBx=0.0f;
	ahrs->integralFBy=0.0f;
	ahrs->integralFBz=0.0f;
	ahrs->anglesComputed=0;
	ahrs->roll=0.0f;
	ahrs->pitch=0.0f;
	ahrs->yaw=0.0f;
	ahrs->sampleFreq = 1200.0f;
	

}

void MahonyAHRS_setSampleFreq(MahonyAHRS* ahrs, float DT)
{
	float freq = 1.0f / DT;

	ahrs->sampleFreq = freq;
}

void MahonyAHRSupdate(MahonyAHRS* ahrs, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float q[4];
	q[0]=ahrs->q0;
	q[1]=ahrs->q1;
	q[2]=ahrs->q2;
	q[3]=ahrs->q3;
	float twoKp = ahrs->twoKp;
	float twoKi = ahrs->twoKi;
	float integralFBx = ahrs->integralFBx;
	float integralFBy = ahrs->integralFBy;
	float integralFBz = ahrs->integralFBz;
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(ahrs, gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / ahrs->sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / ahrs->sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / ahrs->sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / ahrs->sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / ahrs->sampleFreq));
	gz *= (0.5f * (1.0f / ahrs->sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
	ahrs->integralFBx = integralFBx;
	ahrs->integralFBy = integralFBy;
	ahrs->integralFBz = integralFBz;
	ahrs->q0 = q[0];
	ahrs->q1 = q[1];
	ahrs->q2 = q[2];
	ahrs->q3 = q[3];
	ahrs->twoKp = twoKp;
	ahrs->twoKi = twoKi;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(MahonyAHRS* ahrs, float gx, float gy, float gz, float ax, float ay, float az) {
	float q[4];
	q[0]=ahrs->q0;
	q[1]=ahrs->q1;
	q[2]=ahrs->q2;
	q[3]=ahrs->q3;
	float twoKp = ahrs->twoKp;
	float twoKi = ahrs->twoKi;
	float integralFBx = ahrs->integralFBx;
	float integralFBy = ahrs->integralFBy;
	float integralFBz = ahrs->integralFBz;
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;


	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / ahrs->sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / ahrs->sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / ahrs->sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / ahrs->sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / ahrs->sampleFreq));
	gz *= (0.5f * (1.0f / ahrs->sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
	ahrs->integralFBx = integralFBx;
	ahrs->integralFBy = integralFBy;
	ahrs->integralFBz = integralFBz;
	ahrs->q0 = q[0];
	ahrs->q1 = q[1];
	ahrs->q2 = q[2];
	ahrs->q3 = q[3];
	ahrs->twoKp = twoKp;
	ahrs->twoKi = twoKi;
	ahrs->anglesComputed = 0;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void Mahony_computeAngles(MahonyAHRS* ahrs)
{
	float q0 = ahrs->q0;
	float q1 = ahrs->q1;
	float q2 = ahrs->q2;
	float q3 = ahrs->q3;
	int anglesComputed = ahrs->anglesComputed;
	float roll, pitch, yaw;
	// Only compute angles if they haven't been computed yet
	if (anglesComputed) return;
	roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1);
	pitch = asin(-2 * q1 * q3 + 2 * q0 * q2);
	yaw   = atan2(2 *(q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

//	roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
//	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
//	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	anglesComputed = 1;
	ahrs->roll = roll-0.01f;
	ahrs->pitch = pitch-0.02f;
	ahrs->yaw = yaw;
	ahrs->anglesComputed = anglesComputed;
}

// 将四元数转换为机体坐标系下的欧拉角（ZYX: yaw, pitch, roll）
// 输入: q0..q3 (q0 = 实部/标量), 输出以弧度为单位
void QuaternionToEulerZYX(MahonyAHRS* ahrs)
{
  float q0 = ahrs->q0;
	float q1 = ahrs->q1;
	float q2 = ahrs->q2;
	float q3 = ahrs->q3;
  int anglesComputed = ahrs->anglesComputed;
float roll, pitch, yaw;
  // yaw (z-axis rotation)
  yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

  // pitch (y-axis rotation) with clamping to avoid NaN from asin
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (sinp >= 1.0f)
    pitch = 1.5707963267948966f ; // 90 degrees
  else if (sinp <= -1.0f)
    pitch = -1.5707963267948966f ; // -90 degrees
  else
    pitch = asinf(sinp);

  // roll (x-axis rotation)
  roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
  anglesComputed = 1;
  ahrs->roll = roll;
	ahrs->pitch = pitch;
	ahrs->yaw = yaw;
	ahrs->anglesComputed = anglesComputed;
}

// 四元数 (q0,q1,q2,q3) -> 方向余弦矩阵 R[3][3]
// 约定：q0 = 实部 (w)，q1..q3 = 虚部 (x,y,z)
// 结果满足 v_world = R * v_body
void QuaternionToDCM(MahonyAHRS* ahrs, float *R[3][3])
{
  float q0 = ahrs->q0;
	float q1 = ahrs->q1;
	float q2 = ahrs->q2;
	float q3 = ahrs->q3;

    // 预计算项，减少重复乘法
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    *R[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
    *R[0][1] = 2.0f * (q1q2 - q0q3);
    *R[0][2] = 2.0f * (q1q3 + q0q2);

    *R[1][0] = 2.0f * (q1q2 + q0q3);
    *R[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
    *R[1][2] = 2.0f * (q2q3 - q0q1);

    *R[2][0] = 2.0f * (q1q3 - q0q2);
    *R[2][1] = 2.0f * (q2q3 + q0q1);
    *R[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
}

void RotateBodyToWorld(float R[3][3], float accel_body[3], float *accel_world[3])
{
    *accel_world[0] = R[0][0]*accel_body[0] + R[0][1]*accel_body[1] + R[0][2]*accel_body[2];
    *accel_world[1] = R[1][0]*accel_body[0] + R[1][1]*accel_body[1] + R[1][2]*accel_body[2];
    *accel_world[2] = R[2][0]*accel_body[0] + R[2][1]*accel_body[1] + R[2][2]*accel_body[2];
}


// 矩阵乘法 C = A * B (3x3)
void DCMMultiply(const float A[3][3], const float B[3][3], float *C[3][3])
{
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            *C[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
        }
    }
}

// 矩阵转置 At = A^T
void DCMTranspose(const float A[3][3], float *At[3][3])
{
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            *At[i][j] = A[j][i];
        }
    }
}


void DCMToEulerZYX(float *R[3][3],MahonyAHRS* ahrs)
{
    // pitch = asin(-R[2][0])
    float roll, pitch, yaw;
    float sp = -*R[2][0];
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    pitch = asinf(sp);

    // 为了避免 gimbal lock 处的不稳定，判断 cp = cos(pitch) 是否接近 0
    float cp = cosf(pitch);

    if (fabsf(cp) > 1e-6f) {
        roll  = atan2f(*R[2][1], *R[2][2]);             // roll = atan2(R32, R33)
        yaw   = atan2f(*R[1][0], *R[0][0]);             // yaw  = atan2(R21, R11)
    } else {
        // 在极小的 cp 下（接近 +-90°），roll 和 yaw 不是唯一的，退化形式处理
        // 下面的处理给出一个连续解：当 cp -> 0 时，取 roll = 0，yaw = atan2(-R[0][1], R[1][1])
        roll = 0.0f;
        yaw  = atan2f(-*R[0][1], *R[1][1]);

    }

  ahrs->roll = roll+3.0f;
	ahrs->pitch = pitch;
	ahrs->yaw = yaw;



}

void turn_body_to_world(MahonyAHRS ahrs,UAV_xyz *word_speed,UAV_xyz *body_speed)
{
float R[3][3];
float v_body[3];
float v_global[3];
float q_body[4];
float q_conjugate[4];
float q_global[4];
float q_gworld[4];


float v_gworld[4];
	float inv=1/(ahrs.q0*ahrs.q0+ahrs.q1*ahrs.q1+ahrs.q2*ahrs.q2+ahrs.q3*ahrs.q3);
   q_body[0] = ahrs.q0;
          q_body[1] = ahrs.q1;
          q_body[2] = ahrs.q2;
          q_body[3] = ahrs.q3;
          q_body[0] *= inv;
          q_body[1] *= inv;
          q_body[2] *= inv;
          q_body[3] *= inv;
          float q0 = q_body[0];
          float q1 = q_body[1];
          float q2 = q_body[2];
          float q3 = q_body[3];


    R[0][0] = 1 - 2 * (q2 * q2 + q3 * q3);
    R[0][1] = 2 * (q1 * q2 - q0 * q3);
    R[0][2] = 2 * (q1 * q3 + q0 * q2);

    R[1][0] = 2 * (q1 * q2 + q0 * q3);
    R[1][1] = 1 - 2 * (q1 * q1 + q3 * q3);
    R[1][2] = 2 * (q2 * q3 - q0 * q1);

    R[2][0] = 2 * (q1 * q3 - q0 * q2);
    R[2][1] = 2 * (q2 * q3 + q0 * q1);
    R[2][2] = 1 - 2 * (q1 * q1 + q2 * q2);
        


    
          q_conjugate[0] = q_body[0];
          q_conjugate[1] = -q_body[1];
          q_conjugate[2] = -q_body[2];
          q_conjugate[3] = -q_body[3];
      
         
          
          q_global[1] = body_speed->x;
          q_global[2] = body_speed->y;
          q_global[3] = body_speed->z;
          q_global[0] = 0;



         for(int i = 0; i < 3; i++) {
    v_gworld[i] = R[i][0] * q_global[1] +
                     R[i][1] * q_global[2] +
                     R[i][2] * q_global[3];
				 }
				 
				 word_speed->x=v_gworld[0];
				 word_speed->y=v_gworld[1];
//				 word_speed->z=v_gworld[2]-9.8f;
				 word_speed->z=v_gworld[2];
				 
				 
}

        // -----------------------------------------------------------------------------
        // 四元数辅助函数与误差四元数计算
        // 说明：四元数采用 (q0, q1, q2, q3) 格式，q0 为实部（w），q1..q3 为虚部（x,y,z）
        // 误差四元数 q_err 定义为：将估计四元数 q_est 旋转到参考四元数 q_ref 的旋转
        // 即 q_ref = q_err ⊗ q_est  =>  q_err = q_ref ⊗ conj(q_est)
        // 返回的 q_err 已归一化。

        static void quat_conjugate(float *q, float *qc)
        {
          qc[0] =  q[0];
          qc[1] = -q[1];
          qc[2] = -q[2];
          qc[3] = -q[3];
        }

        static void quat_multiply(float *a,float *b, float *r)
        {
          // r = a ⊗ b
          r[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
          r[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
          r[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
          r[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
        }

        static void quat_normalize(float *q)
        {
          float norm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
          q[0] *= norm;
          q[1] *= norm;
          q[2] *= norm;
          q[3] *= norm;
        }

        // 计算误差四元数：q_err = q_ref ⊗ conj(q_est)
        // 输入：q_ref[4]（参考/目标四元数），q_est[4]（估计四元数）
        // 输出：q_err[4]（误差四元数，已归一化）
        void ComputeErrorQuaternion(float *q_ref,float *q_est, float *q_err)
        {
          float q_est_conj[4];
          quat_conjugate(q_est, q_est_conj);
          quat_multiply(q_ref, q_est_conj, q_err);
          quat_normalize(q_err);
        }

        // 将欧拉角 (ZYX: yaw, pitch, roll) 转换为四元数 (q0,q1,q2,q3)
        // 输入：yaw, pitch, roll — 以弧度为单位
        // 输出：q[4]，格式 (w, x, y, z)
        void EulerZYXToQuaternion(float yaw, float pitch, float roll, float *q)
        {
          // 计算半角的三角函数
          float cy = cosf(yaw * 0.5f);
          float sy = sinf(yaw * 0.5f);
          float cp = cosf(pitch * 0.5f);
          float sp = sinf(pitch * 0.5f);
          float cr = cosf(roll * 0.5f);
          float sr = sinf(roll * 0.5f);

          // ZYX顺序 (yaw -> z, pitch -> y, roll -> x)
          q[0] = cr * cp * cy + sr * sp * sy; // w
          q[1] = sr * cp * cy - cr * sp * sy; // x
          q[2] = cr * sp * cy + sr * cp * sy; // y
          q[3] = cr * cp * sy - sr * sp * cy; // z

          // 归一化以避免数值漂移
          quat_normalize(q);
        }

      /*
       * 将机体坐标系下的欧拉角（顺序：roll(X), pitch(Y), yaw(Z)，弧度）转换为四元数
       * 输入：roll, pitch, yaw（弧度）
       * 输出：q[4] (w, x, y, z)
       * 说明：内部调用已有的 ZYX->四元数 实现（EulerZYXToQuaternion），保持一致的约定。
       */
      void BodyEulerToQuaternion(float roll, float pitch, float yaw, float q[4])
      {
        /* EulerZYXToQuaternion 接受参数顺序为 (yaw, pitch, roll) */
        EulerZYXToQuaternion(yaw, pitch, roll, q);
      }

      // 将四元数 q 转为 ZYX (yaw,pitch,roll)，输出以弧度为单位
      // 使用与文件中其它 Euler 转换一致的约定并作夹紧处理以避免 NaN
      static void quat_to_euler_zyx(const float q[4], float *yaw, float *pitch, float *roll)
      {
        float q0 = q[0];
        float q1 = q[1];
        float q2 = q[2];
        float q3 = q[3];

        // yaw (z)
        *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

        // pitch (y) with clamping
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (sinp >= 1.0f) *pitch = 1.5707963267948966f;
        else if (sinp <= -1.0f) *pitch = -1.5707963267948966f;
        else *pitch = asinf(sinp);

        // roll (x)
        *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
      }

      // 计算误差四元数，但忽略 yaw 分量（只保留 roll/pitch 差异）
      // 方法：将参考四元数的 yaw 替换为估计四元数的 yaw，再计算误差四元数
      // 输入：q_ref, q_est（均为长度 4 的数组，格式 w,x,y,z）
      // 输出：q_err（长度 4，已归一化）
      void ComputeErrorQuaternionIgnoreYaw(const float q_ref_in[4], const float q_est_in[4], float q_err[4])
      {
        float yaw_ref, pitch_ref, roll_ref;
        float yaw_est, pitch_est, roll_est;
        float q_ref[4];
        float q_est[4];

        // 复制输入到可写数组（文件中的 quat helper 接口使用非 const 参数）
        q_ref[0] = q_ref_in[0]; q_ref[1] = q_ref_in[1]; q_ref[2] = q_ref_in[2]; q_ref[3] = q_ref_in[3];
        q_est[0] = q_est_in[0]; q_est[1] = q_est_in[1]; q_est[2] = q_est_in[2]; q_est[3] = q_est_in[3];

        // 将四元数转换为 Euler
        quat_to_euler_zyx(q_ref, &yaw_ref, &pitch_ref, &roll_ref);
        quat_to_euler_zyx(q_est, &yaw_est, &pitch_est, &roll_est);

        // 用估计的 yaw 替换参考的 yaw（这样 yaw 差异会被消除）
        float q_ref_adj[4];
        EulerZYXToQuaternion(yaw_est, pitch_ref, roll_ref, q_ref_adj);

        // 计算误差四元数 q_err = q_ref_adj ⊗ conj(q_est)
        float q_est_conj[4];
        quat_conjugate(q_est, q_est_conj);
        quat_multiply(q_ref_adj, q_est_conj, q_err);
        quat_normalize(q_err);
      }



      // 将世界坐标系下的向量转换为机体坐标系下的向量（使用四元数）
      // q: 表示从 body -> world 的旋转（格式 w,x,y,z）
      // v_world: 输入世界坐标向量 (3)
      // v_body: 输出机体坐标向量 (3)
      void WorldToBodyQuaternion(float q[4], float *v_world, float *v_body)
      {
        // 归一化四元数副本以保证稳定
        float qn[4] = { q[0], q[1], q[2], q[3] };
        quat_normalize(qn);

        // vq = [0, v_world]
        float vq[4] = { 0.0f, v_world[0], v_world[1], v_world[2] };

        // q_conj ⊗ vq ⊗ q
        float qconj[4];
        quat_conjugate(qn, qconj);
        float tmp[4];
        quat_multiply(qconj, vq, tmp);
        float res[4];
        quat_multiply(tmp, qn, res);

        // 输出矢量部分
        v_body[0] = res[1];
        v_body[1] = res[2];
        v_body[2] = res[3];
      }

      // 包装函数：使用 MahonyAHRS 结构的当前四元数将世界向量转换为机体向量
      void WorldToBodyFromMahony(MahonyAHRS* ahrs,float *v_world, float *v_body)
      {
        float q[4] = { ahrs->q0, ahrs->q1, ahrs->q2, ahrs->q3 };
        WorldToBodyQuaternion(q, v_world, v_body);
      }

      void WorldToBodyFromMahony_xyz(MahonyAHRS* ahrs,UAV_xyz *word_xyz,UAV_xyz *body_xyz)
      {
        float v_world[3];
        float v_body[3];
        v_world[0]=word_xyz->x;
        v_world[1]=word_xyz->y;
        v_world[2]=word_xyz->z;
        WorldToBodyFromMahony(ahrs,v_world,v_body);
        body_xyz->x=v_body[0];
        body_xyz->y=v_body[1];
        body_xyz->z=v_body[2];
      }
      
  // 将误差四元数转换为三轴角度误差向量
  // 输入：q_err[4] (w,x,y,z)
  // 输出：angle[3]，三轴角度误差（以弧度为单位），对应绕 body x,y,z 轴的旋转量
  // 方法：四元数表示为 q = [cos(theta/2), ux*sin(theta/2), uy*sin(theta/2), uz*sin(theta/2)]
  // 因此旋转角 = theta = 2 * acos(w)，轴 = v / sin(theta/2)
  // 当 sin(theta/2) 很小时使用近似 angle_vec = 2 * v （数值稳定）
  void ErrorQuaternionToAngle(float *q_err, float *angle)
  {
    float w = q_err[0];
    float x = q_err[1];
    float y = q_err[2];
    float z = q_err[3];

    // 限幅防止数值问题
    if (w > 1.0f) w = 1.0f;
    if (w < -1.0f) w = -1.0f;

    float theta = 2.0f * acosf(w); // 旋转角度（rad）
    float s = sqrtf(1.0f - w * w); // sin(theta/2)

//    if (s < 1e-6f) {
//      // 角度非常小，使用线性近似：angle_vec ≈ 2 * (x,y,z)
//      angle[0] = 2.0f * x;
//      angle[1] = 2.0f * y;
//      angle[2] = 2.0f * z;
//    } else {
//      float ux = x / s;
//      float uy = y / s;
//      float uz = z / s;
//      angle[0] = ux * theta;
//      angle[1] = uy * theta;
//      angle[2] = uz * theta;
//    }

      angle[0] = 2.0f * x;
      angle[1] = 2.0f * y;
      angle[2] = 2.0f * z;

  }

// 将误差四元数转换为机体坐标系下的三轴角度误差向量
// 输入：q_err[4]（误差四元数，格式 w,x,y,z），q_est[4]（估计四元数，格式 w,x,y,z）
// 输出：angle_body[3]，在机体坐标系下的角度误差（弧度）
// 方法：先将 q_err 转为轴-角（在参考/world 坐标系），得到旋转向量 r_world = axis * theta，
// 然后用 q_est 的共轭把 r_world 旋转到机体坐标系： r_body = conj(q_est) ⊗ [0,r_world] ⊗ q_est
// 注意：此处假设 q_est 表示 body -> world（与文件中 QuaternionToDCM 的约定一致）。
void ErrorQuaternionToBodyAngle(float *q_err, float *q_est, float *angle_body)
{
  // 先得到轴-角表示（在参考/world 坐标系）
  float w = q_err[0];
  float x = q_err[1];
  float y = q_err[2];
  float z = q_err[3];

  if (w > 1.0f) w = 1.0f;
  if (w < -1.0f) w = -1.0f;

  float theta = 2.0f * acosf(w);
  float s = sqrtf(1.0f - w * w);

  float r_world[3];
  if (s < 1e-6f) {
    // 小角近似
    r_world[0] = 2.0f * x;
    r_world[1] = 2.0f * y;
    r_world[2] = 2.0f * z;
  } else {
    float ux = x / s;
    float uy = y / s;
    float uz = z / s;
    r_world[0] = ux * theta;
    r_world[1] = uy * theta;
    r_world[2] = uz * theta;
  }

  // 将 r_world 旋转到机体坐标系： r_body = conj(q_est) ⊗ [0,r_world] ⊗ q_est
  float q_est_copy[4] = { q_est[0], q_est[1], q_est[2], q_est[3] };
  quat_normalize(q_est_copy);
  float vq[4] = { 0.0f, r_world[0], r_world[1], r_world[2] };
  float qconj[4];
  quat_conjugate(q_est_copy, qconj);
  float tmp[4];
  quat_multiply(qconj, vq, tmp);
  float res[4];
  quat_multiply(tmp, q_est_copy, res);

  angle_body[0] = res[1];
  angle_body[1] = res[2];
  angle_body[2] = res[3];
}

// 便捷包装：使用 MahonyAHRS 中当前估计四元数
void ErrorQuaternionToBodyAngleFromMahony(float *q_err, MahonyAHRS *ahrs, float *angle_body)
{
  float q_est[4] = { ahrs->q0, ahrs->q1, ahrs->q2, ahrs->q3 };
  ErrorQuaternionToBodyAngle(q_err, q_est, angle_body);
}

	
	
	      // 与 Quaternion_compute_to_errorangle 相似的包装，但在计算误差时忽略 yaw
      void Quaternion_compute_to_errorangle_ignoreyaw(float yaw, float pitch, float roll, MahonyAHRS *ahrs, float *angle)
      {
        float q_ref[4];
        float q_est[4];
        float q_err[4];

        EulerZYXToQuaternion(yaw, pitch, roll, q_ref);
        q_est[0] = ahrs->q0; q_est[1] = ahrs->q1; q_est[2] = ahrs->q2; q_est[3] = ahrs->q3;
        ComputeErrorQuaternionIgnoreYaw(q_ref, q_est, q_err);
        ErrorQuaternionToAngle(q_err, angle);
      }

    // 计算误差四元数并将其表示为机体坐标系下的四元数
    // 输入：q_ref（参考四元数，w,x,y,z），q_est（估计四元数，w,x,y,z）
    // 输出：q_err_body（机体坐标系下的误差四元数，已归一化）
    void ComputeErrorQuaternionBodyFrame(float *q_ref,float *q_est, float* q_err_body)
    {
      float q_est_conj[4];
      float q_err_world[4];
      float tmp[4];

      /* q_err_world = q_ref * conj(q_est) */
      quat_conjugate((float*)q_est, q_est_conj);
      quat_multiply((float*)q_ref, q_est_conj, q_err_world);
      quat_normalize(q_err_world);

      /* 将误差四元数从世界/参考框架变换到机体框架：
         q_err_body = conj(q_est) * q_err_world * q_est
         这样四元数的轴会被表示在机体坐标系中 */
      quat_multiply(q_est_conj, q_err_world, tmp);
      quat_multiply(tmp, (float*)q_est, q_err_body);
      quat_normalize(q_err_body);
    }

    // 包装函数：接受参考欧拉角（yaw,pitch,roll）和 MahonyAHRS 估计，返回机体坐标系下的误差四元数
    void Quaternion_compute_error_quat_body_from_euler(float yaw, float pitch, float roll, MahonyAHRS *ahrs, float *q_err_body)
    {
      float q_ref[4];
      float q_est[4];

      EulerZYXToQuaternion(yaw, pitch, roll, q_ref);
      q_est[0] = ahrs->q0; q_est[1] = ahrs->q1; q_est[2] = ahrs->q2; q_est[3] = ahrs->q3;
      ComputeErrorQuaternionBodyFrame(q_ref, q_est, q_err_body);
    }

  // 包装：当参考欧拉角以机体坐标系（roll,pitch,yaw）给出时，计算机体坐标系下的误差四元数
  void Quaternion_compute_error_quat_body_from_bodyEuler(float roll, float pitch, float yaw, MahonyAHRS *ahrs, float *q_err_body)
  {
    float q_ref[4];
    float q_est[4];

    /* 将机体欧拉角转换为参考四元数 */
    BodyEulerToQuaternion(roll, pitch, yaw, q_ref);
    q_est[0] = ahrs->q0; q_est[1] = ahrs->q1; q_est[2] = ahrs->q2; q_est[3] = ahrs->q3;
    ComputeErrorQuaternionBodyFrame(q_ref, q_est, q_err_body);
  }

  // 包装：接受机体坐标系下的参考欧拉角（roll,pitch,yaw），返回机体坐标系下的角度误差向量（弧度）
  void Quaternion_compute_to_errorangle_bodyEuler(float roll, float pitch, float yaw, MahonyAHRS *ahrs, float *angle_body)
  {
    float q_err_body[4];
    Quaternion_compute_error_quat_body_from_bodyEuler(roll, pitch, yaw, ahrs, q_err_body);
    /* q_err_body 已在机体坐标系中，直接转换为角度向量（使用小角近似） */
    ErrorQuaternionToAngle(q_err_body, angle_body);
  }
  
  void Quaternion_compute_to_errorangle(float yaw, float pitch, float roll,MahonyAHRS *ahrs, float *angle)
  {
    float q_ref[4];
    float q_est[4];
    float q_err[4];

    // 将参考欧拉角转换为四元数
    EulerZYXToQuaternion(yaw, pitch, roll, q_ref);
    // 获取估计四元数
    q_est[0] = ahrs->q0;
    q_est[1] = ahrs->q1;
    q_est[2] = ahrs->q2;
    q_est[3] = ahrs->q3;
    // 计算误差四元数
    //ComputeErrorQuaternion(q_ref, q_est, q_err);
    ComputeErrorQuaternionBodyFrame(q_ref, q_est, q_err);
		
    // 将误差四元数转换为角度误差向量
    ErrorQuaternionToAngle(q_err, angle);
//		ErrorQuaternionToBodyAngleFromMahony(q_err,ahrs, angle);
  }
 


//====================================================================================================
// END OF CODE
//====================================================================================================
