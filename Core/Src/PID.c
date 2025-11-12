#include "PID.h"
#include "MahonyAHRS.h"

void PID_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;
	pid->feedforward_input_old = 0.0f;
	
	pid->out = 0.0f;


}

void PIDController_Init(PIDController *pid, float ff_k,float Kp, float Ki, float Kd, float tau, float limMin, float limMax, float limMinInt, float limMaxInt, float T) {

	/* Set gains */
	pid->limMin = limMin;
	pid->limMax = limMax;
	pid->limMinInt = limMinInt;
	pid->limMaxInt = limMaxInt;
	pid->T = T;
	pid->tau = tau;
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->feedforward_kp = ff_k;	
	PID_Init(pid);

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

	/*
	* Error signal
	*/
    float error = setpoint - measurement;
	
	
	/*
	* Feedforward
	*/
	float ff_delta = setpoint - pid->feedforward_input_old;
	float feedforward_out = pid->feedforward_kp * ff_delta;
	pid->feedforward_input_old = setpoint;
	/*
	* Proportional
	*/
   float proportional = pid->Kp * error;


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }


	/*
	* Derivative (band-limited differentiator)
	*/
		
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


	/*
	* Compute output and apply limits
	*/
    pid->out = proportional + pid->integrator + pid->differentiator+ feedforward_out;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}

void PID_Reset(PIDController* pid)
{
	pid->integrator = 0;
	pid->out = 0;
	pid->prevError = 0;
	pid->differentiator = 0;
}

void PIDIncremental_Reset(PIDController* pid)
{
	pid->prevError = 0;
	pid->out = 0;
}


void PIDIncremental_Update(PIDController* pid,float target,float current)
{
	float error = target - current;
	float ff_delta = target - pid->feedforward_input_old;
	float feedforward_out = pid->feedforward_kp * ff_delta;
	pid->feedforward_input_old = target;
	pid->out += pid->Kp*(error-pid->prevError) + pid->Ki*pid->prevError+feedforward_out;
	
	if( pid->out > pid->limMaxInt ) pid->out = pid->limMaxInt;
	if( pid->out < pid->limMinInt ) pid->out = pid->limMinInt;
	
	pid->prevError = error;
	
}

/* Angle PID Controller functions */


void PIDTemp_Init(PIDController *pid, float Kp, float Ki, float Kd, float tau, float limMin, float limMax, float limMinInt, float limMaxInt, float T) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;
	/* Set gains */
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;	
	/* Set limits */
	pid->limMin = limMin;
	pid->limMax = limMax;
	pid->limMinInt = limMinInt;
	pid->limMaxInt = limMaxInt;
	/* Set sample time */
	pid->T = T;
	/* Derivative low-pass filter time
	constant */
	pid->tau = tau;

}

void PIDTemp_Update(PIDController* pid,float target,float current)
{
	float error = target - current;
	pid->out += pid->Kp*(error-pid->prevError) + pid->Ki*pid->prevError;
	
	if( pid->out > pid->limMaxInt ) pid->out = pid->limMaxInt;
	if( pid->out < pid->limMinInt ) pid->out = pid->limMinInt;
	
	pid->prevError = error;
	
}

void PIDControllerAngle_Init(PIDController_Angle *pid) 
{

	/* Clear controller variables */
	pid->x_integrator = 0.0f;
	pid->X_prevError  = 0.0f;

	pid->x_differentiator  = 0.0f;
	pid->x_prevMeasurement = 0.0f;

	pid->x_out = 0.0f;

	pid->y_integrator = 0.0f;
	pid->Y_prevError  = 0.0f;

	pid->y_differentiator  = 0.0f;
	pid->y_prevMeasurement = 0.0f;

	pid->y_out = 0.0f;

	pid->z_integrator = 0.0f;
	pid->Z_prevError  = 0.0f;

	pid->z_differentiator  = 0.0f;
	pid->z_prevMeasurement = 0.0f;

	pid->z_out = 0.0f;

}


void PIDControllerAngle_Update(PIDController_Angle *pid, float x_setpoint, float x_measurement, float y_setpoint, float y_measurement, float z_setpoint, float z_measurement) 
{

	/*
	* Error signal
	*/
	float x_error = x_setpoint - x_measurement;
	float y_error = y_setpoint - y_measurement;
	float z_error = z_setpoint - z_measurement;
	/*
	* Proportional
	*/
	float x_proportional = pid->Kp * x_error;
	float y_proportional = pid->Kp * y_error;
	float z_proportional = pid->Kp * z_error;	
	/*
	* Integral
	*/
	pid->x_integrator = pid->x_integrator + 0.5f * pid->Ki * pid->T * (x_error + pid->X_prevError);
	pid->y_integrator = pid->y_integrator + 0.5f * pid->Ki * pid->T * (y_error + pid->Y_prevError);
	pid->z_integrator = pid->z_integrator + 0.5f * pid->Ki * pid->T * (z_error + pid->Z_prevError);	
	/* Anti-wind-up via integrator clamping */
	if (pid->x_integrator > pid->limMaxInt) {
		pid->x_integrator = pid->limMaxInt;
	} else if (pid->x_integrator < pid->limMinInt) {
		pid->x_integrator = pid->limMinInt;
	}
	if (pid->y_integrator > pid->limMaxInt) {
		pid->y_integrator = pid->limMaxInt;
	} else if (pid->y_integrator < pid->limMinInt) {
		pid->y_integrator = pid->limMinInt;
	}
	if (pid->z_integrator > pid->limMaxInt) {	
		pid->z_integrator = pid->limMaxInt;
	} else if (pid->z_integrator < pid->limMinInt) {
		pid->z_integrator = pid->limMinInt;
	}
	/*
	* Derivative (band-limited differentiator)
	*/
	pid->x_differentiator = -(2.0f * pid->Kd * (x_measurement - pid->x_prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
		+ (2.0f * pid->tau - pid->T) * pid->x_differentiator)
		/ (2.0f * pid->tau + pid->T);
	pid->y_differentiator = -(2.0f * pid->Kd * (y_measurement - pid->y_prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
		+ (2.0f * pid->tau - pid->T) * pid->y_differentiator)
		/ (2.0f * pid->tau + pid->T);
	pid->z_differentiator = -(2.0f * pid->Kd * (z_measurement - pid->z_prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
		+ (2.0f * pid->tau - pid->T) * pid->z_differentiator)
		/ (2.0f * pid->tau + pid->T);
	/*
	* Compute output and apply limits
	*/
	pid->x_out = x_proportional + pid->x_integrator + pid->x_differentiator;
	pid->y_out = y_proportional + pid->y_integrator + pid->y_differentiator;
	pid->z_out = z_proportional + pid->z_integrator + pid->z_differentiator;	
	if (pid->x_out > pid->limMax) {
		pid->x_out = pid->limMax;
	} else if (pid->x_out < pid->limMin) {
		pid->x_out = pid->limMin;
	}
	if (pid->y_out > pid->limMax) {
		pid->y_out = pid->limMax;
	} else if (pid->y_out < pid->limMin) {
		pid->y_out = pid->limMin;
	}
	if (pid->z_out > pid->limMax) {
		pid->z_out = pid->limMax;
	} else if (pid->z_out < pid->limMin) {
		pid->z_out = pid->limMin;
	}
	/* Store error and measurement for later use */
	pid->X_prevError       = x_error;
	pid->x_prevMeasurement = x_measurement;
	pid->Y_prevError       = y_error;
	pid->y_prevMeasurement = y_measurement;
	pid->Z_prevError       = z_error;
	pid->z_prevMeasurement = z_measurement;	
	/* Return controller output */
	//return pid->out;	
}

void PIDAngle_Reset(PIDController_Angle* pid)
{
	pid->x_integrator = 0;
	pid->x_out = 0;
	pid->X_prevError = 0;
	pid->x_differentiator = 0;

	pid->y_integrator = 0;
	pid->y_out = 0;
	pid->Y_prevError = 0;
	pid->y_differentiator = 0;

	pid->z_integrator = 0;
	pid->z_out = 0;
	pid->Z_prevError = 0;
	pid->z_differentiator = 0;
}


 
void PIDAngleIncremental_SET(PIDController_Angle* pid,float p,float i,float d)
{
	pid->Kp = p;
	pid->Ki = i;
	pid->Kd = d;
}


void PIDAngle_init(PIDController_Angle* pid, float kp, float ki, float kd, float tau, float limMin, float limMax, float limMinInt, float limMaxInt, float T)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;

	pid->tau = tau;

	pid->limMin = limMin;
	pid->limMax = limMax;

	pid->limMinInt = limMinInt;
	pid->limMaxInt = limMaxInt;

	pid->T = T;

	PIDControllerAngle_Init(pid);
}

void Quaternion_PIDController(PIDController* pid,float angle)
{
	float error = angle;
	
	
	/*
	* Feedforward
	*/
//	float ff_delta = angle - pid->feedforward_input_old;
//	float feedforward_out = pid->feedforward_kp * ff_delta;
//	pid->feedforward_input_old = angle;
	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }


	/*
	* Derivative (band-limited differentiator)
	*/
		
    pid->differentiator = -(2.0f * pid->Kd * (error -  pid->prevError)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


	/*
	* Compute output and apply limits
	*/
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;

	/* Return controller output */

	
}




/* End of file */


