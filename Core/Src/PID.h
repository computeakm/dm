#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;
	/**/
	float feedforward_kp;
	float feedforward_input_old;
	
	

} PIDController;



typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float x_proportional; 		/* Required for proportional */
	float x_integrator;
	float X_prevError;			/* Required for integrator */
	float x_differentiator;
	float x_prevMeasurement;		/* Required for differentiator */

	float y_proportional; 		/* Required for proportional */
	float y_integrator;
	float Y_prevError;			/* Required for integrator */
	float y_differentiator;
	float y_prevMeasurement;		/* Required for differentiator */

	float z_proportional; 		/* Required for proportional */
	float z_integrator;
	float Z_prevError;			/* Required for integrator */
	float z_differentiator;
	float z_prevMeasurement;		/* Required for differentiator */


	/* Controller output */
	float x_out;
	float y_out;
	float z_out;

} PIDController_Angle;






void PIDController_Init(PIDController *pid, float ff_k,float Kp, float Ki, float Kd, float tau, float limMin, float limMax, float limMinInt, float limMaxInt, float T);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);
void PID_Reset(PIDController* pid);
void PIDIncremental_Reset(PIDController* pid);
void PIDIncremental_Update(PIDController* pid,float target,float current);

/* Angle PID Controller functions */

void  PIDControllerAngle_Init(PIDController_Angle *pid);
void PIDControllerAngle_Update(PIDController_Angle *pid, float x_setpoint, float x_measurement, float y_setpoint, float y_measurement, float z_setpoint, float z_measurement);
void PIDAngle_Reset(PIDController_Angle* pid);
void PIDAngleIncremental_SET(PIDController_Angle* pid,float p,float i,float d);
void PIDAngle_init(PIDController_Angle* pid, float kp, float ki, float kd, float tau, float limMin, float limMax, float limMinInt, float limMaxInt, float T);
void PIDTemp_Update(PIDController* pid,float target,float current);
void PIDTemp_Init(PIDController *pid, float Kp, float Ki, float Kd, float tau, float limMin, float limMax, float limMinInt, float limMaxInt, float T);
void Quaternion_PIDController(PIDController* pid,float angle);

#endif
