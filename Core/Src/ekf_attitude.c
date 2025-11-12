// ekf_attitude.c - minimal attitude EKF (error-state) for gyro+accel
#include "ekf_attitude.h"
#include <math.h>
#include <string.h>

static void quat_normalize(float q[4]){
    float n = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if(n>0.0f){ q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }
}

// small matrix helpers (naive, 3x3 and 6x6 operations)
static void mat3_identity(float A[3][3]){ memset(A,0,sizeof(float)*9); A[0][0]=A[1][1]=A[2][2]=1.0f; }
static void mat6_identity(float A[6][6]){ memset(A,0,sizeof(float)*36); for(int i=0;i<6;i++) A[i][i]=1.0f; }

void EKF_Attitude_Init(EKF_Attitude *ekf, float dt){
    // init quaternion (identity)
    ekf->q[0]=1.0f; ekf->q[1]=0; ekf->q[2]=0; ekf->q[3]=0;
    ekf->b[0]=ekf->b[1]=ekf->b[2]=0.0f;
    ekf->dt = dt;
    // init covariances
    mat6_identity(ekf->P);
    // small initial uncertainty
    for(int i=0;i<3;i++) ekf->P[i][i]=1e-3f; // attitude error
    for(int i=3;i<6;i++) ekf->P[i][i]=1e-4f; // bias error
    // process noise Q
    mat6_identity(ekf->Q);
    for(int i=0;i<3;i++) ekf->Q[i][i]=1e-6f; // gyro noise
    for(int i=3;i<6;i++) ekf->Q[i][i]=1e-8f; // bias random walk
    // accel measurement noise
    mat3_identity(ekf->R);
    for(int i=0;i<3;i++) ekf->R[i][i]=1e-2f;
}

// rotate vector v by quaternion q (q = w,x,y,z)
static void quat_rotate(const float q[4], const float v[3], float out[3]){
    float w=q[0], x=q[1], y=q[2], z=q[3];
    // compute q * (0,v) * q_conj
    float t0 =  w*v[0] + y*v[2] - z*v[1];
    float t1 =  w*v[1] + z*v[0] - x*v[2];
    float t2 =  w*v[2] + x*v[1] - y*v[0];
    float t3 = -x*v[0] - y*v[1] - z*v[2];
    out[0] = t0*w + t3*-x + t1*-z - t2*-y;
    out[1] = t1*w + t3*-y + t2*-x - t0*-z;
    out[2] = t2*w + t3*-z + t0*-y - t1*-x;
}

// predict: integrate quaternion with gyro-bias corrected angular rate
void EKF_Attitude_Predict(EKF_Attitude *ekf, const float gyro[3]){
    float w[3];
    w[0]=gyro[0]-ekf->b[0]; w[1]=gyro[1]-ekf->b[1]; w[2]=gyro[2]-ekf->b[2];
    float dt = ekf->dt;
    // quaternion kinematic: q_dot = 0.5 * Omega(w) * q
    float q0=ekf->q[0], q1=ekf->q[1], q2=ekf->q[2], q3=ekf->q[3];
    float dq0 = -0.5f*( q1*w[0] + q2*w[1] + q3*w[2]);
    float dq1 =  0.5f*( q0*w[0] + q2*w[2] - q3*w[1]);
    float dq2 =  0.5f*( q0*w[1] - q1*w[2] + q3*w[0]);
    float dq3 =  0.5f*( q0*w[2] + q1*w[1] - q2*w[0]);
    ekf->q[0] += dq0*dt; ekf->q[1] += dq1*dt; ekf->q[2] += dq2*dt; ekf->q[3] += dq3*dt;
    quat_normalize(ekf->q);
    // P = P + Q*dt (simple, ignoring linearized F)
    for(int i=0;i<6;i++){
        for(int j=0;j<6;j++){
            ekf->P[i][j] += ekf->Q[i][j]*dt;
        }
    }
}

// update with accel: uses that accel measures gravity direction in body frame
void EKF_Attitude_UpdateAccel(EKF_Attitude *ekf, const float accel[3]){
    // normalize accel measurement
    float an = sqrtf(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    if(an < 1e-6f) return;
    float am[3] = {accel[0]/an, accel[1]/an, accel[2]/an};
    // predicted gravity in body frame = rotate world gravity (0,0,1) by q_conj? For body->world q, gravity_body = R^T * [0,0,1]
    // easier: rotate body vector [0,0,1] by quaternion -> gives world Z; but we want gravity in body: rotate world gravity by q_conj
    // compute predicted gravity in body frame
    float g_world[3] = {0.0f, 0.0f, 1.0f};
    float g_body[3];
    // rotate world gravity into body frame by q_conj: q_conj * g_world * q
    // implement as rotate with conjugate
    float qc[4] = { ekf->q[0], -ekf->q[1], -ekf->q[2], -ekf->q[3] };
    quat_rotate(qc, g_world, g_body);

    // innovation: y = a_meas - g_body
    float y[3] = { am[0]-g_body[0], am[1]-g_body[1], am[2]-g_body[2] };

    // Simplified gain: approximate H and compute small correction on attitude vector (phi)
    // Compute a small error rotation vector proportional to cross(g_body, a_meas)
    float ex = (g_body[1]*am[2] - g_body[2]*am[1]);
    float ey = (g_body[2]*am[0] - g_body[0]*am[2]);
    float ez = (g_body[0]*am[1] - g_body[1]*am[0]);
    float Kp = 0.5f; // tuneable
    // apply correction to quaternion via small angle
    float dphi[3] = { Kp*ex, Kp*ey, Kp*ez };
    // form delta quaternion (approx): [1, 0.5*dphi]
    float dq[4] = {1.0f, 0.5f*dphi[0], 0.5f*dphi[1], 0.5f*dphi[2]};
    // quaternion multiplication ekf->q = dq * ekf->q
    float q0 = dq[0]*ekf->q[0] - dq[1]*ekf->q[1] - dq[2]*ekf->q[2] - dq[3]*ekf->q[3];
    float q1 = dq[0]*ekf->q[1] + dq[1]*ekf->q[0] + dq[2]*ekf->q[3] - dq[3]*ekf->q[2];
    float q2 = dq[0]*ekf->q[2] - dq[1]*ekf->q[3] + dq[2]*ekf->q[0] + dq[3]*ekf->q[1];
    float q3 = dq[0]*ekf->q[3] + dq[1]*ekf->q[2] - dq[2]*ekf->q[1] + dq[3]*ekf->q[0];
    ekf->q[0]=q0; ekf->q[1]=q1; ekf->q[2]=q2; ekf->q[3]=q3;
    quat_normalize(ekf->q);

    // very simplified covariance update: reduce attitude uncertainty slightly
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            ekf->P[i][j] *= 0.95f; // shrink covariance
        }
    }
}
