//=====================================================================================================
// MahonyAHRS.h
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
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration
#include <math.h>
#include "usart.h"
#include "stdio.h"
#include "bmi088driver.h"
#include "buzzer.h"
#include "string.h"
#include "main.h"



typedef struct MahonyAHRS
{
    float twoKp;			// 2 * proportional gain (Kp)
    float twoKi;			// 2 * integral gain (Ki)
    float sampleFreq;       // sample frequency in Hz
    float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
    float integralFBx, integralFBy, integralFBz;	// integral error terms scaled by Ki
    float roll, pitch, yaw;
    int anglesComputed;
}MahonyAHRS;

typedef struct {
    float kx, ky, kz;
    float bx, by, bz;
} AccelCalibParam;

typedef struct {
    float x,y,z;
	  uint32_t time;
} UAV_xyz;



//---------------------------------------------------------------------------------------------------
// Function declarations
// 下面的注释说明每个函数的用途、输入/输出参考系（world/world frame vs body/body frame）、角度单位（均为弧度）

/*
 * 加速度标定与校正（机体坐标系）
 * - calibrate_accel_six_position: 输入为六面放置测得的静态加速度（机体坐标系），g 为重力加速度近似值
 * - accel_correct: 将原始加速度 raw（机体坐标系）校正为 corrected（机体坐标系）
 */
void calibrate_accel_six_position(const float axp, const float axn,
                                 const float ayp, const float ayn,
                                 const float azp, const float azn,
                                 float g, AccelCalibParam* param);
void accel_correct(const float raw[3], const AccelCalibParam* param, float corrected[3]);
void calibrate_accel_six_position_test(AccelCalibParam *param);

/* Mahony AHRS API
 * - 四元数约定： (q0,q1,q2,q3) => (w,x,y,z)
 * - 四元数表示：四元数 q 表示从机体坐标系(body)到参考/世界坐标系(world)的旋转（即 v_world = Q ⊗ v_body ⊗ Q*）
 * - 所有角度均为弧度（rad）
 */
void MahonyAHRS_init(MahonyAHRS* ahrs);
void MahonyAHRSupdate(MahonyAHRS* ahrs, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(MahonyAHRS* ahrs, float gx, float gy, float gz, float ax, float ay, float az);
void Mahony_computeAngles(MahonyAHRS* ahrs);
// 将 ahrs 内的四元数转换为 ZYX 顺序的欧拉角（yaw, pitch, roll），并存回 ahrs->yaw/pitch/roll
void QuaternionToEulerZYX(MahonyAHRS* ahrs);
void MahonyAHRS_setSampleFreq(MahonyAHRS* ahrs, float DT);
// 将 ahrs 四元数转换为方向余弦矩阵（DCM），DCM 描述 body->world 的变换
void QuaternionToDCM(MahonyAHRS* ahrs, float *R[3][3]);
void DCMToEulerZYX(float *R[3][3],MahonyAHRS* ahrs);

/* 坐标变换辅助函数
 * - turn_body_to_world: 使用 ahrs 中的四元数将机体速度/向量转换为世界坐标
 * - WorldToBodyQuaternion/WorldToBodyFromMahony: 将世界向量 v_world 转入机体向量 v_body
 *   注意：WorldToBodyQuaternion 的 q 参数应为 body->world 四元数 (w,x,y,z)
 */
void turn_body_to_world(MahonyAHRS ahrs,UAV_xyz *word_speed,UAV_xyz *body_speed);
void WorldToBodyQuaternion(float q[4], float *v_world, float *v_body);
void WorldToBodyFromMahony(MahonyAHRS* ahrs,float *v_world, float *v_body);
void WorldToBodyFromMahony_xyz(MahonyAHRS* ahrs,UAV_xyz *word_xyz,UAV_xyz *body_xyz);

/* 误差四元数与三轴角度误差（axis-angle / small-angle approx）
 * 说明通用约定：
 * - q_ref, q_est: 均为 body->world 的四元数
 * - 误差四元数 q_err = q_ref ⊗ conj(q_est)
 * - q_err 将把从估计姿态到参考姿态的旋转表示为一个四元数（默认参考/世界参考系）
 * - ErrorQuaternionToAngle 输出为长度为 3 的向量，表示沿 x/y/z 轴的等效小角（弧度），此向量在 q_err 所在参考系中（通常为世界/参考系）
 * - ErrorQuaternionToBodyAngle 将 q_err 映射到机体坐标系后输出角度向量（便于直接用于机体轴 PID）
 */
void Quaternion_compute_to_errorangle(float yaw, float pitch, float roll,MahonyAHRS *ahrs, float *angle);
// 忽略参考 yaw 成分后计算误差角（适合航向不可观测或不关心航向的场景）
void Quaternion_compute_to_errorangle_ignoreyaw(float yaw, float pitch, float roll, MahonyAHRS *ahrs, float *angle);

/* ComputeErrorQuaternionBodyFrame
 * - 输入：q_ref, q_est（长度4数组，按 w,x,y,z 排列），均表示 body->world
 * - 输出：q_err_body（长度4数组），表示在机体坐标系下的误差四元数（即将世界/参考系的误差转换到机体轴）
 * - 用法：当后续控制器期望在机体轴上直接使用角度误差时，调用此函数得到机体参考的误差四元数
 */
void ComputeErrorQuaternionBodyFrame(float *q_ref,float *q_est, float* q_err_body);

// 包装函数：输入为欧拉角（ZYX, yaw,pitch,roll），输出机体参考系下的误差四元数
void Quaternion_compute_error_quat_body_from_euler(float yaw, float pitch, float roll, MahonyAHRS *ahrs, float *q_err_body);
// 包装函数：输入为机体顺序的欧拉角 (roll,pitch,yaw)，更贴近机体角度输入惯例
void Quaternion_compute_error_quat_body_from_bodyEuler(float roll, float pitch, float yaw, MahonyAHRS *ahrs, float *q_err_body);
// 包装函数：输入为机体顺序的欧拉角 (roll,pitch,yaw)，输出机体参考系下的 3 轴角度误差（弧度）
void Quaternion_compute_to_errorangle_bodyEuler(float roll, float pitch, float yaw, MahonyAHRS *ahrs, float *angle_body);

/* ErrorQuaternionToBodyAngleFromMahony
 * - 将给定的 q_err（参考/世界系）使用 ahrs 的当前四元数转换到机体坐标系并返回三个轴上的角度误差
 * - angle_body 输出为 {ex, ey, ez}（弧度），可直接用作机体轴的控制量（小角近似）
 */
void ErrorQuaternionToBodyAngleFromMahony(float *q_err,MahonyAHRS *ahrs, float *angle_body);
void WorldToBodyFromMahony_xyz(MahonyAHRS* ahrs,UAV_xyz *word_xyz,UAV_xyz *body_xyz);
// ---------------------- 平滑滤波（一阶低通） ----------------------
typedef struct {
    float alpha;       // 滤波系数（0..1）
    float prev;        // 上一次输出值
    int initialized;   // 是否已初始化
} LPF_t;

typedef struct {
    LPF_t x;
    LPF_t y;
    LPF_t z;
} LPF3_t;

// cutoff_hz: 截止频率（Hz）， sample_hz: 采样频率（Hz）
void LPF_Init(LPF_t *f, float cutoff_hz, float sample_hz);
float LPF_Update(LPF_t *f, float in);
void LPF_Reset(LPF_t *f);

void LPF3_Init(LPF3_t *f3, float cutoff_hz, float sample_hz);
void LPF3_Update(LPF3_t *f3, const float in[3], float out[3]);
void LPF3_Reset(LPF3_t *f3);
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
