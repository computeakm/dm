/* rc_bandpass.h
 * 简单 RC 带通滤波器（一级高通 + 一级低通 串联）
 * 用法：初始化时提供低截止频率 f_low（Hz）、高截止频率 f_high（Hz）和采样率 fs（Hz）。
 * 处理函数对单个浮点样本返回滤波后结果。
 *
 * 离散实现参考公式：
 *  高通（RC 高通，截止 f_low）： y_hp[n] = a_hp * ( y_hp[n-1] + x[n] - x[n-1] )
 *    a_hp = RC / (RC + dt), RC = 1/(2*pi*f_low), dt = 1/fs
 *  低通（RC 低通，截止 f_high）： y_lp[n] = y_lp[n-1] + a_lp * ( x_hp - y_lp[n-1] )
 *    a_lp = dt / (RC + dt), RC = 1/(2*pi*f_high)
 *
 * 注：f_low < f_high 且 f_high < fs/2
 */

#ifndef RC_BANDPASS_H
#define RC_BANDPASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct {
    float fs;         /* 采样率 Hz */
    float f_low;      /* 带通低截止频率 Hz (高通部分) */
    float f_high;     /* 带通高截止频率 Hz (低通部分) */

    /* 计算得到的系数 */
    float a_hp;       /* 高通 alpha */
    float a_lp;       /* 低通 alpha */

    /* 状态 */
    float x_prev;     /* 上次输入 x[n-1] */
    float y_hp_prev;  /* 上次高通输出 */
    float y_bp_prev;  /* 上次带通输出（即低通输出状态） */

    bool initialized;
} RC_Bandpass;

/* 初始化带通滤波器；返回 true 表示成功，false 表示参数无效 */
bool RC_Bandpass_Init(RC_Bandpass *bp, float f_low, float f_high, float fs);

/* 重置内部状态（零化），保留配置 */
void RC_Bandpass_Reset(RC_Bandpass *bp);

/* 逐样本处理 */
float RC_Bandpass_Process(RC_Bandpass *bp, float x);

/* 示例：
    RC_Bandpass bp;
    if (!RC_Bandpass_Init(&bp, 5.0f, 200.0f, 1000.0f)) { // f_low=5Hz, f_high=200Hz, fs=1kHz
        // 处理错误
    }
    float y = RC_Bandpass_Process(&bp, x_sample);
*/

#ifdef __cplusplus
}
#endif

#endif /* RC_BANDPASS_H */
