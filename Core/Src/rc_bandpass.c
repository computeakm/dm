/* rc_bandpass.c
 * 简单 RC 带通滤波器实现（一级高通 + 一级低通）
 */

#include "rc_bandpass.h"
#include <math.h>

bool RC_Bandpass_Init(RC_Bandpass *bp, float f_low, float f_high, float fs)
{
    if (!bp) return false;
    if (fs <= 0.0f) return false;
    if (!(f_low > 0.0f && f_high > f_low && f_high < fs * 0.5f)) return false;

    bp->fs = fs;
    bp->f_low = f_low;
    bp->f_high = f_high;

    const float dt = 1.0f / fs;
    const float rc_hp = 1.0f / (2.0f * (float)M_PI * f_low);
    const float rc_lp = 1.0f / (2.0f * (float)M_PI * f_high);

    bp->a_hp = rc_hp / (rc_hp + dt); /* 高通 */
    bp->a_lp = dt / (rc_lp + dt);    /* 低通 */

    bp->x_prev = 0.0f;
    bp->y_hp_prev = 0.0f;
    bp->y_bp_prev = 0.0f;
    bp->initialized = true;

    return true;
}

void RC_Bandpass_Reset(RC_Bandpass *bp)
{
    if (!bp) return;
    bp->x_prev = 0.0f;
    bp->y_hp_prev = 0.0f;
    bp->y_bp_prev = 0.0f;
}

float RC_Bandpass_Process(RC_Bandpass *bp, float x)
{
    if (!bp || !bp->initialized) return 0.0f;

    /* 高通： y_hp[n] = a_hp * ( y_hp[n-1] + x[n] - x[n-1] ) */
    float y_hp = bp->a_hp * (bp->y_hp_prev + x - bp->x_prev);

    /* 低通： y_lp[n] = y_lp[n-1] + a_lp * ( y_hp - y_lp[n-1] ) */
    float y_lp = bp->y_bp_prev + bp->a_lp * (y_hp - bp->y_bp_prev);

    /* 更新状态 */
    bp->x_prev = x;
    bp->y_hp_prev = y_hp;
    bp->y_bp_prev = y_lp;

    return y_lp;
}
