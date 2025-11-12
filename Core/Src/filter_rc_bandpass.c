/*
 * filter_rc_bandpass.c
 *
 * 简单 RC 带通滤波器实现：先一阶高通，再一阶低通。
 * 计算代价低，适用于微控制器上的采样数据（例如 ADC）。
 */

#include "filter_rc_bandpass.h"
#include <math.h>

/* 在某些工具链上可能没有定义 M_PI，做兼容处理 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif



void filter_rc_bandpass_init(FilterRCBandpass *f, float fs, float f_low, float f_high)
{
    if (!f) return;

    if (fs <= 0.0f) fs = 1.0f; /* avoid div-zero; caller should provide sensible fs */
    f->fs = fs;
    f->f_low = f_low;
    f->f_high = f_high;
    f->dt = 1.0f / f->fs;

     /* 对于简单的 RC 低通：alpha_lp = dt / (RC + dt)， RC = 1/(2*pi*fc)
         对于简单的一阶高通离散形式：y[n] = a*(y[n-1] + x[n] - x[n-1])
         其中 a = RC/(RC + dt)，RC = 1/(2*pi*fc) */

    float rc_hp = 0.0f;
    float rc_lp = 0.0f;

    if (f_low > 0.0f) {
        rc_hp = 1.0f / (2.0f * M_PI * f_low);
        f->alpha_hp = rc_hp / (rc_hp + f->dt);
    } else {
        /* 如果 f_low == 0，禁用高通（alpha_hp -> 0 相当于直通） */
        f->alpha_hp = 0.0f;
    }

    if (f_high > 0.0f) {
        rc_lp = 1.0f / (2.0f * M_PI * f_high);
        f->alpha_lp = f->dt / (rc_lp + f->dt);
    } else {
        /* 如果 f_high == 0，禁用低通（alpha_lp -> 1 相当于直通） */
        f->alpha_lp = 1.0f;
    }

    /* Initialize state */
    f->lp_prev = 0.0f;
    f->hp_prev = 0.0f;
    f->x_prev = 0.0f;
}

float filter_rc_bandpass_process(FilterRCBandpass *f, float x)
{
    if (!f) return x;

    float hp = 0.0f;

    if (f->alpha_hp > 0.0f) {
        /* 高通差分方程 */
        hp = f->alpha_hp * (f->hp_prev + x - f->x_prev);
    } else {
        /* 无高通 -> 直通 */
        hp = x;
    }

    /* update previous input and hp state */
    f->x_prev = x;
    f->hp_prev = hp;

    float lp = 0.0f;
    if (f->alpha_lp >= 1.0f) {
        /* 无低通 -> 直通 */
        lp = hp;
    } else {
        /* 低通平滑 */
        lp = f->lp_prev + f->alpha_lp * (hp - f->lp_prev);
    }

    f->lp_prev = lp;
    return lp;
}

void filter_rc_bandstop_init(FilterRCBandstop *f, float fs, float f_low, float f_high)
{
     if (!f) return;

    if (fs <= 0.0f) fs = 1.0f; /* avoid div-zero; caller should provide sensible fs */
    f->fs = fs;
    f->f_low = f_low;
    f->f_high = f_high;
    f->dt = 1.0f / f->fs;

     /* 对于简单的 RC 低通：alpha_lp = dt / (RC + dt)， RC = 1/(2*pi*fc)
         对于简单的一阶高通离散形式：y[n] = a*(y[n-1] + x[n] - x[n-1])
         其中 a = RC/(RC + dt)，RC = 1/(2*pi*fc) */

    float rc_hp = 0.0f;
    float rc_lp = 0.0f;

    if (f_low > 0.0f) {
        rc_hp = 1.0f / (2.0f * M_PI * f_low);
        f->alpha_hp = rc_hp / (rc_hp + f->dt);
    } else {
        /* If f_low == 0, disable high-pass (alpha_hp -> 0 behaves like passthrough) */
        f->alpha_hp = 0.0f;
    }

    if (f_high > 0.0f) {
        rc_lp = 1.0f / (2.0f * M_PI * f_high);
        f->alpha_lp = f->dt / (rc_lp + f->dt);
    } else {
        /* If f_high == 0, disable low-pass (alpha_lp -> 1 behaves like passthrough) */
        f->alpha_lp = 1.0f;
    }

    /* Initialize state */
    f->lp_prev = 0.0f;
    f->hp_prev = 0.0f;
    f->x_prev = 0.0f;
}

float filter_rc_bandstop_process(FilterRCBandstop *f, float x)
{
    if (!f) return x;

    float hp = 0.0f;

    if (f->alpha_hp > 0.0f) {
        /* 高通差分方程 */
        hp = f->alpha_hp * (f->hp_prev + x - f->x_prev);
    } else {
        /* 无高通 -> 直通 */
        hp = x;
    }

    /* update previous input and hp state */
    f->x_prev = x;
    f->hp_prev = hp;

    float lp = 0.0f;
    if (f->alpha_lp >= 1.0f) {
        /* 无低通 -> 直通 */
        lp = hp;
    } else {
        /* 低通平滑 */
        lp = f->lp_prev + f->alpha_lp * (hp - f->lp_prev);
    }

    f->lp_prev = lp;
    return x-lp;
}


void filter_rc_bandpass_reset(FilterRCBandpass *f)
{
    if (!f) return;
    f->lp_prev = 0.0f;
    f->hp_prev = 0.0f;
    f->x_prev = 0.0f;
}

/* ---------------------------- 低通实现 ---------------------------- */

void filter_rc_lowpass_init(FilterRCLowpass *f, float fs, float fc)
{
    if (!f) return;

    if (fs <= 0.0f) fs = 1.0f;
    f->fs = fs;
    f->fc = fc;
    f->dt = 1.0f / f->fs;

    if (fc > 0.0f) {
        float rc = 1.0f / (2.0f * M_PI * fc);
        f->alpha = f->dt / (rc + f->dt);
    } else {
        /* fc <= 0 => 直通 */
        f->alpha = 1.0f;
    }

    f->y_prev = 0.0f;
}

float filter_rc_lowpass_process(FilterRCLowpass *f, float x)
{
    if (!f) return x;

    if (f->alpha >= 1.0f) {
        f->y_prev = x;
        return x;
    }

    float y = f->y_prev + f->alpha * (x - f->y_prev);
    f->y_prev = y;
    return y;
}

void filter_rc_lowpass_reset(FilterRCLowpass *f)
{
    if (!f) return;
    f->y_prev = 0.0f;
}

/* ---------------------------- 平滑滤波辅助函数 ---------------------------- */

/* 指数移动平均 (EMA) 帮助函数
 * 用法：
 *   - 根据采样率 fs 和截止频率 fc 计算 alpha：
 *       alpha = dt / (RC + dt)，RC = 1/(2*pi*fc)，dt = 1/fs
 *   - 每个采样调用 smooth_ema(prev_y, x, alpha) 并返回新的 y
 * 该函数无状态，易于集成到现有代码中。
 */
float ema_alpha_from_fc(float fs, float fc)
{
    if (fs <= 0.0f) return 1.0f;
    if (fc <= 0.0f) return 1.0f; /* 透传 */
    float dt = 1.0f / fs;
    float rc = 1.0f / (2.0f * M_PI * fc);
    return dt / (rc + dt);
}

float smooth_ema(float y_prev, float x, float alpha)
{
    /* alpha ∈ [0,1]；alpha==1 => 直接透传 */
    if (alpha >= 1.0f) return x;
    if (alpha <= 0.0f) return y_prev; /* 不更新 */
    return y_prev + alpha * (x - y_prev);
}

/* 简单移动平均（N 点）实现，使用调用者提供的缓冲区以避免 malloc。
 * 调用者必须分配一个长度 >= N 的浮点数组并传入 movavg_init。
 * 状态结构很小，可放在 .bss 或栈中。
 *
 * 示例：
 *   static float buf[16];
 *   MovAvgState st;
 *   movavg_init(&st, buf, 16);
 *   float y = movavg_process(&st, x);
 */

void movavg_init(MovAvgState *s, float *buffer, int N)
{
    if (!s || !buffer || N <= 0) return;
    s->N = N;
    s->idx = 0;
    s->count = 0;
    s->sum = 0.0f;
    s->buf = buffer;
    for (int i = 0; i < N; ++i) buffer[i] = 0.0f;
}

float movavg_process(MovAvgState *s, float x)
{
    if (!s || !s->buf || s->N <= 0) return x;
    if (s->count < s->N) {
        /* 填充阶段 */
        s->sum += x;
        s->buf[s->idx++] = x;
        s->count++;
        if (s->idx >= s->N) s->idx = 0;
        return s->sum / (float)s->count;
    } else {
        /* 稳定阶段 */
        s->sum -= s->buf[s->idx];
        s->sum += x;
        s->buf[s->idx] = x;
        if (++s->idx >= s->N) s->idx = 0;
        return s->sum / (float)s->N;
    }
}

/* 可选：简单的双指数平滑（Holt 线性），可用于含趋势数据的平滑。
 * 状态：level（水平）与 trend（趋势）。alpha 为水平权重，beta 为趋势权重（0..1）。
 * 初始时 y0 = x0，b0 = x1-x0；更新公式：
 *   level = alpha*x + (1-alpha)*(level_prev + trend_prev)
 *   trend = beta*(level - level_prev) + (1-beta)*trend_prev
 */

void doubleexp_init(DoubleExpState *s)
{
    if (!s) return;
    s->level = 0.0f;
    s->trend = 0.0f;
    s->initialized = 0;
}

float doubleexp_process(DoubleExpState *s, float x, float alpha, float beta)
{
    if (!s) return x;
    if (!s->initialized) {
        s->level = x;
        s->trend = 0.0f;
        s->initialized = 1;
        return x;
    }
    float prev_level = s->level;
    s->level = alpha * x + (1.0f - alpha) * (s->level + s->trend);
    s->trend = beta * (s->level - prev_level) + (1.0f - beta) * s->trend;
    return s->level;
}

/* ---------------- 高斯滤波器实现 ---------------- */

int gaussian_radius_for_sigma(float sigma, float trunc)
{
    if (sigma <= 0.0f) return 0;
    if (trunc <= 0.0f) trunc = 3.0f;
    int r = (int)ceilf(trunc * sigma);
    if (r < 0) r = 0;
    return r;
}

void gaussian_build_kernel(float kernel[], int radius, float sigma)
{
    if (radius <= 0 || sigma <= 0.0f || !kernel) {
        if (kernel && radius == 0) kernel[0] = 1.0f; /* degenerate */
        return;
    }

    int N = 2 * radius + 1;
    const float inv_two_sigma2 = 1.0f / (2.0f * sigma * sigma);
    float sum = 0.0f;

    for (int i = -radius; i <= radius; ++i) {
        float v = expf(- (i * i) * inv_two_sigma2);
        kernel[i + radius] = v;
        sum += v;
    }

    /* 归一化，使权重和为 1 */
    if (sum <= 0.0f) {
        /* 避免除零 */
        for (int i = 0; i < N; ++i) kernel[i] = 0.0f;
        kernel[radius] = 1.0f;
        return;
    }
    float invsum = 1.0f / sum;
    for (int i = 0; i < N; ++i) kernel[i] *= invsum;
}

void gaussian_init(GaussianFilterState *s, float *buffer, int N)
{
    if (!s || !buffer || N <= 0) return;
    s->N = N;
    s->idx = 0;
    s->buf = buffer;
    /* 初始化缓冲为 0 */
    for (int i = 0; i < N; ++i) buffer[i] = 0.0f;
}

float gaussian_process(GaussianFilterState *s, const float kernel[], int radius, float x)
{
    if (!s || !s->buf || radius < 0 || !kernel) return x;
    int N = s->N;
    if (N != 2 * radius + 1) {
        /* 不匹配：如果 kernel 长度与状态不匹配，直接返回输入 */
        return x;
    }

    /* 将最新样本写入当前位置 */
    s->buf[s->idx] = x;

    /* 以当前写入位置为中心做卷积：kernel 中心对应最近样本 */
    float y = 0.0f;
    for (int k = -radius; k <= radius; ++k) {
        int buf_idx = s->idx + k;
        /* 环形索引 */
        if (buf_idx < 0) buf_idx += N;
        else if (buf_idx >= N) buf_idx -= N;
        y += kernel[k + radius] * s->buf[buf_idx];
    }

    /* 推进写指针为下次写入做准备 */
    s->idx++;
    if (s->idx >= N) s->idx = 0;

    return y;
}




