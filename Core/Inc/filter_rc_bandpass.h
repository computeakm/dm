/*
 * filter_rc_bandpass.h
 *
 * RC 带通 / 带阻 / 低通 滤波器及若干平滑滤波辅助类型和函数的声明
 * 注：本头文件与实现配套，声明在 Core/Inc 下，便于项目其他模块包含使用。
 */

#ifndef FILTER_RC_BANDPASS_H
#define FILTER_RC_BANDPASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

/* 带通滤波器状态结构体 */
typedef struct {
    float fs;       /* 采样率 (Hz) */
    float f_low;    /* 高通截止频率 (Hz) */
    float f_high;   /* 低通截止频率 (Hz) */
    float dt;       /* 采样周期 (s) */
    float alpha_hp; /* 高通滤波系数 */
    float alpha_lp; /* 低通滤波系数 */
    float lp_prev;  /* 低通上一次输出 */
    float hp_prev;  /* 高通上一次输出 */
    float x_prev;   /* 上一次输入 */
} FilterRCBandpass;

/* 带阻滤波器状态结构（与带通类似） */
typedef struct {
    float fs;
    float f_low;
    float f_high;
    float dt;
    float alpha_hp;
    float alpha_lp;
    float lp_prev;
    float hp_prev;
    float x_prev;
} FilterRCBandstop;

/* 低通滤波器状态结构 */
typedef struct {
    float fs;    /* 采样率 */
    float fc;    /* 截止频率 */
    float dt;    /* 采样周期 */
    float alpha; /* 平滑系数 */
    float y_prev;/* 上一次输出 */
} FilterRCLowpass;

/* 带通 / 带阻 / 低通 API */
void filter_rc_bandpass_init(FilterRCBandpass *f, float fs, float f_low, float f_high);
float filter_rc_bandpass_process(FilterRCBandpass *f, float x);
void filter_rc_bandpass_reset(FilterRCBandpass *f);

void filter_rc_bandstop_init(FilterRCBandstop *f, float fs, float f_low, float f_high);
float filter_rc_bandstop_process(FilterRCBandstop *f, float x);

void filter_rc_lowpass_init(FilterRCLowpass *f, float fs, float fc);
float filter_rc_lowpass_process(FilterRCLowpass *f, float x);
void filter_rc_lowpass_reset(FilterRCLowpass *f);

/* ---------------- 平滑滤波辅助函数与类型（实现位于 filter_rc_bandpass.c） ---------------- */

/* 根据采样率 fs 和截止频率 fc 计算 EMA 的 alpha 系数 */
float ema_alpha_from_fc(float fs, float fc);

/* 单步 EMA：给定上一输出 y_prev，当前输入 x，以及 alpha，返回新的输出 */
float smooth_ema(float y_prev, float x, float alpha);

/* 简单移动平均（N 点），调用者需提供缓冲区以避免动态分配 */
typedef struct {
    int N;
    int idx;
    int count;
    float sum;
    float *buf;
} MovAvgState;

void movavg_init(MovAvgState *s, float *buffer, int N);
float movavg_process(MovAvgState *s, float x);

/* 双指数平滑（Holt）状态与 API */
typedef struct {
    float level;
    float trend;
    int initialized;
} DoubleExpState;

void doubleexp_init(DoubleExpState *s);
float doubleexp_process(DoubleExpState *s, float x, float alpha, float beta);

/* ---------------- 高斯滤波器 API ---------------- */
/* 说明：
 * - 为避免在嵌入式上动态分配，核数组与样本缓冲区均由调用者分配并传入。
 * - kernel 长度为 2*radius+1，对称并归一化（和为 1）。
 * - buffer 长度应等于 kernel 长度，用作环形缓冲保存最近 N=2*radius+1 个样本。
 */

/* 根据 sigma（标准差）和截断倍数 trunc（通常取 3）计算推荐的 radius（半径） */
int gaussian_radius_for_sigma(float sigma, float trunc);

/* 生成高斯核到 caller 提供的 kernel 数组（长度 = 2*radius+1），并归一化 */
void gaussian_build_kernel(float kernel[], int radius, float sigma);

/* 高斯滤波状态（仅维护环形缓冲索引和长度） */
typedef struct {
    int N;      /* kernel 长度 = 2*radius+1 */
    int idx;    /* 下一个写入位置（环形缓冲索引） */
    float *buf; /* 指向调用者提供的缓冲区，长度为 N */
} GaussianFilterState;

/* 初始化高斯滤波状态，buffer 必须由 caller 分配且长度为 N */
void gaussian_init(GaussianFilterState *s, float *buffer, int N);

/* 每样本处理：将当前样本 x 写入环形缓冲并对 kernel 做离散卷积，返回滤波后值
 * kernel 长度应为 N，radius = (N-1)/2
 */
float gaussian_process(GaussianFilterState *s, const float kernel[], int radius, float x);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_RC_BANDPASS_H */
