#ifndef LT_LSQ_H
#define LT_LSQ_H

#include <stdint.h>

/* 最小二乘线性回归：y = a0 + a1*x1 + a2*x2 + ... + an*xn
 *   x    : 自变量数组，大小为 m x n，行优先存储（x[样本][变量]）
 *   y    : 因变量数组，大小为 m
 *   n    : 自变量个数（1 ~ 3）
 *   m    : 样本点数（必须 >= n + 1）
 *   coeff: 输出回归系数，大小为 n+1（coeff[0] 为常数项 a0）
 *   返回 1：成功，0：失败（样本不足或矩阵奇异）
 */
uint8_t lt_lsq_solve(const float *x, const float *y, uint8_t n, uint16_t m, float *coeff);

#endif