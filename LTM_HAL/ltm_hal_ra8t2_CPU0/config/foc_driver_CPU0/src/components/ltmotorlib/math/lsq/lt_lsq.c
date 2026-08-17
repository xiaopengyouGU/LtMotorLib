/* lt_lsq.c */
#include "math/lsq/lt_lsq.h"
#include "math/basic/lt_math.h"
#include <string.h>

/* 高斯消元法解 Ax = b，A 为 n 阶方阵，返回 1 成功 0 失败 */
static uint8_t _gauss_solve(float *A, float *b, uint8_t n);

/* 线性回归，最多支持3变量 */
uint8_t lt_lsq_solve(const float *x, const float *y, uint8_t n, uint16_t m, float *coeff)
{
    if (!x || !y || n == 0 || n > 3 || m < n + 1 || !coeff) return 0;

    /* 构造正规方程 A * coeff = b
     *   A[i][j] = sum_k x[k][i] * x[k][j], 其中 x[k][0] = 1（常数项）
     *   b[i]    = sum_k x[k][i] * y[k]
     */
    float A[3][3] = {0};
    float b[3] = {0};

    for (uint16_t k = 0; k < m; k++) {      /* m : 样本点数 */
        const float *xk = &x[k * n];        /* 行优先存储 */
        float yk = y[k];

        /* 常数项 x0 = 1 */
        A[0][0] += 1.0f;
        for (uint8_t i = 0; i < n; i++) {
            A[0][i + 1] += xk[i];
            A[i + 1][0] += xk[i];
            b[i + 1] += xk[i] * yk;
            for (uint8_t j = 0; j < n; j++) {
                A[i + 1][j + 1] += xk[i] * xk[j];
            }
        }
        b[0] += yk;
    }

    /* 解正规方程 */
    if (!_gauss_solve(&A[0][0], b, n + 1)) return 0;

    for (uint8_t i = 0; i <= n; i++) coeff[i] = b[i];
    return 1;
}

/*****************************************************************************/
/* 高斯消元法解 Ax = b，A 为 n 阶方阵，返回 1 成功 0 失败 */
static uint8_t _gauss_solve(float *A, float *b, uint8_t n)
{
    for (uint8_t col = 0; col < n; col++) {
        /* 选主元 */
        uint8_t pivot = col;
        for (uint8_t row = col + 1; row < n; row++) {
            if (lt_absf(A[row * n + col]) > lt_absf(A[pivot * n + col])) pivot = row;
        }
        if (pivot != col) {
            for (uint8_t j = col; j < n; j++) {
                float tmp = A[col * n + j];
                A[col * n + j] = A[pivot * n + j];
                A[pivot * n + j] = tmp;
            }
            float tmp = b[col];
            b[col] = b[pivot];
            b[pivot] = tmp;
        }
        if (lt_absf(A[col * n + col]) < 1e-12f) return 0;

        /* 消去 */
        for (uint8_t row = col + 1; row < n; row++) {
            float factor = A[row * n + col] / A[col * n + col];
            for (uint8_t j = col; j < n; j++) {
                A[row * n + j] -= factor * A[col * n + j];
            }
            b[row] -= factor * b[col];
        }
    }

    /* 回代 */
    for (int row = n - 1; row >= 0; row--) {
        float sum = b[row];
        for (uint8_t col = row + 1; col < n; col++) {
            sum -= A[row * n + col] * b[col];
        }
        b[row] = sum / A[row * n + row];
    }
    return 1;
}