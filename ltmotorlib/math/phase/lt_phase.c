#include "math/phase/lt_phase.h"

/* 找到跨零点位置，求出相位滞后，四个点取平均 */
static float find_zero_cross(const float* buf, uint16_t start, uint16_t end) {
    for (uint16_t i = start; i < end - 1; i++) {
        if ((buf[i] > 0.0f && buf[i+1] <= 0.0f) ||
            (buf[i] < 0.0f && buf[i+1] >= 0.0f)) {
            float x0 = (float)i, x1 = (float)(i+1);
            float y0 = buf[i], y1 = buf[i+1];
            return x0 - y0 * (x1 - x0) / (y1 - y0);
        }
    }
    return -1.0f;
}

float lt_phase_calculate(const float* buf, uint16_t len, uint16_t m) {
    /* 每个周期点数（必须为整数，否则说明整周期条件不满足）*/
    uint16_t pts_per_cycle = len / m;
    float ideal_zero = (float)pts_per_cycle * 0.5f;

    /* 只检查前2个周期，共4个过零点 */
    uint16_t cycles_to_check = 2;
    float sum = 0.0f;
    uint8_t count = 0;

    for (uint16_t i = 0; i < cycles_to_check; i++) {
        uint16_t start1 = i * pts_per_cycle;
        uint16_t end1 = start1 + pts_per_cycle >> 1;
        float z1 = find_zero_cross(buf, start1, end1);
        if(z1 >= 0){
            sum += z1; 
            count++; 
        }

        uint16_t start2 = start1 + pts_per_cycle >> 1;
        uint16_t end2 = start1 + pts_per_cycle;
        float z2 = find_zero_cross(buf, start2, end2);
        if (z2 >= 0) { sum += z2; count++; }
    }

    if (count == 0) return 0.0f;

    float avg_zero = sum / count;
    float delta = avg_zero - ideal_zero;
    return 360.0f * delta / pts_per_cycle;
}