/*
 * SPDX-License-Identifier: Apache-2.0
 * PLL测速模块纯算法测试（不依赖硬件驱动）
 * 测试数据由 虚拟Encoder 产生
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "control/speed/lt_speed.h"
#include "encoder_test/encoder_sim/encoder_sim.h"

/* ============ 测试参数 ============ */
#define SPEED_TOLERANCE     5.0f         /* 速度误差容忍度 RPM */

/* ============ 测试用例 ============ */

/* 测试1: 静态测试（速度为0） */
static int test_static_zero_speed(void) {
    printf("\n========== 测试1: 静态零速测试 ==========\n");
    
    lt_speed_init(ENCODER_SIM_CPR, ENCODER_SIM_CALL_FREQ);
    encoder_sim_init(0.0);
    encoder_sim_set_speed(0.0);
    
    int pass = 1;
    
    /* 先运行100个周期让PLL稳定 */
    for (int i = 0; i < 100; i++) {
        encoder_sim_step();
        lt_speed_get(encoder_sim_get_count(Spindle_Axis));
    }
    
    /* 测试后续100个周期的速度输出 */
    float max_speed = 0;
    for (int i = 0; i < 100; i++) {
        encoder_sim_step();
        float speed = lt_speed_get(encoder_sim_get_count(Spindle_Axis));
        if (fabs(speed) > max_speed) max_speed = fabs(speed);
    }
    
    printf("期望速度: 0 RPM, 实际最大速度: %.2f RPM\n", max_speed);
    if (max_speed < 1.0f) {
        printf("✓ 测试通过\n");
    } else {
        printf("✗ 测试失败 (静态时速度输出过大)\n");
        pass = 0;
    }
    
    return pass;
}

/* 测试2: 恒速测试 */
static int test_constant_speed(float target_rpm) {
    printf("\n========== 测试2: 恒速测试 (%.0f RPM) ==========\n", target_rpm);
    
    lt_speed_init(ENCODER_SIM_CPR, ENCODER_SIM_CALL_FREQ);
    encoder_sim_init(0.0);
    encoder_sim_set_speed(target_rpm);
    
    int pass = 1;
    int total_steps = (int)(2.0f * ENCODER_SIM_CALL_FREQ);
    int settle_steps = (int)(0.1f * ENCODER_SIM_CALL_FREQ);
    
    float sum_speed = 0;
    float sum_error = 0;
    int stable_count = 0;
    
    for (int i = 0; i < total_steps; i++) {
        encoder_sim_step();
        float speed = lt_speed_get(encoder_sim_get_count(Spindle_Axis));
        
        if (i >= settle_steps) {
            sum_speed += speed;
            sum_error += fabs(speed - target_rpm);
            stable_count++;
        }
    }
    
    float avg_speed = sum_speed / stable_count;
    float avg_error = sum_error / stable_count;
    
    printf("目标速度: %.1f RPM\n", target_rpm);
    printf("平均估计速度: %.2f RPM\n", avg_speed);
    printf("平均误差: %.2f RPM (%.2f%%)\n", avg_error, avg_error / target_rpm * 100);
    
    if (avg_error < SPEED_TOLERANCE) {
        printf("✓ 测试通过\n");
    } else {
        printf("✗ 测试失败 (误差过大)\n");
        pass = 0;
    }
    
    return pass;
}

/* 测试3: 阶跃响应测试 */
static int test_step_response(void) {
    printf("\n========== 测试3: 阶跃响应测试 ==========\n");
    
    lt_speed_init(ENCODER_SIM_CPR, ENCODER_SIM_CALL_FREQ);
    encoder_sim_init(0.0);
    encoder_sim_set_speed(0.0);
    
    float dt = 1.0f / ENCODER_SIM_CALL_FREQ;
    int total_steps = (int)(0.5f * ENCODER_SIM_CALL_FREQ);
    int step_at = (int)(0.05f * ENCODER_SIM_CALL_FREQ);
    
    printf("时间(ms)\t实际速度\t估计速度\t误差\n");
    
    for (int i = 0; i < total_steps; i++) {
        if (i == step_at) {
            encoder_sim_set_speed(1000.0);
        }
        
        encoder_sim_step();
        float estimated_speed = lt_speed_get(encoder_sim_get_count(Spindle_Axis));
        
        if ((i % 50 == 0 && i <= 2500) || (i % 500 == 0 && i > 2500)) {
            float actual_speed = (i < step_at) ? 0.0f : 1000.0f;
            printf("%.1f\t\t%.1f\t\t%.1f\t\t%.1f\n", 
                   i * dt * 1000, actual_speed, estimated_speed, 
                   fabs(actual_speed - estimated_speed));
        }
    }
    
    /* 检查稳态误差 */
    float final_error = 0;
    for (int i = total_steps - 100; i < total_steps; i++) {
        encoder_sim_step();
        float speed = lt_speed_get(encoder_sim_get_count(Spindle_Axis));
        final_error += fabs(speed - 1000.0f);
    }
    final_error /= 100;
    
    printf("\n最终稳态误差: %.2f RPM\n", final_error);
    if (final_error < SPEED_TOLERANCE) {
        printf("✓ 测试通过\n");
        return 1;
    } else {
        printf("✗ 测试失败 (稳态误差过大)\n");
        return 0;
    }
}

/* 测试4: 正反转测试 */
static int test_bidirectional(void) {
    printf("\n========== 测试4: 正反转测试 ==========\n");
    
    lt_speed_init(ENCODER_SIM_CPR, ENCODER_SIM_CALL_FREQ);
    encoder_sim_init(0.0);
    encoder_sim_set_speed(500.0);
    
    int pass = 1;
    
    /* 阶段1: 正转500 RPM */
    printf("阶段1 - 正转500 RPM:\n");
    for (int i = 0; i < (int)(0.3f * ENCODER_SIM_CALL_FREQ); i++) {
        encoder_sim_step();
        lt_speed_get(encoder_sim_get_count(Spindle_Axis));
    }
    encoder_sim_step();
    float speed_fwd = lt_speed_get(encoder_sim_get_count(Spindle_Axis));
    printf("  估计速度: %.2f RPM\n", speed_fwd);
    
    /* 阶段2: 反转-500 RPM */
    printf("阶段2 - 反转-500 RPM:\n");
    encoder_sim_set_speed(-500.0);
    for (int i = 0; i < (int)(0.3f * ENCODER_SIM_CALL_FREQ); i++) {
        encoder_sim_step();
        lt_speed_get(encoder_sim_get_count(Spindle_Axis));
    }
    encoder_sim_step();
    float speed_rev = lt_speed_get(encoder_sim_get_count(Spindle_Axis));
    printf("  估计速度: %.2f RPM\n", speed_rev);
    
    if (speed_fwd > 400 && speed_fwd < 600 && 
        speed_rev < -400 && speed_rev > -600) {
        printf("✓ 测试通过\n");
    } else {
        printf("✗ 测试失败\n");
        pass = 0;
    }
    
    return pass;
}

/* 测试5: 高速跨圈测试 */
static int test_high_speed_wraparound(void) {
    printf("\n========== 测试5: 高速跨圈测试 (3000 RPM) ==========\n");
    
    lt_speed_init(ENCODER_SIM_CPR, ENCODER_SIM_CALL_FREQ);
    encoder_sim_init(0.0);
    encoder_sim_set_speed(3000.0);
    
    int pass = 1;
    int total_steps = (int)(0.5f * ENCODER_SIM_CALL_FREQ);
    int settle_steps = (int)(0.2f * ENCODER_SIM_CALL_FREQ);
    
    float sum_speed = 0;
    int sample_count = 0;
    
    for (int i = 0; i < total_steps; i++) {
        encoder_sim_step();
        float speed = lt_speed_get(encoder_sim_get_count(Spindle_Axis));
        
        if (i >= settle_steps) {
            sum_speed += speed;
            sample_count++;
        }
    }
    
    float avg_speed = sum_speed / sample_count;
    printf("目标速度: 3000 RPM, 平均估计速度: %.2f RPM\n", avg_speed);
    
    if (fabs(avg_speed - 3000.0) < SPEED_TOLERANCE * 3) {
        printf("✓ 测试通过\n");
    } else {
        printf("✗ 测试失败\n");
        pass = 0;
    }
    
    return pass;
}

/* 测试6: 斜坡变速测试 */
static int test_ramp_speed(void) {
    printf("\n========== 测试6: 斜坡变速测试 ==========\n");
    
    lt_speed_init(ENCODER_SIM_CPR, ENCODER_SIM_CALL_FREQ);
    encoder_sim_init(0.0);
    
    float dt = 1.0f / ENCODER_SIM_CALL_FREQ;
    int pass = 1;
    
    /* 斜坡参数 */
    float ramp_up_start = 0.2f, ramp_up_end = 0.6f;
    float ramp_up_rate = 2000.0f / (ramp_up_end - ramp_up_start);
    float const1_end = 1.0f;
    float ramp_down_start = 1.0f, ramp_down_end = 1.2f;
    float ramp_down_rate = -1500.0f / (ramp_down_end - ramp_down_start);
    float const2_end = 1.6f;
    float ramp_rev_start = 1.6f, ramp_rev_end = 1.9f;
    float ramp_rev_rate = -1500.0f / (ramp_rev_end - ramp_rev_start);
    
    float total_time = 2.3f;
    int total_steps = (int)(total_time * ENCODER_SIM_CALL_FREQ);
    
    printf("时间(s)\t\t目标速度\t估计速度\t误差\n");
    
    float max_error = 0, sum_error = 0;
    int sample_count = 0;
    
    for (int i = 0; i < total_steps; i++) {
        float time = i * dt;
        float target_speed;
        
        if (time < ramp_up_start) {
            target_speed = 0.0f;
        } else if (time < ramp_up_end) {
            target_speed = (time - ramp_up_start) * ramp_up_rate;
        } else if (time < const1_end) {
            target_speed = 2000.0f;
        } else if (time < ramp_down_end) {
            target_speed = 2000.0f + (time - ramp_down_start) * ramp_down_rate;
        } else if (time < const2_end) {
            target_speed = 500.0f;
        } else if (time < ramp_rev_end) {
            target_speed = 500.0f + (time - ramp_rev_start) * ramp_rev_rate;
        } else {
            target_speed = -1000.0f;
        }
        
        encoder_sim_set_speed(target_speed);
        encoder_sim_step();
        float estimated_speed = lt_speed_get(encoder_sim_get_count(Spindle_Axis));
        
        if (i > (int)(0.05f * ENCODER_SIM_CALL_FREQ)) {
            float error = fabs(estimated_speed - target_speed);
            if (error > max_error) max_error = error;
            sum_error += error;
            sample_count++;
        }
        
        if (i % 1250 == 0) {
            printf("%.2f\t\t%.1f\t\t%.1f\t\t%.1f\n", 
                   time, target_speed, estimated_speed,
                   fabs(target_speed - estimated_speed));
        }
    }
    
    float avg_error = sum_error / sample_count;
    printf("\n最大跟踪误差: %.2f RPM, 平均误差: %.2f RPM\n", max_error, avg_error);
    
    if (max_error < 100.0f && avg_error < 20.0f) {
        printf("✓ 测试通过\n");
    } else {
        printf("✗ 测试失败\n");
        pass = 0;
    }
    
    return pass;
}

/* ============ 主测试入口 ============ */
int encoder_speed_test(void) {
    printf("╔══════════════════════════════════════╗\n");
    printf("║   PLL测速模块纯算法测试             ║\n");
    printf("║   编码器: %d CPR                   ║\n", ENCODER_SIM_CPR);
    printf("║   采样频率: %.0f Hz                ║\n", ENCODER_SIM_CALL_FREQ);
    printf("╚══════════════════════════════════════╝\n");
    
    int passed = 0;
    int total = 0;
    
    total++; passed += test_static_zero_speed();
    total++; passed += test_constant_speed(100.0);
    total++; passed += test_constant_speed(1000.0);
    total++; passed += test_step_response();
    total++; passed += test_bidirectional();
    total++; passed += test_high_speed_wraparound();
    total++; passed += test_ramp_speed();
    
    printf("\n╔══════════════════════════════════════╗\n");
    printf("║   测试结果: %d/%d 通过               ║\n", passed, total);
    if (passed == total) {
        printf("║   ✓ 所有测试通过!                   ║\n");
    } else {
        printf("║   ✗ 有 %d 个测试失败                ║\n", total - passed);
    }
    printf("╚══════════════════════════════════════╝\n");
    
    return (passed == total) ? 0 : 1;
}