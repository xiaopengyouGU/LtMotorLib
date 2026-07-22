#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "bsp/encoder/encoder.h"
#include "encoder_test/encoder_sim/encoder_sim.h"
#include "math/crc/lt_crc.h"            

/* ============ 测试用例 ============ */

/* 测试1: 上电位置解算 */
static int test_startup_position(void) {
    printf("\n========== 测试1: 上电位置解算 (初始第7.71圈) ==========\n");
    
    double target_turn = 7.71;
    encoder_sim_init(target_turn);
    encoder_init();
    encoder_set_crc8_check(lt_crc8_check);      /* 这一步不能少 */
    
    for (int i = 0; i < 8; i++) encoder_update();
    
    float actual_turns = encoder_get_position() / (float)ENCODER_SIM_CPR;
    printf("期望圈数: %.3f, 解算圈数: %.3f\n", target_turn, actual_turns);
    
    if (fabs(actual_turns - target_turn) < 0.1) {
        printf("✓ 测试通过\n");
        return 1;
    } else {
        printf("✗ 测试失败\n");
        return 0;
    }
}

/* 测试2: 实时位置跟踪 */
static int test_position_tracking(void) {
    printf("\n========== 测试2: 实时位置跟踪 ==========\n");
    
    encoder_sim_init(3.5);
    encoder_init();
    encoder_set_crc8_check(lt_crc8_check);      /* 这一步不能少 */
    
    for (int i = 0; i < 8; i++) encoder_update();
    
    float start_turns = encoder_get_position() / (float)ENCODER_SIM_CPR;
    printf("初始位置: %.4f 圈\n", start_turns);
    
    /* 启动电机 100 RPM */
    encoder_sim_set_speed(100.0);
    
    for (int i = 0; i < 1000; i++) encoder_update();
    
    float end_turns = encoder_get_position() / (float)ENCODER_SIM_CPR;
    float delta_turns = end_turns - start_turns;
    float expected_delta = 100.0 / 60.0 * 0.04;  /* 100RPM * 0.04s = 0.0667圈 */
    
    printf("增量: %.4f 圈 (期望: %.4f 圈)\n", delta_turns, expected_delta);
    
    if (fabs(delta_turns - expected_delta) < 0.005) {
        printf("✓ 测试通过\n");
        return 1;
    } else {
        printf("✗ 测试失败\n");
        return 0;
    }
}

/* 测试3: 多圈测试 */
static int test_multi_turn(void) {
    printf("\n========== 测试3: 多圈测试 (10圈) ==========\n");
    
    encoder_sim_init(2.0);
    encoder_init();
    encoder_set_crc8_check(lt_crc8_check);      /* 这一步不能少 */
    
    for (int i = 0; i < 8; i++) encoder_update();
    
    float start_turns = encoder_get_position() / (float)ENCODER_SIM_CPR;
    printf("初始圈数: %.3f\n", start_turns);
    
    encoder_sim_set_speed(300.0);
    
    int steps = (int)(10.0 / 300.0 * 60.0 * ENCODER_SIM_CALL_FREQ);
    for (int i = 0; i < steps; i++) {
        encoder_update();
        if (i % (steps/10) == 0) {
            float turns = encoder_get_position() / (float)ENCODER_SIM_CPR;
            printf("  进度 %.0f%%, 圈数: %.3f\n", (float)i/steps*100, turns);
        }
    }
    
    float final_turns = encoder_get_position() / (float)ENCODER_SIM_CPR;
    float delta = final_turns - start_turns;
    
    printf("增量: %.3f (期望: 10.000)\n", delta);
    
    if (fabs(delta - 10.0) < 0.02) {
        printf("✓ 测试通过\n");
        return 1;
    } else {
        printf("✗ 测试失败\n");
        return 0;
    }
}

/* 测试4: 正反转测试 */
static int test_encoder_bidirectional(void) {
    printf("\n========== 测试4: 正反转测试 ==========\n");
    
    encoder_sim_init(5.0);
    encoder_init();
    encoder_set_crc8_check(lt_crc8_check);      /* 这一步不能少 */
    
    for (int i = 0; i < 8; i++) encoder_update();
    
    float start_turns = encoder_get_position() / (float)ENCODER_SIM_CPR;
    printf("初始圈数: %.3f\n", start_turns);
    
    /* 正转2圈 */
    encoder_sim_set_speed(500.0);
    int steps_2turns = (int)(2.0 / 500.0 * 60.0 * ENCODER_SIM_CALL_FREQ);
    for (int i = 0; i < steps_2turns; i++) encoder_update();
    
    float after_fwd = encoder_get_position() / (float)ENCODER_SIM_CPR;
    printf("正转2圈后: %.3f (期望: %.3f)\n", after_fwd, start_turns + 2.0);
    
    /* 反转2圈 */
    encoder_sim_set_speed(-500.0);
    for (int i = 0; i < steps_2turns; i++) encoder_update();
    
    float after_rev = encoder_get_position() / (float)ENCODER_SIM_CPR;
    printf("反转2圈后: %.3f (期望: %.3f)\n", after_rev, start_turns);
    
    if (fabs(after_fwd - start_turns - 2.0) < 0.1 && fabs(after_rev - start_turns) < 0.1) {
        printf("✓ 测试通过\n");
        return 1;
    } else {
        printf("✗ 测试失败\n");
        return 0;
    }
}

/* 测试5: 多个初始位置解算 */
static int test_startup_various_positions(void) {
    printf("\n========== 测试5: 多个初始位置解算 ==========\n");
    
    double test_turns[] = {0.34, 5.5, 10.17, 15.79, 20.91};
    int num_tests = sizeof(test_turns) / sizeof(test_turns[0]);
    int passed = 0;
    
    for (int t = 0; t < num_tests; t++) {
        encoder_sim_init(test_turns[t]);
        encoder_init();
        encoder_set_crc8_check(lt_crc8_check);      /* 这一步不能少 */
        
        for (int i = 0; i < 8; i++) encoder_update();
        
        float actual = encoder_get_position() / (float)ENCODER_SIM_CPR;
        float error = actual - test_turns[t];
        
        printf("  期望: %.2f 圈 → 解算: %.3f 圈 (误差: %.4f)\n", 
               test_turns[t], actual, error);
        
        if (fabs(error) < 0.1) passed++;
    }
    
    printf("通过: %d/%d\n", passed, num_tests);
    return (passed == num_tests) ? 1 : 0;
}

/* ============ 主函数 ============ */
int encoder_test(void) {
    printf("╔══════════════════════════════════════╗\n");
    printf("║   多圈绝对值编码器仿真测试套件       ║\n");
    printf("║   编码器: %d CPR (18-bit)           ║\n", ENCODER_SIM_CPR);
    printf("║   齿轮比: 42:44                     ║\n");
    printf("║   调用频率: %.0f Hz                 ║\n", ENCODER_SIM_CALL_FREQ);
    printf("╚══════════════════════════════════════╝\n");
    
    int passed = 0;
    int total = 0;
    
    total++; passed += test_startup_position();
    total++; passed += test_position_tracking();
    total++; passed += test_multi_turn();
    total++; passed += test_encoder_bidirectional();
    total++; passed += test_startup_various_positions();
    
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
