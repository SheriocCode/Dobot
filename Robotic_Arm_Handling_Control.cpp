/**
 * @file Robotic_Arm_Handling_Control.cpp
 * @brief 机械臂控制搬运实验
 *
 * 本程序用于控制机械臂完成从A点到B点的搬运任务。通过Dobot库函数实现机械臂的运动控制，同时通过控制气泵实现物体的吸附与释放。
 *
 * @author Sherioc
 * @date 2025年07月03日
 *
 * @version 1.0
 * @copyright Copyright (c) 2025 Sherioc. All rights reserved.
 *
 * @note 特别说明或注意事项：
 * SPDX-License-Identifier: MIT
 * 1. 确保机械臂的运动范围和工作环境安全，避免碰撞。
 * 2. 如1所述，可以手动调一下底座的旋转角度，这样就不会超限。
 */

#include <Magician.h>

// 定义气泵控制相关参数
#define PUMP_PIN 9 // 气泵控制引脚

// A点
#define A_POINT_X 254
#define A_POINT_Y -2
#define A_POINT_Z -40
#define A_POINT_R 0

// B点
#define B_POINT_X 170
#define B_POINT_Y -190
#define B_POINT_Z -42
#define B_POINT_R 0

int count = 1; // 搬运次数

void setup()
{
    Serial.begin(115200);
    Serial.println("Dobot 气泵控制测试");

    Dobot_Init();
    // 设置初始位置
    Dobot_SetPTPCmd(MOVJ_XYZ, 230, 0, 40, 0);
    delay(2000);

    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW); // 初始关闭气泵

    // 初始化气泵控制（使能末端执行器）
    SetEndEffectorSuctionCup(true);
    delay(500);
}

void loop()
{
    while (count > 0)
    {
        Serial.println("开始第 " + String(4 - count) + " 次搬运...");

        // 移动到A点上方
        Dobot_SetPTPCmd(MOVJ_XYZ, A_POINT_X, A_POINT_Y, 10, A_POINT_R);
        delay(1500);

        // 下降到A点
        Dobot_SetPTPCmd(MOVJ_XYZ, A_POINT_X, A_POINT_Y, A_POINT_Z, A_POINT_R);
        delay(1000);

        // 方式1：通过库函数接口启动气泵（推荐）
        // 参数说明：enableCtrl=true(使能), suck=true(吸气), isQueued=true(加入队列)
        SetEndEffectorSuctionCup(true);
        delay(1500); // 延长吸气时间确保吸附

        // 方式2：直接控制引脚（备用方案，仅当API失效时使用）
        // digitalWrite(PUMP_PIN, HIGH);
        // delay(1500);

        // 上升并移动到B点
        Dobot_SetPTPCmd(MOVJ_XYZ, A_POINT_X, A_POINT_Y, 10, A_POINT_R);
        delay(1500);
        Dobot_SetPTPCmd(MOVJ_XYZ, B_POINT_X, B_POINT_Y, 10, B_POINT_R);
        delay(1500);

        // 下降到B点
        Dobot_SetPTPCmd(MOVJ_XYZ, B_POINT_X, B_POINT_Y, B_POINT_Z, B_POINT_R);
        delay(1000);

        // 关闭气泵释放物块
        SetEndEffectorSuctionCup(false);
        // digitalWrite(PUMP_PIN, LOW);
        delay(1000);

        // 上升到安全位置
        Dobot_SetPTPCmd(MOVJ_XYZ, B_POINT_X, B_POINT_Y, 10, B_POINT_R);
        delay(1500);

        count--;
    }

    // 所有搬运完成后回到初始位置
    Dobot_SetPTPCmd(MOVJ_XYZ, 230, 0, 40, 0);
    Serial.println("所有搬运任务完成！");

    // 进入待机状态
    while (1)
    {
        delay(100);
    }
}