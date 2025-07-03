/**
 * @file Robotic_Arm_Joystick_Control.cpp
 * @brief 摇杆控制机械臂搬运实验
 *
 * 本程序用于实现摇杆模块和 RGB 按键模块控制 Dobot Magician 机械臂移动及气泵启停。
 *
 * @author Sherioc
 * @date 2025年07月03日
 *
 * @version 1.0
 * @copyright Copyright (c) 2025 Sherioc. All rights reserved.
 *
 * @note 特别说明或注意事项：
 * SPDX-License-Identifier: MIT
 */

#include <Dobot.h>

// 引脚定义
#define JoyStick_X A7 // 摇杆X轴模拟输入
#define JoyStick_Y A6 // 摇杆Y轴模拟输入
#define JoyStick_Z A5 // 摇杆Z轴按钮输入

#define BUTTON_RED A0   // 红色按键（控制机械臂向下）
#define BUTTON_GREEN A2 // 绿色按键（控制机械臂向上）
#define BUTTON_BLUE A4  // 蓝色按键（控制气泵）
#define PUMP_PIN 9      // 气泵控制引脚

// 机械臂运动参数
#define MOVEMENT_SPEED 5 // 基础移动速度
#define MAX_SPEED 20     // 最大移动速度
#define MIN_SPEED 1      // 最小移动速度
#define Z_STEP 5         // Z轴每次移动距离(mm)

// 摇杆阈值定义
#define X_THRESHOLD_LOW 250  // X轴低阈值
#define X_THRESHOLD_HIGH 530 // X轴高阈值
#define Y_THRESHOLD_LOW 240  // Y轴低阈值
#define Y_THRESHOLD_HIGH 540 // Y轴高阈值

// 状态变量
int currentStep = MOVEMENT_SPEED; // 当前步长（决定速度）
bool lastBlueButtonState = HIGH;  // 上拉电阻初始HIGH
bool pumpState = false;           // 气泵状态（默认关闭）
bool isMoving = false;            // 机械臂移动状态

void setup()
{
    Serial.begin(115200);
    Serial.println("摇杆控制机械臂实验 - 初始化中...");

    // 初始化Dobot机械臂
    Dobot_Init();
    delay(1000);

    // 初始化引脚
    pinMode(JoyStick_Z, INPUT_PULLUP);
    pinMode(BUTTON_RED, INPUT_PULLUP);
    pinMode(BUTTON_GREEN, INPUT_PULLUP);
    pinMode(BUTTON_BLUE, INPUT_PULLUP);
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW); // 初始关闭气泵

    Serial.println("初始化完成，开始监听控制输入...");
}

void loop()
{
    // 1. 读取摇杆输入
    int joyX = analogRead(JoyStick_X);
    int joyY = analogRead(JoyStick_Y);
    int joyZ = digitalRead(JoyStick_Z); // 摇杆按钮（Z轴）

    bool redBtnActive = (digitalRead(BUTTON_RED) == HIGH);     // 低电平有效（按下时为true）
    bool greenBtnActive = (digitalRead(BUTTON_GREEN) == HIGH); // 低电平有效（按下时为true）
    bool blueBtnActive = (digitalRead(BUTTON_BLUE) == HIGH);   // 低电平有效（按下时为true）

    bool joyZPressed = (digitalRead(JoyStick_Z) == HIGH);

    // === 摇杆Z轴控制速度（通过改变步长）===
    if (joyZPressed)
    {
        currentStep = MAX_SPEED; // 高速模式（大步长）
        Serial.println("高速模式激活");
    }
    else
    {
        currentStep = MOVEMENT_SPEED; // 正常速度（正常步长）
    }

    // === X/Y方向控制 ===
    if (joyX < X_THRESHOLD_LOW)
    {
        Dobot_SetPTPCmd(MOVJ_XYZ_INC, 0, currentStep, 0, 0); // 向左移动
    }
    else if (joyX > X_THRESHOLD_HIGH)
    {
        Dobot_SetPTPCmd(MOVJ_XYZ_INC, 0, -currentStep, 0, 0); // 向右移动
    }

    if (joyY < Y_THRESHOLD_LOW)
    {
        Dobot_SetPTPCmd(MOVJ_XYZ_INC, currentStep, 0, 0, 0); // 向前移动
    }
    else if (joyY > Y_THRESHOLD_HIGH)
    {
        Dobot_SetPTPCmd(MOVJ_XYZ_INC, -currentStep, 0, 0, 0); // 向后移动
    }

    // 红色按键，上移动
    if (redBtnActive)
    {
        Dobot_SetPTPCmd(MOVJ_XYZ_INC, 0, 0, -currentStep, 0); // 下降
    }
    // 绿色按键，下移动
    else if (greenBtnActive)
    {
        Dobot_SetPTPCmd(MOVJ_XYZ_INC, 0, 0, currentStep, 0); // 上升
    }

    // === 气泵控制===
    if (blueBtnActive && !lastBlueButtonState)
    {
        pumpState = !pumpState;
        Dobot_SetEndEffectorSuctionCup(pumpState);

        Serial.print("气泵状态: ");
        Serial.println(pumpState ? "开启" : "关闭");

        // 添加延时防止抖动
        delay(100);
    }

    lastBlueButtonState = blueBtnActive;
    delay(50); // 控制循环频率
}