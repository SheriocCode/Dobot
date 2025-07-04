/**
 * @file Robotic_VoiceRecognition.cpp
 * @brief 语音识别实验
 *
 * 本程序基于 LD3320 实现语音识别模块。
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
#include <Magician.h>
#include <VoiceRecognition.h>

// 创建语音识别对象
VoiceRecognition voice;

// 定义语音指令ID
#define CMD_UP 0       // 上升
#define CMD_DOWN 1     // 下降
#define CMD_LEFT 2     // 左转
#define CMD_RIGHT 3    // 右转
#define CMD_FORWARD 4  // 向前
#define CMD_BACKWARD 5 // 向后
#define CMD_GRAB 6     // 抓住
#define CMD_RELEASE 7  // 松开

// 运动参数
#define MOVE_STEP 50 // 每次移动步长(mm)

// 气泵状态
bool pumpState;

void setup()
{
    Serial.begin(115200);

    // 初始化机械臂
    Dobot_Init();
    Dobot_SetPTPCmd(MOVJ_XYZ, 230, 0, 40, 0); // 设置初始位置

    // 初始化气泵为关闭状态
    Dobot_SetEndEffectorSuctionCup(0);

    // 初始化语音识别模块
    voice.init();

    // 添加语音指令
    voice.addCommand("shang shen", CMD_UP);    // 上升
    voice.addCommand("xia jiang", CMD_DOWN);   // 下降
    voice.addCommand("zuo zhuan", CMD_LEFT);   // 左转
    voice.addCommand("you zhuan", CMD_RIGHT);  // 右转
    voice.addCommand("qian jin", CMD_FORWARD); // 向前
    voice.addCommand("hou tui", CMD_BACKWARD); // 向后
    voice.addCommand("kai qi", CMD_GRAB);      // 开启
    voice.addCommand("guan bi", CMD_RELEASE);  // 关闭

    voice.start(); // 开始语音识别
    Serial.println("语音识别系统已启动");
}

void loop()
{
    switch (voice.read())
    {            // 读取语音指令
    case CMD_UP: // 上升
        Dobot_SetPTPCmd(MOVJ_XYZ_INC, 0, 0, MOVE_STEP, 0);
        Serial.println("指令: 上升");
        break;

    case CMD_DOWN: // 下降
        Dobot_SetPTPCmd(MOVJ_XYZ_INC, 0, 0, -MOVE_STEP, 0);
        Serial.println("指令: 下降");
        break;

    case CMD_LEFT: // 左转
        Dobot_SetPTPCmd(MOVJ_XYZ_INC, 0, MOVE_STEP, 0, 0);
        Serial.println("指令: 左转");
        break;

    case CMD_RIGHT: // 右转
        Dobot_SetPTPCmd(MOVJ_XYZ_INC, 0, -MOVE_STEP, 0, 0);
        Serial.println("指令: 右转");
        break;

    case CMD_FORWARD: // 向前
        Dobot_SetPTPCmd(MOVJ_XYZ_INC, MOVE_STEP, 0, 0, 0);
        Serial.println("指令: 向前");
        break;

    case CMD_BACKWARD: // 向后
        Dobot_SetPTPCmd(MOVJ_XYZ_INC, -MOVE_STEP, 0, 0, 0);
        Serial.println("指令: 向后");
        break;

    case CMD_GRAB: // 开启气泵
        pumpState = true;
        Dobot_SetEndEffectorSuctionCup(pumpState);
        Serial.println("指令: 气泵开启");
        break;

    case CMD_RELEASE: // 关闭气泵
        pumpState = false;
        Dobot_SetEndEffectorSuctionCup(pumpState);
        Serial.println("指令: 气泵关闭");
        break;

    default:
        break;
    }

    delay(100); // 适当延迟
}
