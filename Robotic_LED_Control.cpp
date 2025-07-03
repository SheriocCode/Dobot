/**
 * @file Robotic_LED_Control.cpp
 * @brief LED灯控制实验
 *
 * 本程序用于实现摇杆模块和 RGB 按键模块控制 Dobot Magician 机械臂移动及气泵启停。
 *
 * 阶段一（deprecated，见最底部代码):
    实现文档要求的单独点灯等功能

 * 阶段二（new）：
 * 优化实现 LED 闪烁、三色 LED 轮流点亮，然后同时启用遥感和按钮控制 LED 的功能。
    使用currentPhase变量跟踪当前阶段，通过millis()函数实现 3 秒定时切换。
    阶段顺序：LED 闪烁→三色 LED→遥感 + 按钮控制。
    遥感控制逻辑
    红灯实现半轴控制：
    右半侧（>512+30）：数值越大越亮；
    左半侧（<512-30）：数值越小越亮。
    蓝灯实现全轴控制，Y 轴数值映射到 0-255 亮度。
    按钮与遥感优先级
    按下按钮时，对应 LED 强制点亮（HIGH），优先级高于遥感控制；
    松开按钮时，LED 亮度由遥感控制。
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
#define Red_LED A3   // 红色LED→A3
#define Green_LED A1 // 绿色LED→A1
#define Blue_LED 9   // 蓝色LED→9

#define Red_Button A0   // 红色按钮→A0
#define Green_Button A2 // 绿色按钮→A2
#define Blue_Button A4  // 蓝色按钮→A4

#define Joystick_X A7 // 遥感X轴→A7
#define Joystick_Y A6 // 遥感Y轴→A6

// 阶段状态定义
#define PHASE_LED_FLICKER 0 // LED闪烁阶段
#define PHASE_THREE_COLOR 1 // 三色LED阶段
#define PHASE_JOY_BUTTON 2  // 遥感+按钮阶段

int currentPhase = PHASE_LED_FLICKER;      // 当前阶段
unsigned long phaseStartTime = 0;          // 阶段开始时间
const unsigned long PHASE_DURATION = 3000; // 每阶段持续3000ms（3秒）

void setup()
{
    Serial.begin(9600);

    // 设置LED引脚为输出
    pinMode(Red_LED, OUTPUT);
    pinMode(Green_LED, OUTPUT);
    pinMode(Blue_LED, OUTPUT);

    // 设置按钮引脚为输入上拉
    pinMode(Red_Button, INPUT_PULLUP);
    pinMode(Green_Button, INPUT_PULLUP);
    pinMode(Blue_Button, INPUT_PULLUP);

    // 初始化阶段时间
    phaseStartTime = millis();
    Serial.println("系统启动，进入LED闪烁阶段...");
}

void loop()
{
    // 阶段切换逻辑
    if (millis() - phaseStartTime >= PHASE_DURATION)
    {
        switch (currentPhase)
        {
        case PHASE_LED_FLICKER:
            currentPhase = PHASE_THREE_COLOR;
            Serial.println("进入三色LED轮流点亮阶段...");
            break;
        case PHASE_THREE_COLOR:
            currentPhase = PHASE_JOY_BUTTON;
            Serial.println("进入遥感+按钮控制阶段...");
            break;
        case PHASE_JOY_BUTTON:
            // 保持在第三阶段，不再切换
            break;
        }
        phaseStartTime = millis(); // 重置阶段开始时间
    }

    // 执行当前阶段功能
    switch (currentPhase)
    {
    case PHASE_LED_FLICKER:
        ledFlickerPhase();
        break;
    case PHASE_THREE_COLOR:
        threeColorLEDPhase();
        break;
    case PHASE_JOY_BUTTON:
        joystickAndButtonControl();
        break;
    }
}

// LED闪烁阶段
void ledFlickerPhase()
{
    digitalWrite(Blue_LED, HIGH); // 点亮LED
    delay(500);
    digitalWrite(Blue_LED, LOW); // 熄灭LED
    delay(500);
}

// 三色LED轮流点亮阶段
void threeColorLEDPhase()
{
    // 红色LED点亮
    digitalWrite(Red_LED, HIGH);
    digitalWrite(Green_LED, LOW);
    digitalWrite(Blue_LED, LOW);
    delay(1000);

    // 绿色LED点亮
    digitalWrite(Red_LED, LOW);
    digitalWrite(Green_LED, HIGH);
    digitalWrite(Blue_LED, LOW);
    delay(1000);

    // 蓝色LED点亮
    digitalWrite(Red_LED, LOW);
    digitalWrite(Green_LED, LOW);
    digitalWrite(Blue_LED, HIGH);
    delay(1000);
}

// 遥感+按钮控制阶段
void joystickAndButtonControl()
{
    // 1. 遥感控制逻辑
    int joyX = analogRead(Joystick_X);
    int joyY = analogRead(Joystick_Y);

    // 处理红灯半轴控制（0-90或90-180）
    int redBrightness = 0;
    if (joyX > 512 + 30)
    { // 右半侧（>512+30）
        redBrightness = map(joyX, 512 + 30, 1023, 0, 255);
    }
    else if (joyX < 512 - 30)
    { // 左半侧（<512-30）
        redBrightness = map(joyX, 0, 512 - 30, 255, 0);
    }

    // 处理蓝灯0-180亮度（映射到0-255）
    int blueBrightness = map(joyY, 0, 1023, 0, 255);

    // 2. 按钮控制逻辑
    bool redBtn = !digitalRead(Red_Button);
    bool greenBtn = !digitalRead(Green_Button);
    bool blueBtn = !digitalRead(Blue_Button);

    // 3. 合并控制（按钮优先级高于遥感）
    if (redBtn)
    {
        digitalWrite(Red_LED, HIGH);
    }
    else
    {
        analogWrite(Red_LED, redBrightness);
    }

    if (greenBtn)
    {
        digitalWrite(Green_LED, HIGH);
    }
    else
    {
        digitalWrite(Green_LED, LOW); // 绿色LED仅由按钮控制
    }

    if (blueBtn)
    {
        digitalWrite(Blue_LED, HIGH);
    }
    else
    {
        analogWrite(Blue_LED, blueBrightness);
    }

    // 串口调试输出
    Serial.print("Phase: JOY+BTN | ");
    Serial.print("Red: ");
    Serial.print(redBrightness);
    Serial.print(" | Blue: ");
    Serial.print(blueBrightness);
    Serial.print(" | Buttons: R=");
    Serial.print(redBtn);
    Serial.print(" G=");
    Serial.print(greenBtn);
    Serial.print(" B=");
    Serial.println(blueBtn);
    delay(100);
}

/** 阶段一单独函数，Deprecated
// LED 闪烁功能
void ledFlicker()
{
    int ledPin = 9; // 使用板载LED引脚13
    pinMode(ledPin, OUTPUT);
    while (true)
    {
        digitalWrite(ledPin, HIGH);
        delay(500);
        digitalWrite(ledPin, LOW);
        delay(500);
    }
}

// 三色LED轮流点亮功能
void threeColorLED()
{
    pinMode(Red_LED, OUTPUT);
    pinMode(Green_LED, OUTPUT);
    pinMode(Blue_LED, OUTPUT);
    while (true)
    {
        digitalWrite(Red_LED, HIGH);
        digitalWrite(Green_LED, LOW);
        digitalWrite(Blue_LED, LOW);
        delay(1000);

        digitalWrite(Red_LED, LOW);
        digitalWrite(Green_LED, HIGH);
        digitalWrite(Blue_LED, LOW);
        delay(1000);

        digitalWrite(Red_LED, LOW);
        digitalWrite(Green_LED, LOW);
        digitalWrite(Blue_LED, HIGH);
        delay(1000);
    }
}

// 遥感控制LED灯功能
void joystickControlLED()
{
    pinMode(Blue_LED, OUTPUT); // 使用蓝色LED作为指示
    while (true)
    {
        int joyValue = analogRead(Joystick_X);
        int ledBrightness = map(joyValue, 0, 1023, 0, 255);
        analogWrite(Blue_LED, ledBrightness);

        Serial.print("Joystick Value: ");
        Serial.print(joyValue);
        Serial.print(" | LED Brightness: ");
        Serial.println(ledBrightness);

        delay(100);
    }
}

// 遥感控制双LED灯功能（蓝灯和红灯）
void joystickControlTwoLEDs()
{
// 定义遥感死区阈值（文档未明确，此处设为30）
#define DEADZONE 30
// 定义遥感中点值（文档）
#define JOY_X_MID 512
#define JOY_Y_MID 508
    pinMode(Red_LED, OUTPUT);
    pinMode(Blue_LED, OUTPUT);

    while (true)
    {
        int joyX = analogRead(Joystick_X);
        int joyY = analogRead(Joystick_Y);

        // 处理红灯X轴响应（添加死区逻辑）
        int redBrightness = 0;
        if (joyX > JOY_X_MID + DEADZONE)
        {
            // X轴右半侧（>512+30）映射到0-255
            redBrightness = map(joyX, JOY_X_MID + DEADZONE, 1023, 0, 255);
        }
        else if (joyX < JOY_X_MID - DEADZONE)
        {
            // X轴左半侧（<512-30）映射到0-255（反向）
            redBrightness = map(joyX, 0, JOY_X_MID - DEADZONE, 255, 0);
        }

        // 处理蓝灯Y轴响应（以中点为基准）
        int blueBrightness = map(joyY, 0, 1023, 0, 255);

        // 输出PWM
        analogWrite(Red_LED, redBrightness);
        analogWrite(Blue_LED, blueBrightness);

        // 串口调试
        Serial.print("X:");
        Serial.print(joyX);
        Serial.print(" Y:");
        Serial.print(joyY);
        Serial.print(" R:");
        Serial.print(redBrightness);
        Serial.print(" B:");
        Serial.println(blueBrightness);
        delay(100);
    }
}

// 按钮控制LED指示灯功能
// 按钮控制LED指示灯功能（修正逻辑反转）
void buttonControlLED()
{

    // 设置引脚模式
    pinMode(Red_LED, OUTPUT);
    pinMode(Green_LED, OUTPUT);
    pinMode(Blue_LED, OUTPUT);

    pinMode(Red_Button, INPUT_PULLUP); // 启用内部上拉电阻
    pinMode(Green_Button, INPUT_PULLUP);
    pinMode(Blue_Button, INPUT_PULLUP);

    while (true)
    {
        // 读取按钮状态并反转（按下时为LOW，反转后为HIGH）
        bool redBtn = digitalRead(Red_Button);
        bool greenBtn = digitalRead(Green_Button);
        bool blueBtn = digitalRead(Blue_Button);

        // 按下按钮时点亮LED，松开时熄灭
        digitalWrite(Red_LED, redBtn);
        digitalWrite(Green_LED, greenBtn);
        digitalWrite(Blue_LED, blueBtn);

        // 串口打印状态（调试用）
        Serial.print("Red Button: ");
        Serial.print(redBtn ? "ON" : "OFF");
        Serial.print(" | Green Button: ");
        Serial.print(greenBtn ? "ON" : "OFF");
        Serial.print(" | Blue Button: ");
        Serial.println(blueBtn ? "ON" : "OFF");

        delay(100); // 防抖动延时
    }
}
 */