/**
 * @file Robot_Arm_Vision_Sorting
 * @brief 机械臂视觉分拣实验
 *
 * 本程序基于 Pixy2 视觉识别模块和 LD3320 语音识别模块结合分拣不同颜色的物块。
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
#include <VoiceRecognition.h>
#include <Magician.h>
#include "Pixy2I2C.h"

// --------------------- 坐标定义区域 ---------------------
/** 红色方块目标放置坐标(X,Y,Z,旋转角度) */
#define Red_position_X 200.0145 // 红色方块的目标X坐标
#define Red_position_Y 110.3269 // 红色方块的目标Y坐标
#define Red_position_Z -29.9800 // 红色方块的目标Z坐标
#define Red_position_R -90      // 红色方块的目标旋转角度

/** 绿色方块目标放置坐标(X,Y,Z,旋转角度) */
#define Green_position_X 199.2351 // 绿色方块的目标X坐标
#define Green_position_Y 109.6504 // 绿色方块的目标Y坐标
#define Green_position_Z -6.1981  // 绿色方块的目标Z坐标
#define Green_position_R -90      // 绿色方块的目标旋转角度

/** 蓝色方块目标放置坐标(X,Y,Z,旋转角度) */
#define Blue_position_X 197.8510 // 蓝色方块的目标X坐标
#define Blue_position_Y 109.0078 // 蓝色方块的目标Y坐标
#define Blue_position_Z 17.7011  // 蓝色方块的目标Z坐标
#define Blue_position_R -90      // 蓝色方块的目标旋转角度

/** 黄色方块目标放置坐标(X,Y,Z,旋转角度) */
#define Yellow_position_X 198.6485 // 黄色方块的目标X坐标
#define Yellow_position_Y 106.1209 // 黄色方块的目标Y坐标
#define Yellow_position_Z 42.1484  // 黄色方块的目标Z坐标
#define Yellow_position_R -90      // 黄色方块的目标旋转角度

// --------------------- 视觉参数定义 ---------------------
/** Pixy摄像头识别颜色签名定义 */
#define Signature_Red 4    // 红色物体签名
#define Signature_Green 2  // 绿色物体签名
#define Signature_Blue 3   // 蓝色物体签名
#define Signature_Yellow 1 // 黄色物体签名

/** 方块最小识别尺寸(过滤小物体干扰) */
#define Block_Width 30  // 最小宽度阈值
#define Block_Height 30 // 最小高度阈值

// --------------------- 硬件接口定义 ---------------------
/** LED与按钮引脚定义 */
#define Red_LED A1    // 红色状态指示灯
#define Green_LED A14 // 绿色状态指示灯
#define Blue_LED A3   // 蓝色状态指示灯
#define Button1 A4    // 触发按钮

// 创建Pixy摄像头和语音识别对象
Pixy2I2C pixy;          // Pixy摄像头对象
VoiceRecognition Voice; // 语音识别对象

// 全局变量声明
uint16_t blocks, flag;
uint16_t i;
int gRed = 0, gGreen = 0, gBlue = 0, gYellow = 0; // 各颜色方块计数器

// 机械臂坐标系中的三个参考点坐标
float dobotPoint[9] = {
    232.39, 273, 228.4,
    0.05, -21.7, -43.9,
    1, 1, 1};

// Pixy摄像头坐标系中的三个参考点坐标
float pixyPoint[9] = {
    225, 181, 114,
    45, 147.5, 47,
    1, 1, 1};

// 坐标变换矩阵（用于将摄像头坐标转换为机械臂坐标）
float RT[9] = {0};

// 存储转换后的目标坐标
float Coordinate[2] = {0}; // 目标方块的X和Y坐标

// 定义结构体统一管理颜色参数
typedef struct
{
    uint8_t signature;
    float posX, posY, posZ, posR;
    int *counter;
    const char *colorName;
} BlockParam;

/**
 * @brief 计算3x3矩阵的逆矩阵
 * @param Mat 输入的3x3矩阵（长度为9的数组）
 * @param InvMat 输出的逆矩阵（长度为9的数组）
 * @note 用于坐标变换
 */
void CalcInvMat(float *Mat, float *InvMat)
{
    int i = 0;
    double Det = 0.0;
    // 计算3x3矩阵的行列式
    Det = Mat[0] * (Mat[4] * Mat[8] - Mat[5] * Mat[7]) -
          Mat[3] * (Mat[1] * Mat[8] - Mat[2] * Mat[7]) +
          Mat[6] * (Mat[1] * Mat[5] - Mat[2] * Mat[4]);

    // 计算伴随矩阵
    InvMat[0] = Mat[4] * Mat[8] - Mat[5] * Mat[7];
    InvMat[1] = Mat[2] * Mat[7] - Mat[1] * Mat[8];
    InvMat[2] = Mat[1] * Mat[5] - Mat[2] * Mat[4];
    InvMat[3] = Mat[5] * Mat[6] - Mat[3] * Mat[8];
    InvMat[4] = Mat[0] * Mat[8] - Mat[2] * Mat[6];
    InvMat[5] = Mat[3] * Mat[2] - Mat[0] * Mat[5];
    InvMat[6] = Mat[3] * Mat[7] - Mat[4] * Mat[6];
    InvMat[7] = Mat[1] * Mat[6] - Mat[7] * Mat[0];
    InvMat[8] = Mat[0] * Mat[4] - Mat[3] * Mat[1];

    // 伴随矩阵除以行列式得到逆矩阵
    for (i = 0; i < 9; i++)
    {
        InvMat[i] = InvMat[i] / Det;
    }
}

/**
 * @brief 3x3矩阵乘法
 * @param Mat1 第一个3x3矩阵（长度为9的数组）
 * @param Mat2 第二个3x3矩阵（长度为9的数组）
 * @param Result 结果矩阵（长度为9的数组）
 * @note 用于坐标变换
 */
void MatMultiMat(float *Mat1, float *Mat2, float *Result)
{
    // 初始化结果矩阵
    for (int i = 0; i < 9; i++)
    {
        Result[i] = 0;
    }

    // 3x3矩阵乘法
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                Result[i * 3 + j] += Mat1[i * 3 + k] * Mat2[k * 3 + j];
            }
        }
    }
}

/**
 * @brief 坐标变换函数，将摄像头坐标转换为机械臂坐标
 * @param x 摄像头坐标系下的X坐标
 * @param y 摄像头坐标系下的Y坐标
 * @note 结果存储在全局变量Coordinate中
 */
void transForm(float x, float y)
{
    Coordinate[0] = (RT[0] * x) + (RT[1] * y) + (RT[2] * 1); // 转换后的X坐标
    Coordinate[1] = (RT[3] * x) + (RT[4] * y) + (RT[5] * 1); // 转换后的Y坐标
}

/**
 * @brief 输出当前检测到的所有方块信息
 * @param cycle 检测轮次编号
 */
void printBlocksInfo(int cycle)
{
    Serial.print("Cycle ");
    Serial.print(cycle);
    Serial.print(" - Detected Blocks: ");
    Serial.println(pixy.ccc.numBlocks);
    for (uint16_t k = 0; k < pixy.ccc.numBlocks; k++)
    {
        Serial.print("Block ");
        Serial.print(k + 1);
        Serial.print(" - Signature: ");
        Serial.print(pixy.ccc.blocks[k].m_signature);
        Serial.print(", X: ");
        Serial.print(pixy.ccc.blocks[k].m_x);
        Serial.print(", Y: ");
        Serial.print(pixy.ccc.blocks[k].m_y);
        Serial.print(", Width: ");
        Serial.print(pixy.ccc.blocks[k].m_width);
        Serial.print(", Height: ");
        Serial.print(pixy.ccc.blocks[k].m_height);
        Serial.print(", Area: ");
        Serial.println(pixy.ccc.blocks[k].m_width * pixy.ccc.blocks[k].m_height);
    }
    Serial.println("-----------------------------");
}

/**
 * @brief Arduino初始化函数，完成各模块初始化和参数设置
 * @note 包括串口、引脚、语音识别、机械臂、Pixy摄像头等
 */
void setup()
{
    Serial.begin(115200);

    // 初始化各引脚模式
    pinMode(Red_LED, OUTPUT);   // 红色LED设为输出
    pinMode(Green_LED, OUTPUT); // 绿色LED设为输出
    pinMode(Blue_LED, OUTPUT);  // 蓝色LED设为输出
    pinMode(Button1, INPUT);    // 按钮设为输入

    // 初始化各模块
    Voice.init(); // 初始化语音识别

    Dobot_Init(); // 初始化机械臂
    Serial.print("Dobot init");

    pixy.init(); // 初始化Pixy摄像头
    Serial.print("pixy init");
    if (pixy.init() != 0)
    {
        Serial.println("Pixy初始化失败");
        while (1)
            ; // 卡死在这里便于排查
    }

    // 设置机械臂参数
    Dobot_SetPTPJumpParams(20); // 设置机械臂抬升最大高度

    // 计算坐标变换矩阵
    float inv_pixy[9] = {0};
    CalcInvMat(pixyPoint, inv_pixy);       // 计算Pixy坐标系的逆矩阵
    MatMultiMat(dobotPoint, inv_pixy, RT); // 计算变换矩阵RT

    Dobot_SetPTPCommonParams(100, 100); // 设置机械臂运动参数

    // 添加语音命令
    Voice.addCommand("zhua qu hong se", 1); // 红色命令
    Voice.addCommand("hong se", 1);
    Voice.addCommand("zhua qu lv se", 2); // 绿色命令
    Voice.addCommand("lv se", 2);
    Voice.addCommand("zhua qu lan se", 3); // 蓝色命令
    Voice.addCommand("lan se", 3);
    Voice.addCommand("zhua qu huang se", 4); // 黄色命令
    Voice.addCommand("huang se", 4);

    Voice.start(); // 开始语音识别
}

// 封装抓取流程
void handleBlock(const BlockParam &param)
{
    uint16_t i = 0;
    Serial.print("handle ");
    Serial.print(param.colorName);
    Serial.println(" block");
    Serial.println("--- Pixy Camera Detection Information ---");
    // 连续获取三次方块数据，并输出详细信息
    for (int j = 0; j < 3; j++)
    {
        pixy.ccc.getBlocks();
        printBlocksInfo(j + 1);
        delay(10);
    }
    pixy.ccc.getBlocks();
    uint16_t blocks = pixy.ccc.numBlocks;
    while (i < blocks && pixy.ccc.blocks[i].m_signature != param.signature)
    {
        i++;
    }
    if (i >= blocks)
    {
        Serial.print("未找到");
        Serial.print(param.colorName);
        Serial.println("方块");
        return;
    }
    transForm(pixy.ccc.blocks[i].m_x, pixy.ccc.blocks[i].m_y);
    Dobot_SetPTPCmd(JUMP_XYZ, Coordinate[0], Coordinate[1], -45, -90);
    Dobot_SetEndEffectorSuctionCup(true);
    Dobot_SetPTPCmd(MOVL_XYZ, Coordinate[0], Coordinate[1], 50, -90);
    Dobot_SetPTPCmd(JUMP_XYZ, param.posX, param.posY, param.posZ, param.posR);
    Dobot_SetEndEffectorSuctionCup(false);
    Dobot_SetPTPCmd(JUMP_XYZ, param.posX, param.posY, 100, -90);
    (*(param.counter))++;
    Serial.print("已抓取");
    Serial.print(param.colorName);
    Serial.print("方块数量: ");
    Serial.println(*(param.counter));
}

/**
 * @brief 主循环函数，处理按钮触发、语音识别和分拣流程
 * @note 按下按钮后，根据语音命令分拣不同颜色方块
 */
void loop()
{
    // 移动机械臂到摄像头位置
    Dobot_SetPTPCmd(MOVJ_XYZ, 235, -26, 50, -90);

    // 检测按钮是否按下
    int val = digitalRead(Button1);
    if (val == HIGH)
    {
        // 按下按钮时点亮所有LED
        digitalWrite(Red_LED, HIGH);
        digitalWrite(Green_LED, HIGH);
        digitalWrite(Blue_LED, HIGH);

        //        // 等待按钮释放
        while (val)
        {
            val = digitalRead(Button1);
            delay(500);
        }

        // 关闭所有LED
        digitalWrite(Red_LED, 0);
        digitalWrite(Green_LED, 0);
        digitalWrite(Blue_LED, 0);

        // 获取语音命令
        /**
         * @brief 读取语音识别结果，返回命令编号
         * @return int 命令编号（1-红色，2-绿色，3-蓝色，4-黄色）
         */
        int command = Voice.read();

        // 根据命令执行相应操作
        BlockParam blockParams[] = {
            {Signature_Red, Red_position_X, Red_position_Y, Red_position_Z, Red_position_R, &gRed, "红色"},
            {Signature_Green, Green_position_X, Green_position_Y, Green_position_Z, Green_position_R, &gGreen, "绿色"},
            {Signature_Blue, Blue_position_X, Blue_position_Y, Blue_position_Z, Blue_position_R, &gBlue, "蓝色"},
            {Signature_Yellow, Yellow_position_X, Yellow_position_Y, Yellow_position_Z, Yellow_position_R, &gYellow, "黄色"}};
        if (command >= 1 && command <= 4)
        {
            handleBlock(blockParams[command - 1]);
        }
        else
        {
            Serial.println("未知命令");
        }
    }
}
