#ifndef SBUS_CONTROL_H
#define SBUS_CONTROL_H

#include "sbus.h" // 包含 SBUS 库
// --- 移除：#include "usart.h"

// 定义控制范围的最大值
#define CTRL_RANGE_MAX 100.0f

// 定义命令类型枚举
typedef enum {
    CMD_STOP = 0,
    CMD_MOVE = 1,
    // 可以添加更多命令类型
} CommandType_t;

// 定义处理后的SBUS值结构体
typedef struct {
    float Vx;           // 纵向速度 (前后)
    float Vy;           // 横向速度 (左右)
    float W;            // 角速度 (旋转)
    float LiftPercent;  // CH3 升降台速度指令 (-100.0f ~ 100.0f)
    int CmdEnable;      // 命令使能
    float Aux1;         // 辅助通道1
    float Aux2;         // 辅助通道2
    CommandType_t CmdType; // 命令类型

    uint8_t RobotMode;  // 当前保持 0：底盘与升降/夹爪机构模式
    float Arm_Axis1;    // 备用达妙 1
    float Arm_Axis2;    // 备用达妙 2
    float Arm_Axis3;    // CH9 龙门架平动两点：0=上电伸出固定点，100=反转 3 圈点
    float Arm_Axis4;    // CH5 吸夹翻转开关：0/100
    float Arm_Axis5;    // CH11 吸夹翻转微调速度指令：中位保持，增大正调，减小反调
    uint16_t Raw_CH7;   // CH7 原始值，用于夹爪总成状态机
    // 可以根据需要添加更多字段
} ProcessedSbusValue_t;

// 全局变量声明
extern ProcessedSbusValue_t G_SbusValue;

// 函数声明
void SbusControl_Init(void);
void SbusControl_ProcessData(void);

// --- 移除：用于初始化串口句柄的函数声明 ---
// void SbusControl_SetDebugUartHandle(UART_HandleTypeDef *huart);

#endif // SBUS_CONTROL_H
