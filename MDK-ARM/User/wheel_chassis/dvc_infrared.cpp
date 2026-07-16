/**
 * @file dvc_infrared.cpp
 * @brief 串口红外通信设备抽象类实现
 * @author Antigravity
 * @version 1.0
 * @date 2026-06-14
 *
 * @copyright USTC-RoboWalker
 */

/* Includes ------------------------------------------------------------------*/
#include "dvc_infrared.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化红外串口通信设备
 * @param huart 绑定的 HAL 库串口句柄指针 (例如 &huart3)
 */
void Class_Infrared::Init(UART_HandleTypeDef *huart)
{
    // 匹配底层通用串口管理对象
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART7)
    {
        UART_Manage_Object = &UART7_Manage_Object;
    }
    // 可根据实际启用的串口在此处添加匹配逻辑
}

/**
 * @brief 串口数据接收完成/空闲中断回调解析逻辑
 * @param Rx_Data 接收到的数据缓冲区指针
 * @param Length 接收到的数据长度
 */
void Class_Infrared::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    if (Length > 0)
    {
        // 记录最新接收到的最后一个字节作为当前键值/命令
        Last_Rx_Byte = Rx_Data[Length - 1];
        New_Data_Flag = 1;
    }
}

/**
 * @brief 发送单个字节红外指令 (透传)
 * @param Data 发送的单字节数据
 */
void Class_Infrared::Send_Byte(uint8_t Data)
{
    if (UART_Manage_Object != nullptr && UART_Manage_Object->UART_Handler != nullptr)
    {
        UART_Send_Data(UART_Manage_Object->UART_Handler, &Data, 1);
    }
}

/**
 * @brief 发送多个字节红外数据包 (透传)
 * @param Data 数据缓冲区指针
 * @param Length 数据长度
 */
void Class_Infrared::Send_Packet(uint8_t *Data, uint16_t Length)
{
    if (UART_Manage_Object != nullptr && UART_Manage_Object->UART_Handler != nullptr && Data != nullptr && Length > 0)
    {
        UART_Send_Data(UART_Manage_Object->UART_Handler, Data, Length);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
