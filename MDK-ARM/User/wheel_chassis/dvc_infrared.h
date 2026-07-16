/**
 * @file dvc_infrared.h
 * @brief 串口红外通信设备抽象类
 * @author Antigravity
 * @version 1.0
 * @date 2026-06-14
 *
 * @copyright USTC-RoboWalker
 */

#ifndef DVC_INFRARED_H
#define DVC_INFRARED_H

/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 串口红外收发设备类
 * @note 支持单字节指令以及多字节数据包的接收与解析
 */
class Class_Infrared
{
public:
    // 初始化函数，绑定底层串口管理结构体
    void Init(UART_HandleTypeDef *huart);

    // 串口接收中断/空闲中断回调接口
    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

    // 发送单个字节指令
    void Send_Byte(uint8_t Data);

    // 发送多字节数据包 (透传模式)
    void Send_Packet(uint8_t *Data, uint16_t Length);

    // 获取最新接收到的单个字节
    uint8_t Get_Last_Byte() { return Last_Rx_Byte; }

    // 检查并获取新数据标志位
    uint8_t Check_New_Data_Flag() { return New_Data_Flag; }
    void Clear_New_Data_Flag() { New_Data_Flag = 0; }

protected:
    // 绑定的底层串口管理结构体指针
    Struct_UART_Manage_Object *UART_Manage_Object;

    // 内部接收状态变量
    uint8_t Last_Rx_Byte = 0;      // 存储最新接收到的单个字节
    uint8_t New_Data_Flag = 0;     // 接收到新数据的标志位（1表示有新数据，0表示已处理）
};

#endif // DVC_INFRARED_H
