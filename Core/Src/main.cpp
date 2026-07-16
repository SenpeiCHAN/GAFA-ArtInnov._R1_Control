/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_bsp.h"
#include "drv_math.h"
#include "dvc_serialplot.h"
#include "dvc_motor.h"


#include "sbus.h"
#include "sbus_control.h"
#include <stdio.h>
#include <string.h>

#include "dvc_motor_dm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 底盘与机构参数
#define CHASSIS_RADIUS      0.2f        // 底盘半径 (m)
#define FRONT_SCALE         1.0f        // 前轮补偿系数
#define BACK_SCALE          1.0f        // 后轮补偿系数
#define GEAR_RATIO          19.0f       // 电机减速比
#define MAX_WHEEL_SPEED     25.0f       // 最大轮速 (rad/s)
#define CHASSIS_CONTROL_DT  0.002f      // 底盘控制周期: 2ms
#define LIFT_MAX_ANGLE (10.0f * PI)     // 升降台最大行程。10*PI 表示电机减速后的输出轴转 5 圈
#define SBUS_FRAME_SIZE 25              // SBUS 通信定义
// HWT101 陀螺仪串口协议参数
#define HWT101_FRAME_SIZE 11U
#define HWT101_REG_RSW 0x02U
#define HWT101_REG_RRATE 0x03U
#define HWT101_OUTPUT_ANGLE_ONLY 0x0008U
#define HWT101_RRATE_100HZ 0x0009U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#if 0
// SBUS 相关变量
uint8_t rx_data = 0;                         // 用于接收单个字节的临时变量
uint8_t sbus_frame_buffer[SBUS_FRAME_SIZE];  // 用于存储完整的SBUS帧的缓冲区
uint8_t sbus_frame_index = 0;                // 用于跟踪当前接收了多少字节
uint8_t sbus_frame_complete = 0;             // 用于标记是否接收到完整的帧
uint8_t waiting_for_header = 1;              // 标记是否正在等待帧头

// 维特智能 HWT101CT 陀螺仪变量 (UART7 - PE7/PE8)
volatile float robot_yaw = 0.0f;             // 最终输出的航向角
volatile float robot_pitch = 0.0f;           // 俯仰角
volatile float robot_roll = 0.0f;            // 横滚角
volatile float robot_yaw_rate_dps = 0.0f;    // Z轴角速度，单位 deg/s
volatile uint8_t gyro_rx_data = 0;           // 串口7接收单个字节的临时变量
uint8_t gyro_buffer[HWT101_FRAME_SIZE];      // 维特智能协议标准 11 字节缓冲区
uint8_t gyro_index = 0;                      // 接收索引
float target_yaw = 0.0f;                     // 机器人的目标航向角

// 底盘相关变量

#endif
uint8_t rx_data = 0;
uint8_t sbus_frame_buffer[SBUS_FRAME_SIZE] = {0};
uint8_t sbus_frame_index = 0;
uint8_t sbus_frame_complete = 0;
uint8_t waiting_for_header = 1;
volatile uint32_t sbus_rx_byte_count = 0;
volatile uint32_t sbus_header_count = 0;
volatile uint32_t sbus_frame_count = 0;
volatile uint32_t sbus_update_ok_count = 0;
volatile uint32_t sbus_update_fail_count = 0;
volatile uint32_t sbus_uart_error_count = 0;
volatile uint8_t sbus_last_byte = 0;
volatile uint8_t sbus_last_frame_flag = 0;
volatile uint8_t sbus_last_frame_end = 0;
volatile uint32_t sbus_last_byte_tick = 0;
volatile uint32_t sbus_last_frame_tick = 0;

volatile float robot_yaw = 0.0f;
volatile float robot_pitch = 0.0f;
volatile float robot_roll = 0.0f;
volatile float robot_yaw_rate_dps = 0.0f;
volatile uint8_t gyro_rx_data = 0;
uint8_t gyro_buffer[HWT101_FRAME_SIZE] = {0};
uint8_t gyro_index = 0;
float target_yaw = 0.0f;
volatile uint32_t gyro_rx_byte_count = 0;
volatile uint32_t gyro_frame_ok_count = 0;
volatile uint32_t gyro_frame_bad_count = 0;
volatile uint8_t gyro_last_byte = 0;
volatile uint8_t gyro_last_frame_type = 0;
volatile uint32_t gyro_last_frame_tick = 0;
uint32_t gyro_current_baud = 9600;
volatile uint8_t ir8_rx_byte = 0;
volatile uint8_t ir8_received_123 = 0;
volatile uint32_t ir8_rx_byte_count = 0;
volatile uint32_t ir8_received_123_count = 0;
static uint8_t ir8_match_index = 0;

Class_Serialplot serialplot;
Class_Motor_C620 motor1;  // 左前电机
Class_Motor_C620 motor2;  // 左后电机
Class_Motor_C620 motor3;  // 右后电机
Class_Motor_C620 motor4;  // 右前电机
Class_Motor_C620 motor5;  // 升降台左侧电机
Class_Motor_C620 motor6;  // 升降台右侧电机
Class_Motor_C620 lift_extend_motor;        // CAN2 0x205: 龙门架平动伸缩
Class_Motor_C620 suction_claw_left_motor;  // CAN2 0x203: 吸夹翻转左电机
Class_Motor_C620 suction_claw_right_motor; // CAN2 0x204: 吸夹翻转右电机
Class_Motor_C610 claw_extend_motor;        // CAN1 0x205: CH7 夹爪伸缩
Class_Motor_C610 claw_flip_motor;          // CAN1 0x206: CH7 夹爪翻转
float chassis_target_angle[4] = {0.0f};    // motor1~4 的位置环目标角

// 机械臂 (CAN2)
Class_Motor_DM_Normal arm_dm_1;     //DM_J4310_2ECV1.1
Class_Motor_DM_Normal arm_dm_2;     //DM_J4310_2ECV1.1
Class_Motor_C620      arm_m3508;    // M3508   (ID: 0x201)
Class_Motor_C620      arm_m2006_1;  // M2006_1 (ID: 0x202) 
Class_Motor_C620      arm_m2006_2;  // M2006_2 (ID: 0x203)

float Target_Omega_1, Now_Omega_1
, Target_Omega_2, Now_Omega_2
, Target_Omega_3, Now_Omega_3
, Target_Omega_4, Now_Omega_4,Target_Omega;

float torque, fx=0, target_torque;

// 机构目标角度缓存
float arm_target[5] = {0.0f};
float lift_device_target[3] = {0.0f}; // CAN2 0x205 平动、0x203/0x204 吸夹翻转
float claw_motor_target[2] = {0.0f};  // CAN1 0x205 伸缩、0x206 翻转

// 速度平滑滤波变量
float vx_f = 0.0f, vy_f = 0.0f, w_f = 0.0f;

uint32_t Counter = 0;

static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
   //电机调PID
	//角度
    "pa",
    "ia",
    "da",
	//速度
    "po",
    "io",
    "do",
	"torque",
	"fx",
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 全向轮运动控制函数声明
void omni_move(float vx, float vy, float w);
void move_front(float speed);
void move_back(float speed);
void move_left(float speed);
void move_right(float speed);
void turn_left(float speed);
void turn_right(float speed);
void stop(float speed);
void Direction_Init(void);

// SBUS 相关回调和处理函数声明
void execute_sbus_motion_commands(void); // 根据SBUS解析结果执行动作
static void Control_ClawRelay_By_CH8(void);
static void Control_SuctionClawRelay_By_CH6(void);
void CAN1_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer);
void CAN2_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer);
static void Sbus_RestartReceive(void);

// 陀螺仪与红外串口
static void Gyro_ClearUartErrors(void);
static void Gyro_SetUartBaud(uint32_t baud_rate);
static HAL_StatusTypeDef Gyro_SendRawCommand(const uint8_t *command);
static HAL_StatusTypeDef Gyro_WriteRegister(uint8_t reg, uint16_t value);
static void Gyro_Configure(void);
static void Gyro_StartReceive(void);
static void IR8_ClearUartErrors(void);
static void IR8_StartReceive(void);
static uint32_t IR8_GetBoardId(void);
static void USART2_DebugSend(char *buffer, uint16_t buffer_size, int length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Sbus_RestartReceive(void)
{
    volatile uint32_t tmp;

    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_PE);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);

    // On STM32F4, PE/FE/NE/ORE are cleared by reading SR followed by DR.
    tmp = huart1.Instance->SR;
    tmp = huart1.Instance->DR;
    (void)tmp;

    huart1.ErrorCode = HAL_UART_ERROR_NONE;
    huart1.RxState = HAL_UART_STATE_READY;
    __HAL_UNLOCK(&huart1);

    waiting_for_header = 1;
    sbus_frame_index = 0;
    sbus_frame_complete = 0;
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

static void Gyro_ClearUartErrors(void)
{
    volatile uint32_t tmp;

    __HAL_UART_DISABLE_IT(&huart7, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart7, UART_IT_PE);
    __HAL_UART_DISABLE_IT(&huart7, UART_IT_ERR);

    tmp = huart7.Instance->SR;
    tmp = huart7.Instance->DR;
    (void)tmp;

    gyro_index = 0;
    huart7.ErrorCode = HAL_UART_ERROR_NONE;
    huart7.RxState = HAL_UART_STATE_READY;
    __HAL_UNLOCK(&huart7);
}

static void Gyro_SetUartBaud(uint32_t baud_rate)
{
    HAL_UART_DeInit(&huart7);
    huart7.Init.BaudRate = baud_rate;
    gyro_current_baud = baud_rate;

    if (HAL_UART_Init(&huart7) != HAL_OK)
    {
        Error_Handler();
    }

    Gyro_ClearUartErrors();
}

static HAL_StatusTypeDef Gyro_SendRawCommand(const uint8_t *command)
{
    return HAL_UART_Transmit(&huart7, const_cast<uint8_t *>(command), 5, 100);
}

static HAL_StatusTypeDef Gyro_WriteRegister(uint8_t reg, uint16_t value)
{
    uint8_t command[5] = {
        0xFF,
        0xAA,
        reg,
        static_cast<uint8_t>(value & 0x00FFU),
        static_cast<uint8_t>((value >> 8) & 0x00FFU)
    };

    return HAL_UART_Transmit(&huart7, command, sizeof(command), 100);
}

static void Gyro_Configure(void)
{
    static const uint8_t unlock_command[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    static const uint8_t save_command[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};

    // 先按常见的 9600 波特率尝试配置，再把链路切到 115200，给 100Hz 角度+角速度留足带宽。
    Gyro_SetUartBaud(9600);
    HAL_Delay(20);
    Gyro_SendRawCommand(unlock_command);
    HAL_Delay(2);
    Gyro_WriteRegister(HWT101_REG_RSW, HWT101_OUTPUT_ANGLE_ONLY);
    HAL_Delay(2);
    Gyro_WriteRegister(HWT101_REG_RRATE, HWT101_RRATE_100HZ);
    HAL_Delay(2);
    Gyro_SendRawCommand(save_command);
    HAL_Delay(50);

    // 再用 115200 复写一次，兼容模块已经被配置成 115200 的情况。
    robot_yaw_rate_dps = 0.0f;
}

static void Gyro_StartReceive(void)
{
    if (HAL_UART_Receive_IT(&huart7, const_cast<uint8_t *>(&gyro_rx_data), 1) != HAL_OK)
    {
        Gyro_ClearUartErrors();
        HAL_UART_Receive_IT(&huart7, const_cast<uint8_t *>(&gyro_rx_data), 1);
    }
}

static void IR8_ClearUartErrors(void)
{
    volatile uint32_t tmp;

    __HAL_UART_DISABLE_IT(&huart8, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart8, UART_IT_PE);
    __HAL_UART_DISABLE_IT(&huart8, UART_IT_ERR);

    tmp = huart8.Instance->SR;
    tmp = huart8.Instance->DR;
    (void)tmp;

    ir8_match_index = 0;
    huart8.ErrorCode = HAL_UART_ERROR_NONE;
    huart8.RxState = HAL_UART_STATE_READY;
    __HAL_UNLOCK(&huart8);
}

static void IR8_StartReceive(void)
{
    if (HAL_UART_Receive_IT(&huart8, const_cast<uint8_t *>(&ir8_rx_byte), 1) != HAL_OK)
    {
        IR8_ClearUartErrors();
        HAL_UART_Receive_IT(&huart8, const_cast<uint8_t *>(&ir8_rx_byte), 1);
    }
}

static uint32_t IR8_GetBoardId(void)
{
    return HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
}

static void USART2_DebugSend(char *buffer, uint16_t buffer_size, int length)
{
    if (buffer == nullptr || buffer_size == 0U || length <= 0)
    {
        return;
    }

    if (length >= (int)buffer_size)
    {
        length = (int)buffer_size - 1;
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, 50);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 检查是否在等待帧头
        sbus_rx_byte_count++;
        sbus_last_byte = rx_data;
        sbus_last_byte_tick = HAL_GetTick();

        if (waiting_for_header)
        {
            // 如果当前字节是帧头 (0x0F)
            if (rx_data == 0x0F)
            {
                // 将帧头存入缓冲区第一个位置
                sbus_header_count++;
                sbus_frame_buffer[0] = rx_data;
                // 设置索引为1，准备接收下一字节
                sbus_frame_index = 1;
                // 不再等待帧头
                waiting_for_header = 0;
            }
            // 如果当前字节不是帧头，继续等待，不做任何操作
        }
        else // 不在等待帧头，说明已经在接收一帧数据
        {
            // 将接收到的一个字节存入临时缓冲区的当前索引位置
            sbus_frame_buffer[sbus_frame_index] = rx_data;
            // 更新索引
            sbus_frame_index++;
            // 检查是否接收完一帧 (25字节)
            if (sbus_frame_index >= SBUS_FRAME_SIZE)
            {
                // 标记帧已完整接收
                sbus_frame_count++;
                sbus_last_frame_flag = sbus_frame_buffer[23];
                sbus_last_frame_end = sbus_frame_buffer[24];
                sbus_last_frame_tick = HAL_GetTick();
                sbus_frame_complete = 1;
                // 重新开始等待下一帧的帧头
                waiting_for_header = 1;
            }
        }
        // 重新启动接收下一个字节
        if (HAL_UART_Receive_IT(&huart1, &rx_data, 1) != HAL_OK)
        {
            Sbus_RestartReceive();
        }
    }
    // 处理维特智能陀螺仪数据 (0x55 0x53 角度包)
    else if (huart->Instance == UART7)
    {
        // 适配 WitMotion HWT101CT-TTL 协议陀螺仪
        gyro_last_byte = gyro_rx_data;
        gyro_rx_byte_count++;
        if (gyro_index == 0)
        {
            if (gyro_rx_data != 0x55)
            {
                Gyro_StartReceive();
                return;
            }

            gyro_buffer[gyro_index] = gyro_rx_data;
            gyro_index++;
        }
            else
        {
            gyro_buffer[gyro_index] = gyro_rx_data;
            gyro_index++;
            
            if (gyro_index >= HWT101_FRAME_SIZE)
            {
                gyro_index = 0; 
                
                uint8_t sum = 0;
                for (int i = 0; i < 10; i++) 
                {
                    sum += gyro_buffer[i];
                }
                
                if (sum == gyro_buffer[10])
                {                    
                    gyro_frame_ok_count++;
                    gyro_last_frame_type = gyro_buffer[1];
                    gyro_last_frame_tick = HAL_GetTick();
                    if (gyro_buffer[1] == 0x52)
                    {
                        int16_t yaw_rate_raw = (int16_t)((gyro_buffer[7] << 8) | gyro_buffer[6]);
                        robot_yaw_rate_dps = (float)yaw_rate_raw / 32768.0f * 2000.0f;
                    }
                    else if (gyro_buffer[1] == 0x53) 
                    {
                        int16_t roll_raw  = (int16_t)((gyro_buffer[3] << 8) | gyro_buffer[2]);
                        int16_t pitch_raw = (int16_t)((gyro_buffer[5] << 8) | gyro_buffer[4]);
                        int16_t yaw_raw   = (int16_t)((gyro_buffer[7] << 8) | gyro_buffer[6]);
                        
                        robot_yaw_rate_dps = 0.0f;
                        robot_roll  = (float)roll_raw / 32768.0f * 180.0f;
                        robot_pitch = (float)pitch_raw / 32768.0f * 180.0f;
                        robot_yaw   = (float)yaw_raw / 32768.0f * 180.0f;
                    }
                }
                else
                {
                    gyro_frame_bad_count++;
                }
            }
        }
        Gyro_StartReceive();
    }
    else if (huart->Instance == UART8)
    {
        static const uint8_t ir8_pattern[] = {'1', '2', '3'};

        ir8_rx_byte_count++;

        if (ir8_rx_byte == ir8_pattern[ir8_match_index])
        {
            ir8_match_index++;
            if (ir8_match_index >= sizeof(ir8_pattern))
            {
                ir8_received_123 = 1;
                ir8_received_123_count++;
                ir8_match_index = 0;
            }
        }
        else
        {
            ir8_match_index = (ir8_rx_byte == ir8_pattern[0]) ? 1 : 0;
        }

        IR8_StartReceive();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        sbus_uart_error_count++;
        Sbus_RestartReceive();
    }
    // UART7 溢出或错误时自动恢复陀螺仪接收。
    else if (huart->Instance == UART7)
    {
        Gyro_ClearUartErrors();
        Gyro_StartReceive();
    }
    else if (huart->Instance == UART8)
    {
        IR8_ClearUartErrors();
        IR8_StartReceive();
    }
}

// CH7 三档控制夹爪总成：PD14 继电器 + CAN1 0x205/0x206。
static uint8_t Get_CH7_Stage(uint16_t ch7_value)
{
    if (ch7_value < 600U) return 0U;
    if (ch7_value < 1400U) return 1U;
    return 2U;
}

static void Control_ClawAssembly_By_CH7(void)
{
    enum {
        CLAW_STAGE_OPEN = 0,
        CLAW_STAGE_CLAMPED = 1,
        CLAW_STAGE_STOWED = 2
    };

    enum {
        CLAW_SEQ_START_EXTEND = 0,
        CLAW_SEQ_IDLE,
        CLAW_SEQ_TO_HIGH_FLIP,
        CLAW_SEQ_TO_HIGH_WAIT,
        CLAW_SEQ_TO_HIGH_RETRACT,
        CLAW_SEQ_TO_MID_EXTEND,
        CLAW_SEQ_TO_MID_UNFLIP
    };

    const float claw_extend_direction = -1.0f;
    const float claw_flip_direction = 1.0f;   // 若 0x206 翻转方向反了，改为 -1.0f。
    const float claw_extend_angle = 2.0f * 2.0f * PI;
    const float claw_flip_angle = 1.5f * 2.0f * PI;
    const float claw_angle_done_error = 0.08f * 2.0f * PI;
    const uint32_t claw_extend_timeout_ms = 2500U;
    const uint32_t claw_flip_timeout_ms = 5000U;
    const uint32_t high_flip_wait_ms = 1000U;

    static uint8_t init_flag = 0;
    static uint8_t current_stage = CLAW_STAGE_OPEN;
    static uint8_t seq_state = CLAW_SEQ_START_EXTEND;
    static uint8_t last_target_stage = CLAW_STAGE_OPEN;
    static float extend_retracted_target = 0.0f;
    static float extend_extended_target = 0.0f;
    static float flip_origin_target = 0.0f;
    static float flip_flipped_target = 0.0f;
    static uint32_t seq_enter_tick = 0U;
    static uint32_t high_flip_done_tick = 0U;

    if (!SBUS_CH.ConnectState) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
        return;
    }

    uint8_t target_stage = Get_CH7_Stage(G_SbusValue.Raw_CH7);
    uint32_t now_tick = HAL_GetTick();

    if (!init_flag) {
        extend_retracted_target = claw_extend_motor.Get_Now_Angle();
        extend_extended_target = extend_retracted_target + claw_extend_direction * claw_extend_angle;
        flip_origin_target = claw_flip_motor.Get_Now_Angle();
        flip_flipped_target = flip_origin_target + claw_flip_direction * claw_flip_angle;
        claw_motor_target[0] = extend_extended_target;
        claw_motor_target[1] = flip_origin_target;
        current_stage = CLAW_STAGE_OPEN;
        seq_state = CLAW_SEQ_START_EXTEND;
        last_target_stage = target_stage;
        seq_enter_tick = now_tick;
        init_flag = 1;
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    }

    if ((target_stage == CLAW_STAGE_STOWED) &&
        (seq_state != CLAW_SEQ_TO_HIGH_FLIP) &&
        (seq_state != CLAW_SEQ_TO_HIGH_WAIT) &&
        (seq_state != CLAW_SEQ_TO_HIGH_RETRACT) &&
        ((current_stage != CLAW_STAGE_STOWED) ||
         (fabsf(claw_flip_motor.Get_Now_Angle() - flip_flipped_target) >= claw_angle_done_error))) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        current_stage = CLAW_STAGE_CLAMPED;
        seq_state = CLAW_SEQ_TO_HIGH_FLIP;
        seq_enter_tick = now_tick;
    }

    if (target_stage != last_target_stage) {
        last_target_stage = target_stage;
        if (target_stage != CLAW_STAGE_STOWED) {
            if ((current_stage == CLAW_STAGE_STOWED) ||
                (seq_state == CLAW_SEQ_TO_HIGH_RETRACT)) {
                seq_state = CLAW_SEQ_TO_MID_EXTEND;
                seq_enter_tick = now_tick;
            } else if ((seq_state == CLAW_SEQ_TO_HIGH_FLIP) ||
                       (seq_state == CLAW_SEQ_TO_HIGH_WAIT)) {
                seq_state = CLAW_SEQ_TO_MID_UNFLIP;
                seq_enter_tick = now_tick;
            }
        }
    }

    switch (seq_state) {
    case CLAW_SEQ_START_EXTEND:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,
                          (target_stage >= CLAW_STAGE_CLAMPED) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        claw_motor_target[0] = extend_extended_target;
        claw_motor_target[1] = flip_origin_target;
        if ((fabsf(claw_extend_motor.Get_Now_Angle() - extend_extended_target) < claw_angle_done_error) ||
            ((now_tick - seq_enter_tick) >= claw_extend_timeout_ms)) {
            current_stage = (target_stage >= CLAW_STAGE_CLAMPED) ? CLAW_STAGE_CLAMPED : CLAW_STAGE_OPEN;
            if (target_stage == CLAW_STAGE_STOWED) {
                seq_state = CLAW_SEQ_TO_HIGH_FLIP;
            } else {
                seq_state = CLAW_SEQ_IDLE;
            }
            seq_enter_tick = now_tick;
        }
        break;

    case CLAW_SEQ_IDLE:
        if (current_stage == CLAW_STAGE_OPEN) {
            claw_motor_target[0] = extend_extended_target;
            claw_motor_target[1] = flip_origin_target;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,
                              (target_stage >= CLAW_STAGE_CLAMPED) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            if (target_stage >= CLAW_STAGE_CLAMPED) {
                current_stage = CLAW_STAGE_CLAMPED;
                if (target_stage == CLAW_STAGE_STOWED) {
                    seq_state = CLAW_SEQ_TO_HIGH_FLIP;
                    seq_enter_tick = now_tick;
                }
            }
        } else if (current_stage == CLAW_STAGE_CLAMPED) {
            claw_motor_target[0] = extend_extended_target;
            claw_motor_target[1] = flip_origin_target;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,
                              (target_stage == CLAW_STAGE_OPEN) ? GPIO_PIN_RESET : GPIO_PIN_SET);
            if (target_stage == CLAW_STAGE_OPEN) {
                current_stage = CLAW_STAGE_OPEN;
            } else if (target_stage == CLAW_STAGE_STOWED) {
                seq_state = CLAW_SEQ_TO_HIGH_FLIP;
                seq_enter_tick = now_tick;
            }
        } else {
            claw_motor_target[0] = extend_retracted_target;
            claw_motor_target[1] = flip_flipped_target;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
            if (target_stage != CLAW_STAGE_STOWED) {
                seq_state = CLAW_SEQ_TO_MID_EXTEND;
                seq_enter_tick = now_tick;
            } else if (fabsf(claw_flip_motor.Get_Now_Angle() - flip_flipped_target) >= claw_angle_done_error) {
                current_stage = CLAW_STAGE_CLAMPED;
                seq_state = CLAW_SEQ_TO_HIGH_FLIP;
                seq_enter_tick = now_tick;
            }
        }
        break;

    case CLAW_SEQ_TO_HIGH_FLIP:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        claw_motor_target[0] = extend_extended_target;
        claw_motor_target[1] = flip_flipped_target;
        if ((fabsf(claw_flip_motor.Get_Now_Angle() - flip_flipped_target) < claw_angle_done_error) ||
            ((now_tick - seq_enter_tick) >= claw_flip_timeout_ms)) {
            high_flip_done_tick = now_tick;
            seq_state = CLAW_SEQ_TO_HIGH_WAIT;
            seq_enter_tick = now_tick;
        }
        break;

    case CLAW_SEQ_TO_HIGH_WAIT:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        claw_motor_target[0] = extend_extended_target;
        claw_motor_target[1] = flip_flipped_target;
        if ((HAL_GetTick() - high_flip_done_tick) >= high_flip_wait_ms) {
            seq_state = CLAW_SEQ_TO_HIGH_RETRACT;
            seq_enter_tick = now_tick;
        }
        break;

    case CLAW_SEQ_TO_HIGH_RETRACT:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        claw_motor_target[0] = extend_retracted_target;
        claw_motor_target[1] = flip_flipped_target;
        if ((fabsf(claw_extend_motor.Get_Now_Angle() - extend_retracted_target) < claw_angle_done_error) ||
            ((now_tick - seq_enter_tick) >= claw_extend_timeout_ms)) {
            current_stage = CLAW_STAGE_STOWED;
            seq_state = CLAW_SEQ_IDLE;
            seq_enter_tick = now_tick;
        }
        break;

    case CLAW_SEQ_TO_MID_EXTEND:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        claw_motor_target[0] = extend_extended_target;
        claw_motor_target[1] = flip_flipped_target;
        if ((fabsf(claw_extend_motor.Get_Now_Angle() - extend_extended_target) < claw_angle_done_error) ||
            ((now_tick - seq_enter_tick) >= claw_extend_timeout_ms)) {
            seq_state = CLAW_SEQ_TO_MID_UNFLIP;
            seq_enter_tick = now_tick;
        }
        break;

    case CLAW_SEQ_TO_MID_UNFLIP:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        claw_motor_target[0] = extend_extended_target;
        claw_motor_target[1] = flip_origin_target;
        if ((fabsf(claw_flip_motor.Get_Now_Angle() - flip_origin_target) < claw_angle_done_error) ||
            ((now_tick - seq_enter_tick) >= claw_flip_timeout_ms)) {
            current_stage = CLAW_STAGE_CLAMPED;
            seq_state = CLAW_SEQ_IDLE;
            seq_enter_tick = now_tick;
        }
        break;

    default:
        seq_state = CLAW_SEQ_IDLE;
        seq_enter_tick = now_tick;
        break;
    }
}
// CH8 继电器输出，高电平触发。
static void Control_ClawRelay_By_CH8(void)
{
    const uint16_t claw_ch8_on_threshold = 1192U; // 992 中位以上约 200 作为开启阈值
    GPIO_PinState relay_state = GPIO_PIN_RESET;

    if (SBUS_CH.ConnectState && SBUS_CH.CH8 > claw_ch8_on_threshold) {
        relay_state = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, relay_state);
}

static void Control_SuctionClawRelay_By_CH6(void)
{
    const uint16_t ch6_claw_threshold = 600U;
    const uint16_t ch6_suction_threshold = 1400U;
    GPIO_PinState suction_state = GPIO_PIN_RESET;
    GPIO_PinState claw_state = GPIO_PIN_RESET;

    // CH6 三档控制吸夹继电器：1792 两路关闭，992 只开吸盘，192 吸盘保持并闭合夹爪。
    if (SBUS_CH.ConnectState) {
        if (SBUS_CH.CH6 < ch6_claw_threshold) {
            suction_state = GPIO_PIN_SET;
            claw_state = GPIO_PIN_SET;
        } else if (SBUS_CH.CH6 < ch6_suction_threshold) {
            suction_state = GPIO_PIN_SET;
        }
    }

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, suction_state);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, claw_state);
}

static void Chassis_SyncTargetToNow(void)
{
    vx_f = 0.0f;
    vy_f = 0.0f;
    w_f = 0.0f;

    chassis_target_angle[0] = motor1.Get_Now_Angle(); // motor1: left front
    chassis_target_angle[1] = motor2.Get_Now_Angle(); // motor2: left rear
    chassis_target_angle[2] = motor3.Get_Now_Angle(); // motor3: right rear
    chassis_target_angle[3] = motor4.Get_Now_Angle(); // motor4: right front

    motor1.Set_Target_Angle(chassis_target_angle[0]);
    motor2.Set_Target_Angle(chassis_target_angle[1]);
    motor3.Set_Target_Angle(chassis_target_angle[2]);
    motor4.Set_Target_Angle(chassis_target_angle[3]);
}

void execute_sbus_motion_commands(void) {

    Control_ClawRelay_By_CH8();
    Control_SuctionClawRelay_By_CH6();
    // 获取 SBUS 解析模块处理后的值
    float target_vx_raw = -(G_SbusValue.Vx / 100.0f) * MAX_WHEEL_SPEED;
    float target_vy_raw = (G_SbusValue.Vy / 100.0f) * MAX_WHEEL_SPEED;
    float target_w_raw = (G_SbusValue.W / 100.0f) * MAX_WHEEL_SPEED;
    const float current_yaw = robot_yaw;

    // 机构上电初始化计时。初始化期间只同步目标，不响应机械臂手动增量。
    static uint16_t arm_init_timer = 0; 
    static uint8_t arm_init_flag = 0;
    // 角度单位为 rad；1 圈 = 2*PI。
    float speed_dm = 1.0f;
    float speed_m3508 = 3.0f;
    float speed_m2006 = 5.0f;
    float lift_manual_speed = 5.0f;
    const float lift_extend_direction = 1.0f;                       // CAN2 0x205 方向，反了改为 -1.0f
    const float lift_extend_home_to_fixed_angle = 2.0f * 2.0f * PI; // 上电固定起点 -> CH9=192 伸出固定点
    const float lift_extend_retract_angle = 3.0f * 2.0f * PI;       // CH9=1792 时从固定点反转 3 圈
    const float suction_claw_flip_angle = (110.0f / 360.0f) * 2.0f * PI; // CH5 使能后的基础翻转角
    const float suction_claw_fine_speed = 0.6f;                         // CH11 满量程微调速度，单位 rad/s
    const float suction_claw_fine_limit = (30.0f / 180.0f) * PI;        // CH11 累计微调限幅，避免目标角无限偏移
    static float lift_target_rad = 0.0f;
    static float lift_device_origin[3] = {0.0f};
    static float lift_extend_stage_target[3] = {0.0f};
    static float suction_claw_fine_offset = 0.0f;                       // CH11 累计出的附加翻转角
    static uint8_t lift_extend_home_done = 0;
    static uint8_t lift_extend_command_stage = 0;
    static uint8_t lift_extend_last_switch_stage = 0;

    

  

    // 备用机械臂角度限制
    float arm_max_angle = 10.0f * 2.0f * PI;

    // 大疆备用电机软限位，单位为输出轴 rad。
    float m3508_max_angle = 3.5f * 2.0f * PI;
    float m2006_max_angle = 5.0f * 2.0f * PI;


    if (arm_init_flag == 0) {
        if (arm_init_timer == 0) {
        arm_target[0] = arm_dm_1.Get_Now_Angle(); 
        arm_target[1] = arm_dm_2.Get_Now_Angle();
        arm_target[2] = arm_m3508.Get_Now_Angle();
        arm_target[3] = arm_m2006_1.Get_Now_Angle();
        arm_target[4] = arm_m2006_2.Get_Now_Angle();
        lift_target_rad = 0.5f * (-motor5.Get_Now_Angle() + motor6.Get_Now_Angle());
        lift_device_origin[0] = lift_extend_motor.Get_Now_Angle();
        lift_device_target[0] = lift_extend_motor.Get_Now_Angle();
        lift_device_target[1] = suction_claw_left_motor.Get_Now_Angle();
        lift_device_target[2] = suction_claw_right_motor.Get_Now_Angle();
        claw_motor_target[0] = claw_extend_motor.Get_Now_Angle();
        claw_motor_target[1] = claw_flip_motor.Get_Now_Angle();
        lift_extend_stage_target[0] = lift_device_origin[0] + lift_extend_direction * lift_extend_home_to_fixed_angle;
        lift_extend_stage_target[1] = lift_extend_stage_target[0] - lift_extend_direction * lift_extend_retract_angle;
        lift_extend_stage_target[2] = lift_extend_stage_target[0];
        lift_extend_home_done = 0;
        lift_extend_command_stage = 0;
        lift_extend_last_switch_stage = 0;
        lift_device_target[0] = lift_device_origin[0];
        lift_device_origin[1] = lift_device_target[1];
        lift_device_origin[2] = lift_device_target[2];
        motor5.Set_Target_Angle(-lift_target_rad);
        motor6.Set_Target_Angle(lift_target_rad);
        lift_extend_motor.Set_Target_Angle(lift_device_target[0]);
        suction_claw_left_motor.Set_Target_Angle(lift_device_target[1]);
        suction_claw_right_motor.Set_Target_Angle(lift_device_target[2]);
        claw_extend_motor.Set_Target_Angle(claw_motor_target[0]);
        claw_flip_motor.Set_Target_Angle(claw_motor_target[1]);

        // 上电时同步底盘目标角，避免第一次使能时位置环突跳。
        target_yaw = current_yaw; 
        Chassis_SyncTargetToNow();
        }
    
        arm_init_timer++;
        // 控制周期约 2ms，初始化约 2s 后再允许备用机械臂手动增量。
        if (arm_init_timer >= 1000) { 
            arm_init_flag = 1;
        }
    }
        
    Control_ClawAssembly_By_CH7();
    
    if (G_SbusValue.Aux2 >= -50.0f) { // CH10 使能
        arm_dm_1.Set_K_P(2.0f);
        arm_dm_1.Set_K_D(0.2f);
        arm_dm_2.Set_K_P(2.0f);
        arm_dm_2.Set_K_D(0.2f);

        // CH3 控制升降台：中位保持，变大上升，变小下降。
        lift_target_rad -= (G_SbusValue.LiftPercent / 100.0f) * lift_manual_speed * CHASSIS_CONTROL_DT;
        if (lift_target_rad > LIFT_MAX_ANGLE) lift_target_rad = LIFT_MAX_ANGLE;
        if (lift_target_rad < -LIFT_MAX_ANGLE) lift_target_rad = -LIFT_MAX_ANGLE;
        motor5.Set_Target_Angle(-lift_target_rad);
        motor6.Set_Target_Angle(lift_target_rad);

        uint8_t extend_stage = lift_extend_last_switch_stage;
        if (SBUS_CH.ConnectState) {
            extend_stage = (G_SbusValue.Arm_Axis3 > 75.0f) ? 1 : 0;
        }

        if (!lift_extend_home_done) {
            lift_device_target[0] = lift_extend_stage_target[0];
            if ((fabsf(lift_extend_motor.Get_Now_Angle() - lift_extend_stage_target[0]) < (0.15f * 2.0f * PI)) &&
                (arm_init_timer > 200))
            {
                lift_extend_home_done = 1;
                lift_extend_command_stage = extend_stage;
                if (SBUS_CH.ConnectState) {
                    lift_extend_last_switch_stage = extend_stage;
                }
            }
        } else {
            if (SBUS_CH.ConnectState && (extend_stage != lift_extend_last_switch_stage)) {
                lift_extend_last_switch_stage = extend_stage;
                lift_extend_command_stage = extend_stage;
            }
            lift_device_target[0] = lift_extend_stage_target[lift_extend_command_stage];
        }

        // CH5 负责进入基础翻转位；只有 CH5 使能后，CH11 才在基础角上做相对微调。
        float flip_offset = 0.0f;
        if (G_SbusValue.Arm_Axis4 > 50.0f) {
            suction_claw_fine_offset += (G_SbusValue.Arm_Axis5 / 100.0f) * suction_claw_fine_speed * CHASSIS_CONTROL_DT;
            if (suction_claw_fine_offset > suction_claw_fine_limit) suction_claw_fine_offset = suction_claw_fine_limit;
            if (suction_claw_fine_offset < -suction_claw_fine_limit) suction_claw_fine_offset = -suction_claw_fine_limit;
            flip_offset = suction_claw_flip_angle + suction_claw_fine_offset;
        } else {
            // 关闭 CH5 时回到吸夹翻转原点，同时清掉本次微调量。
            suction_claw_fine_offset = 0.0f;
        }
        lift_device_target[1] = lift_device_origin[1] + flip_offset; // CAN2 0x203
        lift_device_target[2] = lift_device_origin[2] - flip_offset; // CAN2 0x204，与 0x203 反向

    if (G_SbusValue.RobotMode == 0) {
        float final_w = target_w_raw;
        const float w_deadzone = 3.0f;

        if (fabsf(final_w) < w_deadzone) {
            final_w = 0.0f;
        }
        target_yaw = current_yaw;

        // 输出到底盘运动学解算。
        omni_move(target_vx_raw, target_vy_raw, final_w);
        
        arm_dm_1.Set_Control_Angle(arm_target[0]);
        arm_dm_2.Set_Control_Angle(arm_target[1]);
        arm_m3508.Set_Target_Angle(arm_target[2]); 
        arm_m2006_1.Set_Target_Angle(arm_target[3]);
        arm_m2006_2.Set_Target_Angle(arm_target[4]);
        lift_extend_motor.Set_Target_Angle(lift_device_target[0]);
        suction_claw_left_motor.Set_Target_Angle(lift_device_target[1]);
        suction_claw_right_motor.Set_Target_Angle(lift_device_target[2]);
        claw_extend_motor.Set_Target_Angle(claw_motor_target[0]);
        claw_flip_motor.Set_Target_Angle(claw_motor_target[1]);
       
    } 
    else 
    { 
        
        // 备用机械臂模式：底盘刹停。
        omni_move(0.0f, 0.0f, 0.0f);
        target_yaw = current_yaw;

        // 初始化完成后才允许备用机械臂目标角递增。
            if (arm_init_flag == 1) {
        // 达妙 1
        arm_target[0] += (G_SbusValue.Arm_Axis1 / 100.0f) * speed_dm * 0.001f;
        if(arm_target[0] > arm_max_angle) arm_target[0] = arm_max_angle;
        if(arm_target[0] < -arm_max_angle) arm_target[0] = -arm_max_angle;

          // 达妙 2
        arm_target[1] += (G_SbusValue.Arm_Axis2 / 100.0f) * speed_dm * 0.001f;
        if(arm_target[1] > arm_max_angle) arm_target[1] = arm_max_angle;
        if(arm_target[1] < -arm_max_angle) arm_target[1] = -arm_max_angle;

        // M3508
        arm_target[2] += (G_SbusValue.Arm_Axis3 / 100.0f) * speed_m3508 * 0.001f;
        if(arm_target[2] > m3508_max_angle) arm_target[2] = m3508_max_angle;
        if(arm_target[2] < -m3508_max_angle) arm_target[2] = -m3508_max_angle;

        // M2006 双电机差动
        float m2006_omega_cmd = (G_SbusValue.Arm_Axis4 / 100.0f) * speed_m2006 * 0.001f;
        
            arm_target[3] += m2006_omega_cmd;
            if(arm_target[3] > m2006_max_angle) arm_target[3] = m2006_max_angle;
            if(arm_target[3] < -m2006_max_angle) arm_target[3] = -m2006_max_angle;

            arm_target[4] -= m2006_omega_cmd; 
            if(arm_target[4] > m2006_max_angle) arm_target[4] = m2006_max_angle;
            if(arm_target[4] < -m2006_max_angle) arm_target[4] = -m2006_max_angle;

            }
            arm_dm_1.Set_Control_Angle(arm_target[0]);
            arm_dm_2.Set_Control_Angle(arm_target[1]);
            arm_m3508.Set_Target_Angle(arm_target[2]); 
            arm_m2006_1.Set_Target_Angle(arm_target[3]);
            arm_m2006_2.Set_Target_Angle(arm_target[4]);
            lift_extend_motor.Set_Target_Angle(lift_device_target[0]);
            suction_claw_left_motor.Set_Target_Angle(lift_device_target[1]);
            suction_claw_right_motor.Set_Target_Angle(lift_device_target[2]);
            claw_extend_motor.Set_Target_Angle(claw_motor_target[0]);
            claw_flip_motor.Set_Target_Angle(claw_motor_target[1]);
        }
    } else { // CH10 失能保护
        omni_move(0.0f, 0.0f, 0.0f);
        target_yaw = current_yaw;

        // 达妙电机失能，其他目标同步到当前位置，避免重新使能时突跳。
        arm_dm_1.Set_K_P(0.0f); arm_dm_1.Set_K_D(0.0f);
        arm_dm_2.Set_K_P(0.0f); arm_dm_2.Set_K_D(0.0f);
        
        arm_target[0] = arm_dm_1.Get_Now_Angle();
        arm_target[1] = arm_dm_2.Get_Now_Angle();
        arm_target[2] = arm_m3508.Get_Now_Angle();
        arm_target[3] = arm_m2006_1.Get_Now_Angle();
        arm_target[4] = arm_m2006_2.Get_Now_Angle();
        lift_target_rad = 0.5f * (-motor5.Get_Now_Angle() + motor6.Get_Now_Angle());
        lift_device_target[0] = lift_extend_motor.Get_Now_Angle();
        lift_device_target[1] = suction_claw_left_motor.Get_Now_Angle();
        lift_device_target[2] = suction_claw_right_motor.Get_Now_Angle();
        claw_motor_target[0] = claw_extend_motor.Get_Now_Angle();
        claw_motor_target[1] = claw_flip_motor.Get_Now_Angle();
        motor5.Set_Target_Angle(-lift_target_rad);
        motor6.Set_Target_Angle(lift_target_rad);
        lift_extend_motor.Set_Target_Angle(lift_device_target[0]);
        suction_claw_left_motor.Set_Target_Angle(lift_device_target[1]);
        suction_claw_right_motor.Set_Target_Angle(lift_device_target[2]);
        claw_extend_motor.Set_Target_Angle(claw_motor_target[0]);
        claw_flip_motor.Set_Target_Angle(claw_motor_target[1]);

     
    }
}
/**
 * @brief 四轮全向/麦轮底盘运动分配
 * @param vx 前进速度（+为前进，-为后退）
 * @param vy 侧向速度（+为左移，-为右移）
 * @param w  旋转角速度（+为逆时针，-为顺时针）
 */
void omni_move(float vx, float vy, float w)
{
    const float stop_deadzone = 0.01f;

    if ((fabsf(vx) < stop_deadzone) && (fabsf(vy) < stop_deadzone) && (fabsf(w) < stop_deadzone)) {
        Chassis_SyncTargetToNow();
        return;
    }

    // 一阶低通限制加速度，避免启动和换向过猛。
    float alpha = 0.06f;
    vx_f = (fabsf(vx) < stop_deadzone) ? 0.0f : (alpha * vx + (1.0f - alpha) * vx_f);
    vy_f = (fabsf(vy) < stop_deadzone) ? 0.0f : (alpha * vy + (1.0f - alpha) * vy_f);
    w_f  = (fabsf(w)  < stop_deadzone) ? 0.0f : (alpha * w  + (1.0f - alpha) * w_f);

    // K = (左右轮距 + 前后轴距) / 2，单位 m。
    float K = 0.25f; 

// FL/FR/RL/RR 表示车体视角：左前、右前、左后、右后。
float target_FL = -vx_f + vy_f + w_f * K;
float target_FR =  vx_f + vy_f + w_f * K;
float target_RL = -vx_f - vy_f + w_f * K;
float target_RR =  vx_f - vy_f + w_f * K;

// 右侧电机方向若需要反相，可在这里统一处理。
//target_FR = -target_FR;
//target_RR = -target_RR;

    // 归一化到最大轮速内，避免某个轮子饱和后运动方向失真。
    float max_wheel_speed = fmaxf(fmaxf(fabsf(target_FL), fabsf(target_FR)),
                                  fmaxf(fabsf(target_RL), fabsf(target_RR)));
    if (max_wheel_speed > MAX_WHEEL_SPEED) {
        target_FL = (target_FL / max_wheel_speed) * MAX_WHEEL_SPEED;
        target_FR = (target_FR / max_wheel_speed) * MAX_WHEEL_SPEED;
        target_RL = (target_RL / max_wheel_speed) * MAX_WHEEL_SPEED;
        target_RR = (target_RR / max_wheel_speed) * MAX_WHEEL_SPEED;
    }

    // 实际布置：motor1=左前，motor2=左后，motor3=右后，motor4=右前。
chassis_target_angle[0] += target_FL * CHASSIS_CONTROL_DT;
chassis_target_angle[3] += target_FR * CHASSIS_CONTROL_DT;
chassis_target_angle[1] += target_RL * CHASSIS_CONTROL_DT;
chassis_target_angle[2] += target_RR * CHASSIS_CONTROL_DT;

motor1.Set_Target_Angle(chassis_target_angle[0]);
motor4.Set_Target_Angle(chassis_target_angle[3]);
motor2.Set_Target_Angle(chassis_target_angle[1]);
motor3.Set_Target_Angle(chassis_target_angle[2]);

}

void (*last_fxfunction)(float speed); // 兼容旧的动作函数接口

//前进
void move_front(float speed)
{
    omni_move(speed, 0, 0);
}

//后退
void move_back(float speed)
{
    omni_move(-speed, 0, 0);
}

//向左
void move_left(float speed)
{
    omni_move(0, speed, 0);
}

//向右
void move_right(float speed)
{
    omni_move(0, -speed, 0);
}

//右转
void turn_right(float speed)
{
    omni_move(0, 0, -speed);
}

//左转
void turn_left(float speed)
{
    omni_move(0, 0, speed);
}         

//停止
void stop(float speed)
{
    omni_move(0, 0, 0);
}

//方向初始化
void Direction_Init(void)
{
	last_fxfunction=stop;
}


/**
 * @brief CAN报文回调函数
 *
 * @param Rx_Buffer CAN接收的信息结构体
 * * * @return void
 * @note 处理不同ID的电机CAN数据。
 */
void CAN1_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
        case (0x1B):
        { 
            arm_dm_1.CAN_RxCpltCallback(Rx_Buffer->Data);
         } break;
       case (0x201):
        {
            motor1.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x202):
        {
            motor2.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x203):
        {
            motor3.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x204):
        {
            motor4.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x205):
        {
            claw_extend_motor.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x206):
        {
            claw_flip_motor.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
    }
}
// CAN2 反馈：升降台、龙门架平动、吸夹翻转和达妙 2。
void CAN2_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId) {
        
        case (0x1A):
        {
            arm_dm_2.CAN_RxCpltCallback(Rx_Buffer->Data); 
        }
        break;
        case (0x201):
        {
             motor5.CAN_RxCpltCallback(Rx_Buffer->Data); 
        }
        break;
        case (0x202):
        {
             motor6.CAN_RxCpltCallback(Rx_Buffer->Data); 
        }
        break;
        case (0x203): 
        {
            suction_claw_left_motor.CAN_RxCpltCallback(Rx_Buffer->Data); 
         }
         break;
        case (0x204): 
        {
            suction_claw_right_motor.CAN_RxCpltCallback(Rx_Buffer->Data); 
         }
         break;
        case (0x205): 
        {
            lift_extend_motor.CAN_RxCpltCallback(Rx_Buffer->Data); 
         }
         break;
        }
}

/**
 * @brief HAL库UART接收DMA空闲中断
 * @param Buffer 接收缓冲区
 * @param Length 数据长度
 * @return void
 * @note 处理串口调参和运动指令。
 */
void UART_Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    serialplot.UART_RxCpltCallback(Buffer);
    switch (serialplot.Get_Variable_Index())
    {
        // 电机调PID
        case(0):
        {
            motor1.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
						motor2.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
						motor3.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
						motor4.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
           
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(1):
        {
            motor1.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
						motor2.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
						motor3.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
						motor4.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
            
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(2):
        {
            motor1.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
						motor2.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
						motor3.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
						motor4.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
           
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(3):
        {
           //motor1.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
						//motor2.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
						//motor3.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
						//motor4.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
           
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(4):
        {
            motor1.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
						motor2.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
						motor3.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
						motor4.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
           
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(5):
        {
            motor1.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
						motor2.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
						motor3.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
						motor4.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
           
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
				case(6):
        {
            torque=serialplot.Get_Variable_Value();
						// last_fxfunction(torque);  // 禁用：避免设置扭矩时重新执行运动
        }
        break;
				case(7):
        {
            fx=serialplot.Get_Variable_Value();
            if(fx==0.0f){stop(0); last_fxfunction=stop;}
            else if(fx==1.0f){move_front(torque); last_fxfunction=move_front;}
            else if(fx==2.0f){move_back(torque); last_fxfunction=move_back;}
            else if(fx==3.0f){move_right(torque); last_fxfunction=move_right;}
            else if(fx==4.0f){move_left(torque); last_fxfunction=move_left;}
            else if(fx==5.0f){turn_left(torque); last_fxfunction=turn_left;}
            else if(fx==6.0f){turn_right(torque); last_fxfunction=turn_right;}
            else{last_fxfunction(torque);}
    }
}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
  BSP_Init(BSP_DC24_LU_ON | BSP_DC24_LD_ON | BSP_DC24_RU_ON | BSP_DC24_RD_ON);

    //绑定双路 CAN
    CAN1_Manage_Object.Callback_Function = CAN1_Call_Back;
    CAN2_Manage_Object.Callback_Function = CAN2_Call_Back;

    CAN_Init(&hcan1, CAN1_Call_Back);
    CAN_Init(&hcan2, CAN2_Call_Back);

    UART_Init(&huart2, UART_Serialplot_Call_Back, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
    serialplot.Init(&huart2, 8, (char **)Variable_Assignment_List);

    Direction_Init();
		
   //初始化SBUS控制模块 
    SbusControl_Init();


    // 启动SBUS串口的中断接收（一次接收一个字节）
//char ready_msg[] = "SBUS receive started. Waiting for data...\r\n";
//HAL_UART_Transmit(&huart2, (uint8_t*)ready_msg, strlen(ready_msg), 10);
Sbus_RestartReceive();

// 启动 UART7 陀螺仪 (HWT101CT-TTL) 的中断接收。
Gyro_Configure();
Gyro_StartReceive();
IR8_StartReceive();

{
    char ir8_debug_str[128];
    int ir8_debug_len = snprintf(
        ir8_debug_str,
        sizeof(ir8_debug_str),
        "[IR8] test start, UART8 115200 8N1, send 123, USART2 debug, board=0x%08lX\r\n",
        (unsigned long)IR8_GetBoardId()
    );
    USART2_DebugSend(ir8_debug_str, sizeof(ir8_debug_str), ir8_debug_len);
}


// 底盘四轮 PID。外环输出目标转速，内环输出电调电流。
motor1.PID_Angle.Init(14.0f, 0.0f, 0.0f, 0.0f, 0.0f, MAX_WHEEL_SPEED, CHASSIS_CONTROL_DT, 0.0f);
motor2.PID_Angle.Init(14.0f, 0.0f, 0.0f, 0.0f, 0.0f, MAX_WHEEL_SPEED, CHASSIS_CONTROL_DT, 0.0f);
motor3.PID_Angle.Init(14.0f, 0.0f, 0.0f, 0.0f, 0.0f, MAX_WHEEL_SPEED, CHASSIS_CONTROL_DT, 0.0f);
motor4.PID_Angle.Init(14.0f, 0.0f, 0.0f, 0.0f, 0.0f, MAX_WHEEL_SPEED, CHASSIS_CONTROL_DT, 0.0f);

motor1.PID_Omega.Init(500.0f, 300.0f, 0.0f, 0.0f, 1000.0f, 5500.0f, CHASSIS_CONTROL_DT, 0.0f);
motor2.PID_Omega.Init(500.0f, 300.0f, 0.0f, 0.0f, 1000.0f, 5500.0f, CHASSIS_CONTROL_DT, 0.0f);
motor3.PID_Omega.Init(500.0f, 300.0f, 0.0f, 0.0f, 1000.0f, 5500.0f, CHASSIS_CONTROL_DT, 0.0f);
motor4.PID_Omega.Init(500.0f, 300.0f, 0.0f, 0.0f, 1000.0f, 5500.0f, CHASSIS_CONTROL_DT, 0.0f); 
		

// 升降台 PID：位置环保持高度，速度环提供抗负载电流。
    motor5.PID_Angle.Init(35.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.001f, 0.0f);
    motor6.PID_Angle.Init(35.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.001f, 0.0f);
    
    motor5.PID_Omega.Init(800.0f, 50.0f, 0.0f, 0.0f, 3000.0f, 10000.0f, 0.001f, 0.0f);
    motor6.PID_Omega.Init(800.0f, 50.0f, 0.0f, 0.0f, 3000.0f, 10000.0f, 0.001f, 0.0f);

    lift_extend_motor.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 0.0f, 40.0f);
    lift_extend_motor.PID_Omega.Init(800.0f, 50.0f, 0.0f, 0.0f, 3000.0f, 10000.0f);
    suction_claw_left_motor.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 0.0f, 6.0f);
    suction_claw_left_motor.PID_Omega.Init(800.0f, 50.0f, 0.0f, 0.0f, 3000.0f, 10000.0f);
    suction_claw_right_motor.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 0.0f, 6.0f);
    suction_claw_right_motor.PID_Omega.Init(800.0f, 50.0f, 0.0f, 0.0f, 3000.0f, 10000.0f);

// 龙门架平动与吸夹翻转 PID。
    arm_m3508.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 0.0f, 40.0f);
    arm_m3508.PID_Omega.Init(800.0f, 50.0f, 0.0f, 0.0f, 3000.0f, 10000.0f);
    
    arm_m2006_1.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 0.0f, 60.0f);
    arm_m2006_1.PID_Omega.Init(500.0f, 50.0f, 0.0f, 0.0f, 2000.0f, 10000.0f);
    
    // arm_m2006_2.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 0.0f, 60.0f);
    // arm_m2006_2.PID_Omega.Init(500.0f, 50.0f, 0.0f, 0.0f, 2000.0f, 10000.0f);

    claw_extend_motor.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 0.0f, 60.0f);
    claw_extend_motor.PID_Omega.Init(500.0f, 50.0f, 0.0f, 0.0f, 2000.0f, 10000.0f);
    claw_flip_motor.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f);
    claw_flip_motor.PID_Omega.Init(700.0f, 60.0f, 0.0f, 0.0f, 2000.0f, 10000.0f);

// 达妙 MIT 模式默认刚度，失能时会在控制循环中清零。
    arm_dm_1.Set_K_P(10.0f);
    arm_dm_1.Set_K_D(0.3f);
    arm_dm_2.Set_K_P(10.0f);
    arm_dm_2.Set_K_D(0.3f);

// 电机 CAN ID 绑定。
    motor1.Init(&hcan1, CAN_Motor_ID_0x201, Control_Method_ANGLE, GEAR_RATIO);
    motor2.Init(&hcan1, CAN_Motor_ID_0x202, Control_Method_ANGLE, GEAR_RATIO);
    motor3.Init(&hcan1, CAN_Motor_ID_0x203, Control_Method_ANGLE, GEAR_RATIO);
    motor4.Init(&hcan1, CAN_Motor_ID_0x204, Control_Method_ANGLE, GEAR_RATIO);
    claw_extend_motor.Init(&hcan1, CAN_Motor_ID_0x205, Control_Method_ANGLE, 36.0f);
    claw_flip_motor.Init(&hcan1, CAN_Motor_ID_0x206, Control_Method_ANGLE, 36.0f);
    motor5.Init(&hcan2, CAN_Motor_ID_0x201, Control_Method_ANGLE, GEAR_RATIO);
    motor6.Init(&hcan2, CAN_Motor_ID_0x202, Control_Method_ANGLE, GEAR_RATIO);
    suction_claw_left_motor.Init(&hcan2, CAN_Motor_ID_0x203, Control_Method_ANGLE, 19.0f);
    suction_claw_right_motor.Init(&hcan2, CAN_Motor_ID_0x204, Control_Method_ANGLE, 19.0f);
    lift_extend_motor.Init(&hcan2, CAN_Motor_ID_0x205, Control_Method_ANGLE, 19.0f);

    // CAN2 0x201/0x202 已用于升降台，0x203/0x204 用于吸夹翻转，0x205 用于龙门架平动。
    // arm_m3508.Init(&hcan2, CAN_Motor_ID_0x201, Control_Method_ANGLE, 19.0f);
    // arm_m2006_1.Init(&hcan2, CAN_Motor_ID_0x202, Control_Method_ANGLE, 36.0f);
    // arm_m2006_2.Init(&hcan2, CAN_Motor_ID_0x203, Control_Method_ANGLE, 36.0f);
    arm_dm_1.Init(&hcan1, 0x1B, 0x0B, Motor_DM_Control_Method_NORMAL_MIT);
    arm_dm_2.Init(&hcan2, 0x1A, 0x0A, Motor_DM_Control_Method_NORMAL_MIT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t motion_counter = 0;  // 2ms 控制循环计数
  static uint32_t last_tick = 0;
  static uint32_t gyro_print_tick = 0;
  while (1)
  {
	// 主循环中检查是否有完整 SBUS 帧到达。
    if (sbus_frame_complete)
    {
        sbus_frame_complete = 0; // 清除标志位
        if (sbus_frame_buffer[0] == 0x0F)
        {
            // 调用sbus.c中的解析函数
            if(update_sbus(sbus_frame_buffer))
            {
                // 解析成功后更新处理后的通道值。USART2 调试输出默认关闭。
                sbus_update_ok_count++;
                SbusControl_ProcessData();

                static uint32_t last_sbus_debug_tick = 0;
                uint32_t now_tick = HAL_GetTick();
                if (now_tick - last_sbus_debug_tick >= 100U)
                {
                    last_sbus_debug_tick = now_tick;
                    char debug_str[420];
                    int debug_len = snprintf(
                        debug_str,
                        sizeof(debug_str),
                        "SBUS CH1:%u CH2:%u CH3:%u CH4:%u CH5:%u CH6:%u CH7:%u CH8:%u CH9:%u CH10:%u CH11:%u CH12:%u CH13:%u CH14:%u CH15:%u CH16:%u Conn:%u Mode:%u Aux2:%d PD13:%u\r\n",
                        SBUS_CH.CH1,
                        SBUS_CH.CH2,
                        SBUS_CH.CH3,
                        SBUS_CH.CH4,
                        SBUS_CH.CH5,
                        SBUS_CH.CH6,
                        SBUS_CH.CH7,
                        SBUS_CH.CH8,
                        SBUS_CH.CH9,
                        SBUS_CH.CH10,
                        SBUS_CH.CH11,
                        SBUS_CH.CH12,
                        SBUS_CH.CH13,
                        SBUS_CH.CH14,
                        SBUS_CH.CH15,
                        SBUS_CH.CH16,
                        SBUS_CH.ConnectState,
                        G_SbusValue.RobotMode,
                        (int)G_SbusValue.Aux2,
                        (unsigned int)HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13)
                    );

                    if (debug_len > 0)
                    {
                        if (debug_len >= (int)sizeof(debug_str))
                        {
                            debug_len = (int)sizeof(debug_str) - 1;
                        }
                        // HAL_UART_Transmit(&huart2, (uint8_t *)debug_str, (uint16_t)debug_len, 20);
                    }
                }
            }
            else
            {
                sbus_update_fail_count++;
            }
        }
    }

    // --- 串口绘图显示内容  ---
    /*Now_Omega_1 = motor1.Get_Now_Omega();
    Now_Omega_2 = motor2.Get_Now_Omega();
    Now_Omega_3 = motor3.Get_Now_Omega();
    Now_Omega_4 = motor4.Get_Now_Omega();
    Target_Omega = motor1.Get_Target_Omega();
    serialplot.Set_Data(5, &Target_Omega, &Now_Omega_1, &Now_Omega_2, &Now_Omega_3, &Now_Omega_4);
    serialplot.TIM_Write_PeriodElapsedCallback();*/

  // 2ms 控制和 CAN 发送周期。
      uint32_t current_tick = HAL_GetTick();
      static uint32_t last_sbus_rx_status_tick = 0;
      if (current_tick - last_sbus_rx_status_tick >= 500U)
      {
          last_sbus_rx_status_tick = current_tick;
          char sbus_status_str[260];
          int sbus_status_len = snprintf(
              sbus_status_str,
              sizeof(sbus_status_str),
              "SBUS_RX bytes:%lu hdr:%lu frames:%lu ok:%lu fail:%lu err:%lu last:0x%02X flag:0x%02X end:0x%02X idx:%u wait:%u uartErr:0x%08lX ageB:%lu ageF:%lu\r\n",
              (unsigned long)sbus_rx_byte_count,
              (unsigned long)sbus_header_count,
              (unsigned long)sbus_frame_count,
              (unsigned long)sbus_update_ok_count,
              (unsigned long)sbus_update_fail_count,
              (unsigned long)sbus_uart_error_count,
              sbus_last_byte,
              sbus_last_frame_flag,
              sbus_last_frame_end,
              sbus_frame_index,
              waiting_for_header,
              (unsigned long)huart1.ErrorCode,
              (unsigned long)(current_tick - sbus_last_byte_tick),
              (unsigned long)(current_tick - sbus_last_frame_tick)
          );

          if (sbus_status_len > 0)
          {
              if (sbus_status_len >= (int)sizeof(sbus_status_str))
              {
                  sbus_status_len = (int)sizeof(sbus_status_str) - 1;
              }
              // HAL_UART_Transmit(&huart2, (uint8_t *)sbus_status_str, (uint16_t)sbus_status_len, 20);
          }
      }
#if 0
            if (current_tick - gyro_print_tick >= 100)
      {
          gyro_print_tick = current_tick;

          const float debug_yaw = robot_yaw;
          const float debug_pitch = robot_pitch;
          const float debug_roll = robot_roll;
          const uint32_t debug_baud = gyro_current_baud;
          const uint32_t debug_rx_count = gyro_rx_byte_count;
          const uint32_t debug_ok_count = gyro_frame_ok_count;
          const uint32_t debug_bad_count = gyro_frame_bad_count;
          const uint8_t debug_last_byte = gyro_last_byte;
          const uint8_t debug_last_type = gyro_last_frame_type;
          char gyro_debug_str[160];
          int gyro_debug_len = snprintf(
              gyro_debug_str,
              sizeof(gyro_debug_str),
              "GYRO baud=%lu rx=%lu ok=%lu bad=%lu last=0x%02X type=0x%02X yaw=%.2f pitch=%.2f roll=%.2f\r\n",
              (unsigned long)debug_baud,
              (unsigned long)debug_rx_count,
              (unsigned long)debug_ok_count,
              (unsigned long)debug_bad_count,
              debug_last_byte,
              debug_last_type,
              debug_yaw,
              debug_pitch,
              debug_roll
          );

          if (gyro_debug_len > 0)
          {
              // HAL_UART_Transmit(&huart2, (uint8_t *)gyro_debug_str, (uint16_t)gyro_debug_len, 20);
          }
      }
#endif
      if (current_tick - last_tick >= 2) 
      {
          last_tick = current_tick;

          // 解析遥控器输入并下发各机构目标。
          execute_sbus_motion_commands(); 

          // 电机 PID 计算。
          motor1.TIM_PID_PeriodElapsedCallback(); // 左前
          motor2.TIM_PID_PeriodElapsedCallback(); // 左后
          motor3.TIM_PID_PeriodElapsedCallback(); // 右后
          motor4.TIM_PID_PeriodElapsedCallback(); // 右前
          motor5.TIM_PID_PeriodElapsedCallback(); 
          motor6.TIM_PID_PeriodElapsedCallback();
          lift_extend_motor.TIM_PID_PeriodElapsedCallback();
          suction_claw_left_motor.TIM_PID_PeriodElapsedCallback();
          suction_claw_right_motor.TIM_PID_PeriodElapsedCallback();
          claw_extend_motor.TIM_PID_PeriodElapsedCallback();
          claw_flip_motor.TIM_PID_PeriodElapsedCallback();
          // CAN 总线集中发送。
          
// 达妙电机发送。
         // CAN2 0x201/0x202 已用于升降台。
         // arm_m3508.TIM_PID_PeriodElapsedCallback();
          // arm_m2006_1.TIM_PID_PeriodElapsedCallback();
          // arm_m2006_2.TIM_PID_PeriodElapsedCallback();
          arm_dm_1.TIM_Send_PeriodElapsedCallback();
          arm_dm_2.TIM_Send_PeriodElapsedCallback();

          TIM_CAN_PeriodElapsedCallback();

          // 达妙电机的掉线检测 (大约100ms执行一次)
          static uint32_t dm_alive_counter = 0;
          if (++dm_alive_counter >= 100) {
              dm_alive_counter = 0;
            arm_dm_1.TIM_Alive_PeriodElapsedCallback();
            arm_dm_2.TIM_Alive_PeriodElapsedCallback();
          }
          motion_counter++;
        }

if (ir8_received_123)
{
    ir8_received_123 = 0;

    char ir8_rx_debug_str[96];
    int ir8_rx_debug_len = snprintf(
        ir8_rx_debug_str,
        sizeof(ir8_rx_debug_str),
        "[IR8 RX] 123, rx_count=%lu, byte_count=%lu, tick=%lu\r\n",
        (unsigned long)ir8_received_123_count,
        (unsigned long)ir8_rx_byte_count,
        (unsigned long)HAL_GetTick()
    );
    USART2_DebugSend(ir8_rx_debug_str, sizeof(ir8_rx_debug_str), ir8_rx_debug_len);
}

static uint32_t last_ir_tx_tick = 0;
static uint32_t ir8_tx_count = 0;
static uint8_t ir8_tx_timing_ready = 0;

if (ir8_tx_timing_ready == 0U)
{
    const uint32_t ir8_board_id = IR8_GetBoardId();
    const uint32_t ir8_tx_offset_ms = 100U + (ir8_board_id % 400U);

    ir8_tx_timing_ready = 1U;
    last_ir_tx_tick = HAL_GetTick() + ir8_tx_offset_ms - 1000U;
}

if (HAL_GetTick() - last_ir_tx_tick >= 1000)
{
    last_ir_tx_tick = HAL_GetTick();

    const uint8_t ir_msg[] = {'1', '2', '3'};
    HAL_StatusTypeDef ir8_tx_status = HAL_UART_Transmit(&huart8, (uint8_t *)ir_msg, 3, 20);
    if (ir8_tx_status == HAL_OK)
    {
        ir8_tx_count++;
    }

    char ir8_tx_debug_str[96];
    int ir8_tx_debug_len = snprintf(
        ir8_tx_debug_str,
        sizeof(ir8_tx_debug_str),
        "[IR8 TX] 123, tx_count=%lu, status=%d, tick=%lu\r\n",
        (unsigned long)ir8_tx_count,
        (int)ir8_tx_status,
        (unsigned long)HAL_GetTick()
    );
    USART2_DebugSend(ir8_tx_debug_str, sizeof(ir8_tx_debug_str), ir8_tx_debug_len);
}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
