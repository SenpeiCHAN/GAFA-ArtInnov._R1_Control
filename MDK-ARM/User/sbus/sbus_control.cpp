#include "sbus_control.h"
#include "sbus.h" 
#include <stdio.h> 
#include <string.h> 
#include <math.h>


// 全局变量定义
ProcessedSbusValue_t G_SbusValue = {0};


#define SBUS_RAW_MIN 192
#define SBUS_RAW_MID 992
#define SBUS_RAW_MAX 1792



// 将 SBUS 原始值按中位对称映射到 -ctrl_range ~ +ctrl_range。
static float map_stick_to_ctrl(uint16_t raw_value, uint16_t raw_min, uint16_t raw_mid, uint16_t raw_max, float ctrl_range) {
    float raw_range_upper = (float)(raw_max - raw_mid);
    float raw_range_lower = (float)(raw_mid - raw_min);
    float ctrl_value = 0.0f;

    if (raw_value >= raw_mid) {
        if (raw_value >= raw_max) {
            ctrl_value = ctrl_range;
        } else {
            float normalized = ((float)(raw_value - raw_mid)) / raw_range_upper;
            ctrl_value = normalized * ctrl_range;
        }
    } else {
        if (raw_value <= raw_min) {
            ctrl_value = -ctrl_range;
        } else {
            float normalized = ((float)(raw_value - raw_mid)) / raw_range_lower;
            ctrl_value = normalized * ctrl_range;
        }
    }
    return ctrl_value;
}

static float apply_chassis_curve(float ctrl_value)
{
    if (ctrl_value > CTRL_RANGE_MAX) ctrl_value = CTRL_RANGE_MAX;
    if (ctrl_value < -CTRL_RANGE_MAX) ctrl_value = -CTRL_RANGE_MAX;

    float normalized = ctrl_value / CTRL_RANGE_MAX;
    float abs_normalized = fabs(normalized);
    const float expo = 0.65f;
    float curved = ((1.0f - expo) * abs_normalized) + (expo * abs_normalized * abs_normalized * abs_normalized);

    return ((normalized >= 0.0f) ? curved : -curved) * CTRL_RANGE_MAX;
}



void SbusControl_Init(void) {
    // 初始化处理后的 SBUS 值。
    G_SbusValue.Vx = 0.0f;
    G_SbusValue.Vy = 0.0f;
    G_SbusValue.W = 0.0f;
    G_SbusValue.LiftPercent = 0.0f;
    G_SbusValue.CmdEnable = 0;
    G_SbusValue.Aux1 = 0.0f;
    G_SbusValue.Aux2 = -100.0f;
    G_SbusValue.CmdType = CMD_STOP;
    G_SbusValue.Arm_Axis5 = 0.0f;
    G_SbusValue.Raw_CH7 = 0;
}

void SbusControl_ProcessData(void) {
    // 这里保留接口形式，底盘实际加速度限制在 main.cpp 的 omni_move 中完成。
    float alpha = 1.0f; 
    static ProcessedSbusValue_t G_SbusValue_Last = {0};

    if (!SBUS_CH.ConnectState) {
        G_SbusValue.Vx = 0.0f;
        G_SbusValue.Vy = 0.0f;
        G_SbusValue.W = 0.0f;
        G_SbusValue.LiftPercent = 0.0f;
        G_SbusValue.RobotMode = 0;
        G_SbusValue.Arm_Axis3 = 0.0f;
        G_SbusValue.Arm_Axis4 = 0.0f;
        G_SbusValue.Arm_Axis5 = 0.0f;
        G_SbusValue.Aux2 = -100.0f; // 断连时强制失能
        return; 
    }

    // 直接读取 SBUS_CH 中的最新通道值。
    uint16_t raw_ch1 = SBUS_CH.CH1;   
    uint16_t raw_ch2 = SBUS_CH.CH2;   
    uint16_t raw_ch3 = SBUS_CH.CH3;   
    uint16_t raw_ch4 = SBUS_CH.CH4;
    uint16_t raw_ch5 = SBUS_CH.CH5;       // CH5: 吸夹翻转
    uint16_t raw_aux2 = SBUS_CH.CH10;     // CH10: 整车使能
    uint16_t raw_arm_axis3 = SBUS_CH.CH9; // CH9: 龙门架平动两点
    uint16_t raw_ch7 = SBUS_CH.CH7;
    uint16_t raw_ch11 = SBUS_CH.CH11;     // CH11: 吸夹翻转角度微调输入
    const uint16_t ch5_flip_on_threshold = 1192U; // 992 中位以上约 200 作为开启阈值
    const uint16_t ch9_extend_retract_threshold = 1400U;
    G_SbusValue.Raw_CH7 = raw_ch7;
    // CH10 使能通道，任何模式下都生效。
    float mapped_aux2 = map_stick_to_ctrl(raw_aux2, SBUS_RAW_MIN, SBUS_RAW_MID, SBUS_RAW_MAX, CTRL_RANGE_MAX);
    G_SbusValue.Aux2 = mapped_aux2;

    G_SbusValue.RobotMode = 0; // 当前固定为底盘与机构控制模式。

// CH3 升降台速度：中位保持，向上/向下对称。
#define CH3_MIN   192
#define CH3_MID   992
#define CH3_MAX   1792

    float lift_cmd = map_stick_to_ctrl(raw_ch3, CH3_MIN, CH3_MID, CH3_MAX, CTRL_RANGE_MAX);
    if (fabs(lift_cmd) < 5.0f)
    {
        lift_cmd = 0.0f;
    }
    G_SbusValue.LiftPercent = lift_cmd;

    // CH11 是速度型微调量：中位输出 0，向上/向下分别让吸夹翻转目标角继续增减。
    float mapped_a5 = map_stick_to_ctrl(raw_ch11, SBUS_RAW_MIN, SBUS_RAW_MID, SBUS_RAW_MAX, CTRL_RANGE_MAX);
    if (fabs(mapped_a5) < 8.0f) {
        mapped_a5 = 0.0f;
    }
    G_SbusValue.Arm_Axis5 = mapped_a5;


    // 当前只使用 RobotMode 0；保留模式分支兼容旧机械臂代码。
    if (G_SbusValue.RobotMode == 0) {
        // 底盘摇杆加曲线，保留小幅拨杆的细腻控制。
        float mapped_vx = apply_chassis_curve(map_stick_to_ctrl(raw_ch2, SBUS_RAW_MIN, SBUS_RAW_MID, SBUS_RAW_MAX, CTRL_RANGE_MAX));
        if (fabs(mapped_vx) < 8.0f) mapped_vx = 0.0f;
        G_SbusValue.Vx = alpha * mapped_vx + (1.0f - alpha) * G_SbusValue_Last.Vx;

        float mapped_vy = apply_chassis_curve(map_stick_to_ctrl(raw_ch1, SBUS_RAW_MIN, SBUS_RAW_MID, SBUS_RAW_MAX, CTRL_RANGE_MAX));
        if (fabs(mapped_vy) < 8.0f) mapped_vy = 0.0f;
        G_SbusValue.Vy = alpha * mapped_vy + (1.0f - alpha) * G_SbusValue_Last.Vy;

        float mapped_w = apply_chassis_curve(map_stick_to_ctrl(raw_ch4, SBUS_RAW_MIN, SBUS_RAW_MID, SBUS_RAW_MAX, CTRL_RANGE_MAX));
        if (fabs(mapped_w) < 8.0f) mapped_w = 0.0f;
        G_SbusValue.W = alpha * mapped_w + (1.0f - alpha) * G_SbusValue_Last.W;

        // CH9/CH5 机构开关。
        G_SbusValue.Arm_Axis1 = 0.0f;
        G_SbusValue.Arm_Axis2 = 0.0f;

        float mapped_a3 = (raw_arm_axis3 > ch9_extend_retract_threshold) ? CTRL_RANGE_MAX : 0.0f;
        G_SbusValue.Arm_Axis3 = mapped_a3;

        float mapped_a4 = (raw_ch5 > ch5_flip_on_threshold) ? CTRL_RANGE_MAX : 0.0f;
        G_SbusValue.Arm_Axis4 = mapped_a4;

    } else {
        // 备用机械臂模式：底盘强制刹停。
        G_SbusValue.Vx = 0.0f;
        G_SbusValue.Vy = 0.0f;
        G_SbusValue.W  = 0.0f;
        // LiftPercent 保持上一帧，避免模式切换时升降台目标突变。

        // 备用机械臂轴。
        float mapped_a1 = map_stick_to_ctrl(raw_ch4, SBUS_RAW_MIN, SBUS_RAW_MID, SBUS_RAW_MAX, CTRL_RANGE_MAX);
        if (fabs(mapped_a1) < 10.0f) mapped_a1 = 0.0f;
        G_SbusValue.Arm_Axis1 = mapped_a1;

        float mapped_a2 = map_stick_to_ctrl(raw_ch1, SBUS_RAW_MIN, SBUS_RAW_MID, SBUS_RAW_MAX, CTRL_RANGE_MAX);
        if (fabs(mapped_a2) < 8.0f) mapped_a2 = 0.0f;
        G_SbusValue.Arm_Axis2 = mapped_a2;

        // CH9 龙门架平动两点。
        float mapped_a3 = (raw_arm_axis3 > ch9_extend_retract_threshold) ? CTRL_RANGE_MAX : 0.0f;
        G_SbusValue.Arm_Axis3 = mapped_a3;

        // CH5 吸夹翻转。
        float mapped_a4 = (raw_ch5 > ch5_flip_on_threshold) ? CTRL_RANGE_MAX : 0.0f;
        G_SbusValue.Arm_Axis4 = mapped_a4;

        
    }

    // 保存底盘通道，供下一帧滤波兼容使用。
    G_SbusValue_Last.Vx = G_SbusValue.Vx;
    G_SbusValue_Last.Vy = G_SbusValue.Vy;
    G_SbusValue_Last.W = G_SbusValue.W;
}
