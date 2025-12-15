#include "DJIMotor.hpp"
#include "main.h"
#include "ServiceShooter.hpp"

// Reference the single shared instance defined in another translation unit
extern DJIMotorHandler* DJIMotorhandler;


void ServiceShooter::MotorRegister() {
    // 假设 Shooter 在 CAN1 (如果是 CAN2，请把 &hcan1 改为 &hcan2)
    // M3508 ID 1 -> 反馈 ID 0x201
    DJIMotorhandler->registerMotor(&FrictionL, &hcan1, 0x202);
    FrictionL.gearBox = GearBox_None; // 摩擦轮直连

    // M3508 ID 2 -> 反馈 ID 0x202
    DJIMotorhandler->registerMotor(&FrictionR, &hcan1, 0x203);
    FrictionR.gearBox = GearBox_None;

    // M2006 ID 3 -> 反馈 ID 0x203
    DJIMotorhandler->registerMotor(&Trigger, &hcan1, 0x201);
    // M2006 内部减速比 36:1，如果你的库支持设置减速比自动换算角度，可以在这里设
    Trigger.gearBox = GearBox_M2006; 
}
void ServiceShooter::SetModeAndPidParam()
{
// --- 1. 摩擦轮 (速度环) ---
    // 左轮
    FrictionL.controlMode = M3508::SPD_MODE; // 确保你的类有这个枚举
    FrictionL.speedPid.kp = 15.0f;      // M3508 负载惯量大，KP 给大点
    FrictionL.speedPid.ki = 0.5f;
    FrictionL.speedPid.kd = 0.0f;
    FrictionL.speedPid.maxOut = 16000; // M3508 最大电流 16384

    // 右轮 (参数通常一致)
    FrictionR.controlMode = M3508::SPD_MODE;
    FrictionR.speedPid = FrictionL.speedPid; 

    // --- 2. 拨弹电机 (位置环/双环) ---
    // 必须启用 ANGLE_LOOP 才能实现精准的“转45度”
    Trigger.controlMode = M2006::POS_MODE; 
    
    // 外环：角度环 (控制到位精度)
    Trigger.positionPid.kp = 12.0f; 
    Trigger.positionPid.ki = 0.0f;
    Trigger.positionPid.kd = 0.5f;
    Trigger.positionPid.maxOut = 10000; // 限制传递给内环的最大速度

    // 内环：速度环 (控制动作快慢)
    Trigger.speedPid.kp = 10.0f;
    Trigger.speedPid.ki = 0.1f;
    Trigger.speedPid.maxOut= 10000; // M2006 最大电流 10000
}