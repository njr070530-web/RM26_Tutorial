#pragma once
#ifndef SERVICESHOOTER_HPP
#define SERVICESHOOTER_HPP

#include "M3508.hpp"
#include "M2006.hpp"

#include "DJIMotorHandler.hpp"

class ServiceShooter
{
public:
    // --- 1. 定义电机对象 ---
    // 摩擦轮使用 M3508
    M3508 FrictionL; // ID 1
    M3508 FrictionR; // ID 2
    
    // 拨弹盘使用 M2006
    M2006 Trigger;   // ID 3

    // --- 2. 核心功能函数 ---
    void MotorRegister();       // 注册电机ID和CAN句柄
    void SetModeAndPidParam();  // 设定PID参数和控制模式
    void AllMotorSetOutput();   // 发送CAN控制帧（放在循环最后调用）

    // --- 3. 单例模式获取接口 ---
    static ServiceShooter *Instance()
    {
        static ServiceShooter instance;
        return &instance;
    }
};

#endif