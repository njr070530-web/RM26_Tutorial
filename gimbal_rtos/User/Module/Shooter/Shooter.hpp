#pragma once
// #ifndef SHOOTER_HPP
// #define SHOOTER_HPP
#include "magicmsgs.hpp"
#include "cstdint"
#include "drv_math.h"
#include "DJIMotor.hpp"

#include "M3508.hpp"
#include "M2006.hpp"


enum ShooterState_e
{
    SHOOTER_STOP = 0,        // 停止/安全模式
    SHOOTER_PREPARE,         // 摩擦轮加速中
    SHOOTER_READY,           // 摩擦轮达标，等待开火
    SHOOTER_SHOOT_ONE,       // 正在执行单发动作
    SHOOTER_JAMMED,          // 卡弹处理
};

class Shooter{
public:
    Shooter();
    void Init();
    void Update(const msg_remoter_t* rc);
private:
    ShooterState_e current_state;
    ShooterState_e next_state;

    float friction_speed_target;

    float trigger_target_angle;
    bool fire_rc_last;

    // --- 内部功能函数 ---
    void check_jamming(); // 检测是否卡弹
    bool is_friction_ready(); // 摩擦轮是否转速达标

    float my_total_angle;
    float last_raw_angle;

    void update_total_angle();

    int jam_check_timer;
    int reversing_timer;

    const int JAM_TRIGGER_TIME =200;
    const int REVERSE_TIME=300;
    const float BLOCK_SPEED_THRESHOLD=50.0f;


};