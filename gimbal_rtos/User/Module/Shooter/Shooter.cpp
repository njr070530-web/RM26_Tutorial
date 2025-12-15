#include "Shooter.hpp"
#include "magicmsgs.hpp"
#include "math.hpp"
#include "ServiceShooter.hpp"
#include "main.h"

#define ANGLE_PER_BULLET 1620.0f //假设8个空位
#define TRIGGER_ERROR_TOLERANCE 50.0f // 允许的角度误差
#define FRICTION_SPEED_SET 6000.0f
#define SHOOT_POS_KP 15.0f 
#define MAX_TRIGGER_SPEED 10000.0f
Shooter::Shooter() {
    current_state = SHOOTER_STOP;
    friction_speed_target = 0;
    fire_rc_last = false;
    trigger_target_angle = 0;
}
void Shooter::update_total_angle(){
    auto* hw = ServiceShooter::Instance();
    float current_raw=hw->Trigger.motorFeedback.positionFdb;
//现在的位置，应该是一个0-360度的值
    float delta=current_raw - last_raw_angle;
    if (delta < -180.0f) {
        delta += 360.0f;
    }
    // 如果突然从 1 跳到了 359，差值是 +358，说明反向转了一圈
    else if (delta > 180.0f) {
        delta -= 360.0f;
    }
    my_total_angle+=delta;
    last_raw_angle=current_raw;
}


bool Shooter::is_friction_ready() {
    auto hw = ServiceShooter::Instance();
    // 简单判断：左右轮速度误差都在 500 以内
    bool l_ok = fabs(hw->FrictionL.motorFeedback.speedFdb - FRICTION_SPEED_SET) < 500.0f;
    bool r_ok = fabs(hw->FrictionR.motorFeedback.speedFdb + FRICTION_SPEED_SET) < 500.0f; // 右轮通常反转
    return l_ok && r_ok;
}
void Shooter::Init() {
    ServiceShooter::Instance()->MotorRegister();
    ServiceShooter::Instance()->SetModeAndPidParam();

    auto hw = ServiceShooter::Instance();
    last_raw_angle=hw->Trigger.motorFeedback.positionFdb;
    my_total_angle=last_raw_angle;

    trigger_target_angle = my_total_angle;
}



void Shooter::Update(const msg_remoter_t *rc)
{
    auto hw = ServiceShooter::Instance();
    update_total_angle();
switch (current_state){
    case SHOOTER_STOP:
        jam_check_timer=0;
        if(rc->shoot_sw==Warm){
            current_state=SHOOTER_PREPARE;
        }
    break;
    case SHOOTER_PREPARE:
            if(is_friction_ready()){
                current_state=SHOOTER_READY;
            }
            else if(rc->shoot_sw!=Warm){
                current_state=SHOOTER_STOP;
            }
    break;
    case SHOOTER_READY:
        jam_check_timer=0;
        if(rc->shoot_sw==Fire && !fire_rc_last){
            current_state=SHOOTER_SHOOT_ONE;
            trigger_target_angle=my_total_angle + ANGLE_PER_BULLET;
        }
        else if(rc->shoot_sw!=Warm && rc->shoot_sw != Fire){
            current_state=SHOOTER_STOP;
        }
    break;
    case SHOOTER_SHOOT_ONE:
        if(fabs(my_total_angle - trigger_target_angle)<TRIGGER_ERROR_TOLERANCE){
            current_state=SHOOTER_READY;
            jam_check_timer=0;
        }
        if (fabs(my_total_angle - trigger_target_angle) > TRIGGER_ERROR_TOLERANCE && 
                fabs(hw->Trigger.motorFeedback.speedFdb) < BLOCK_SPEED_THRESHOLD) 
            {
                jam_check_timer++; // 累加计时 (假设1ms调一次)
            } else {
                jam_check_timer = 0; // 只要动起来了，就清零
            }
            if (jam_check_timer > JAM_TRIGGER_TIME) {
                current_state = SHOOTER_JAMMED; // 切换到反转状态
                reversing_timer = 0; // 重置反转计时
            }
        if(rc->shoot_sw != Warm && rc->shoot_sw != Fire) {
                current_state = SHOOTER_STOP;
            }
    break;
    case SHOOTER_JAMMED:
            reversing_timer++;
            
            // 反转一定时间后，回去重试
            if (reversing_timer > REVERSE_TIME) {
                current_state = SHOOTER_READY; 
                
                trigger_target_angle = my_total_angle; 
            }
            
            if(rc->shoot_sw != Warm && rc->shoot_sw != Fire) current_state = SHOOTER_STOP;
            break;

    }
fire_rc_last = (rc->shoot_sw == Fire);

float friction_cmd = 0.0f;
    float trigger_speed_cmd = 0.0f; // 最终发给拨盘的速度指令

    switch (current_state) {
        case SHOOTER_STOP:
            friction_cmd = 0.0f;
            trigger_speed_cmd = 0.0f; // 停止时直接给0
            break;

        case SHOOTER_PREPARE:
            friction_cmd = FRICTION_SPEED_SET;
            // 准备阶段：锁死在当前目标位置 (防止有人手拨动)
            {
                float error = trigger_target_angle - my_total_angle;
                trigger_speed_cmd = error * SHOOT_POS_KP;
            }
            break;
        case SHOOTER_READY:
        case SHOOTER_SHOOT_ONE:
            friction_cmd = FRICTION_SPEED_SET;
            {
            // --- 软位置环核心逻辑 ---
            // 1. 计算误差 (Target - Total)
            float pos_error = trigger_target_angle - my_total_angle;
            
            // 2. 比例控制 (P-Control): 误差越大，速度越快
            trigger_speed_cmd = pos_error * SHOOT_POS_KP;
            
            // 3. 限幅 (防止发给电机的电流/速度太大)
            if (trigger_speed_cmd > MAX_TRIGGER_SPEED) trigger_speed_cmd = MAX_TRIGGER_SPEED;
            if (trigger_speed_cmd < -MAX_TRIGGER_SPEED) trigger_speed_cmd = -MAX_TRIGGER_SPEED;
            }
            break;
        // 【新增】反转状态的执行动作
        case SHOOTER_JAMMED:
            friction_cmd = FRICTION_SPEED_SET; // 摩擦轮还要转
            trigger_speed_cmd = -300.0f; 
            break;
    }

    
    // 设置摩擦轮
    hw->FrictionL.speedSet = friction_cmd;
    hw->FrictionR.speedSet = -friction_cmd;


    hw->Trigger.speedSet = trigger_speed_cmd;
}
