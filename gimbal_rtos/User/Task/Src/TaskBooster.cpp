#include "TaskBooster.hpp"
#include "M3508.hpp"
#include "bsp_dwt.hpp"

#include "magicmsgs.hpp"
#include "main.h"
#include "tx_api.h"
#include "ServiceShooter.hpp"
#include "Shooter.hpp"

TX_THREAD ShooterThread;
msg_remoter_t debug_dr16_shooter;

uint8_t ShooterThreadStack[4096];
[[noreturn]] void ShooterThreadFun(ULONG initial_input);

extern om_topic_t* remoter_topic;
#define TX_NAME(s) const_cast<CHAR*>(s)
void TaskBooster()
{
        tx_thread_create(&ShooterThread, TX_NAME("ShooterThread"),
        ShooterThreadFun, 0x1234, ShooterThreadStack, sizeof(ShooterThreadStack),
        6, 6, TX_NO_TIME_SLICE, TX_AUTO_START);

}
[[noreturn]] void ShooterThreadFun(ULONG initial_input)
{   
    UNUSED(initial_input);
    DJIMotorHandler* DJIMotorhandler = DJIMotorHandler::Instance();
    auto hw = ServiceShooter::Instance();
    hw->MotorRegister();

    hw->FrictionL.controlMode = M3508::SPD_MODE;
    hw->FrictionL.speedPid.kp = 10.0f; 
    hw->FrictionL.speedPid.ki = 0.1f;
    hw->FrictionL.speedPid.maxOut = 16000;

    hw->FrictionR.controlMode = M3508::SPD_MODE;
    hw->FrictionR.speedPid = hw->FrictionL.speedPid;

    hw->Trigger.controlMode = M2006::POS_MODE; 
    hw->Trigger.positionPid.kp = 10.0f; 
    hw->Trigger.speedPid.kp = 10.0f;    
    hw->Trigger.speedPid.maxOut = 10000;

    tx_thread_sleep(100); 
    hw->Trigger.positionSet = hw->Trigger.motorFeedback.positionFdb;
    // static Shooter Shooter_fsm;
    // //电机初始化
    // //引入FSM
    // Shooter_fsm.Init();
    // //订阅remoter
    bool fire_rc_last = false;


    om_suber_t *remoter_control = om_subscribe(remoter_topic);
    msg_remoter_t rc={};

    for (;;)
    {
        om_suber_export(remoter_control,&rc,false);
        memcpy(&debug_dr16_shooter,&rc,sizeof(rc));
        // Shooter_fsm.Update(&rc);
        // //发送控制指令给电机
        // DJIMotorhandler->sendControlData();
        // tx_thread_sleep(1);
        
        if(rc.offline==true || rc.shoot_sw==Closed)
        {
             hw->FrictionL.speedSet=0;
             hw->FrictionR.speedSet=0;
             hw->Trigger.speedSet=0;
        }
        else if(rc.shoot_sw==Warm)
        {
            hw->Trigger.controlMode = M2006::POS_MODE;
            hw->FrictionL.currentSet=2000;
            hw->FrictionR.speedSet=-2000;
        }
        else if(rc.shoot_sw==Fire && !fire_rc_last)
        {
            hw->FrictionL.speedSet=2000;
            hw->FrictionR.speedSet=-2000;  
            hw->Trigger.positionSet+=1620;
            if (!fire_rc_last) 
            {
                // 直接修改 positionSet，让底层 PID 去追
                hw->Trigger.positionSet += 1620.0f; 
                fire_rc_last = true;
            }
            
        }
        if (rc.shoot_sw != Fire)
        {
            fire_rc_last = false;
        }
        hw->FrictionL.setOutput();
        hw->FrictionR.setOutput();
        hw->Trigger.setOutput();

        // DJIMotorhandler->sendControlData();

        tx_thread_sleep(1);
    }
}
