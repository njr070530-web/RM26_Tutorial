//
// Created by cosmosmount on 2025/9/10.
//

#include "bsp_dwt.hpp"
#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "pid.hpp"
#include "crc.hpp"
#include "math.hpp"
#include "filter.hpp"
#include "magicmsgs.hpp"
#include "ServiceIMU.hpp"
#include "ServiceRemoter.hpp"
#include "GM6020.hpp"
#include "ServiceMotor.hpp"


using namespace Filter;
using namespace Numeric;

TX_THREAD GimbalThread;
uint8_t GimbalThreadStack[4096] = {0};

extern TX_SEMAPHORE IMUThreadSem;
extern om_topic_t* ins_topic;
extern om_topic_t* remoter_topic;



float K_ff=10;


//this if for debug
msg_remoter_t debug_dr16;
msg_ins_t debug_ins;
msg_visionrx_t debug_vrx;
ctrl_debug_t yaw_debug;
ctrl_debug_t pitch_debug;

// om_suber_t* ins_sub_debug = om_subscribe(ins_topic);
// om_suber_t* remoter_control_debug=om_subscribe(remoter_topic);
// msg_ins_t ins_debug;
msg_remoter_t rc_debug;
msg_ins_t ins_debug;

float yaw_current_debug;
float pitch_current_debug;

// use the following pid for yaw
PID yaw_pos_pid(20.0f, 0.0f, 100.0f, 10000.0f, 10.0f, PID_POSITION | PID_Derivative_On_Measurement);
PID yaw_spd_pid(1000.0f, 10.0f, 1000.0f, 10000.0f, 100.0f, PID_POSITION);

PID pitch_pos_pid(80.0f, 0.0f, 0.0f, 10000.0f, 0.0f, PID_POSITION);
PID pitch_spd_pid(1000.0f, 0.0f, 200.0f, 10000.0f, 0.0f, PID_POSITION);
GM6020 &YawMotor=ServiceMotors::Instance()->YawMotor;
GM6020 &PitchMotor=ServiceMotors::Instance()->PitchMotor;

float yaw_pos_fdb_debug;
float yaw_spd_fdb_debug;
float yaw_pos_ref_debug;
float yaw_spd_ref_debug;

float pit_pos_fdb_debug;
float pit_spd_fdb_debug;
float pit_pos_ref_debug;
float pit_spd_ref_debug;


double yaw_signal_step = 0.82f;
double pitch_signal_T = 0.2f;

static double signal(double t)
{
    constexpr double T = 0.6;           // 周期

    constexpr double buffer_ratio = 1.0 / 5.0;
    constexpr double rise_ratio = 4.0 / 5.0;

    const double t_mod = std::fmod(t, T);

    constexpr double buffer_time = buffer_ratio * T;
    constexpr double rise_time = rise_ratio * T;

    if (t_mod < buffer_time)
    {
        // 前1/5缓冲段
        return 0.0;
    }
    else if (t_mod < T)
    {
        double step_value = yaw_signal_step;

        const double rise_t = t_mod - buffer_time;
        const double progress = rise_t / rise_time; // 0 ~ 1

        const double smooth = (1 - std::cos(Numeric::Pi * progress)) / 2.0;

        return step_value * smooth;
    }
    else
    {
        return 0.0;
    }
}

static double tri_signal(double t)
{
    constexpr double T = 0.6;           // 周期

    constexpr double buffer_ratio = 1.0 / 5.0;
    constexpr double rise_ratio = 4.0 / 5.0;

    const double t_mod = std::fmod(t, T);

    constexpr double buffer_time = buffer_ratio * T;
    constexpr double rise_time = rise_ratio * T;

    if (t_mod < buffer_time)
    {
        // 前1/5缓冲段
        return 0.0;
    }
    else if (t_mod < T)
    {
        double step_value = yaw_signal_step;
        // 后4/5 线性上升段（三角波）
        const double rise_t = t_mod - buffer_time;
        const double progress = rise_t / rise_time; // 0 ~ 1

        // 线性
        const double smooth = progress;

        return step_value * smooth;
    }
    else
    {
        return 0.0;
    }
}

static double sin_signal(double t)
{
    const double amplitude = 0.03;  // 振幅（小幅度）
    double T = pitch_signal_T;          // 周期（快速振动）
    const double omega = 2.0 * M_PI / T; // 角频率 ω = 2π / T

    return amplitude * std::sin(omega * t);
}



[[noreturn]] void GimbalThreadFun(ULONG initial_input)
{
    DJIMotorHandler* DJIMotorhandler = DJIMotorHandler::Instance();
    UNUSED(initial_input);
    // om_topic_t *gimbal_topic = om_config_topic(nullptr, "ca", "gimbalctrl", sizeof(msg_gimbal_ctrl_t));


    constexpr float DEG2RAD = Numeric::Pi / 180.0f;

    //@TODO: what to input?
    // msg_gimbal_ctrl_t msg_gimbal{};

    float target_yaw =0;
    float target_pit =0;




    //@TODO:subscribe what topic?
    if (ins_topic == nullptr) {
        __BKPT(0);   // 让程序停在这里
    }
    if (remoter_topic == nullptr) {
        __BKPT(1);
    }
    om_suber_t* ins_sub = om_subscribe(ins_topic);
    om_suber_t* remoter_control=om_subscribe(remoter_topic);
    msg_ins_t ins;
    msg_remoter_t rc;


    for (;;)
    {

        //@TODO: subscribe message taken out and memcpy()
        om_suber_export(remoter_control,&rc,false);
        memcpy(&rc_debug,&rc,sizeof(rc));
        float left_x=-rc.left_x;
        float left_y=-rc.left_y;

        //only if IMU have send data, start gimbal control
        if (tx_semaphore_get(&IMUThreadSem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            om_suber_export(ins_sub,&ins,false);
            memcpy(&ins_debug,&ins,sizeof(ins));
            float yaw_spd_fdb=ins.gyro_y;
            float yaw_po_fdb=ins.total_yaw*DEG2RAD;
            float pitch_spd_fdb=ins.gyro_p;
            float pitch_po_fdb=ins.pitch*DEG2RAD;



                //@TODO: if remoter is offline or in relax state, set all motors' currentSet to 0, protection
                if (rc.offline==true || rc.ctrl_sw==N2R ||rc.ctrl_sw==Relax)
                {
                    // msg_gimbal.pitch_speed=0;
                    // msg_gimbal.yaw_speed=0;
                    ServiceMotors::Instance()->YawMotor.currentSet=0;
                    ServiceMotors::Instance()->PitchMotor.currentSet=0;

                    DJIMotorhandler->sendControlData();
                    tx_thread_sleep(1);
                    continue;

                    // PitchMotor.positionPid.UpdateResult();
                    // PitchMotor.speedPid.UpdateResult();
                }

                //@TODO: otherwise, use remote controller to control pitch and yaw
                //@warning!!!!!!!!!!!!!!!: remember to check is position in degree or in radius, in ref and fdb.  VERY DANGEROUS!!!!!!!!!!!!!
                else
                {
                    if (rc.ctrl_sw==Normal)
                    {
                        // msg_gimbal.yaw_mode = SPD;

                        target_yaw=left_x* 0.005;//position是degree
                        target_pit=left_y* 0.005;//0.2 -0.37

                        // ServiceMotors::Instance()->PitchMotor.positionPid;
                        yaw_pos_pid.ref+=target_yaw;
                        yaw_pos_pid.fdb=yaw_po_fdb;
                        yaw_pos_pid.UpdateResult();
                        yaw_spd_pid.ref= yaw_pos_pid.result;
                        yaw_spd_pid.fdb=yaw_spd_fdb;
                        yaw_spd_pid.UpdateResult();

                        pitch_pos_pid.ref+=target_pit;
                        pitch_pos_pid.ref = FloatConstrain(pitch_pos_pid.ref, -0.5f, 0.15f);
                        pitch_pos_pid.fdb=pitch_po_fdb;
                        pitch_pos_pid.UpdateResult();
                        pitch_spd_pid.ref= pitch_pos_pid.result;
                        pitch_spd_pid.fdb=pitch_spd_fdb;
                        pitch_spd_pid.UpdateResult();
                        // msg_gimbal.pitch_speed=YawMotor.speedPid.result;
                        memcpy(&yaw_pos_fdb_debug,&yaw_po_fdb,sizeof(yaw_po_fdb));
                        memcpy(&yaw_pos_ref_debug,&yaw_pos_pid.ref,sizeof(yaw_pos_pid.ref));

                        memcpy(&yaw_spd_fdb_debug,&yaw_spd_fdb,sizeof(yaw_spd_fdb));
                        memcpy(&yaw_spd_ref_debug,&yaw_spd_pid.ref,sizeof(yaw_spd_pid.ref));


                        memcpy(&pit_pos_fdb_debug,&pitch_po_fdb,sizeof(pitch_po_fdb));
                        memcpy(&pit_pos_ref_debug,&pitch_pos_pid.ref,sizeof(pitch_pos_pid.ref));

                        memcpy(&pit_spd_fdb_debug,&pitch_spd_fdb,sizeof(pitch_spd_fdb));
                        memcpy(&pit_spd_ref_debug,&pitch_spd_pid.ref,sizeof(pitch_spd_pid.ref));

                    }

                }
            ServiceMotors::Instance()->YawMotor.currentSet   = yaw_spd_pid.result;
            ServiceMotors::Instance()->PitchMotor.currentSet = pitch_spd_pid.result-K_ff*cosf(pitch_po_fdb);
            yaw_current_debug=yaw_spd_pid.result;
            pitch_current_debug=pitch_spd_pid.result;

            }
        tx_thread_sleep(1);
        }
    }
