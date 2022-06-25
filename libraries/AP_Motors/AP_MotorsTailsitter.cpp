/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsTailsitter.cpp - ArduCopter motors library for tailsitters and bicopters
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTailsitter.h"
#include <GCS_MAVLink/GCS.h>

#include <math.h>


//控制分配矩阵定义及赋值

//定义两个旋翼转速平方分别与各自的两个余弦值相乘
float motor1_2_sin1;
float motor1_2_cos1;
float motor2_2_sin2;
float motor2_2_cos2;

//定义机体坐标系下旋翼在OXb与OZb轴上产生的力
float force_OXb;
float force_OZb;

//定义机体坐标系下旋翼在三个轴上产生的力矩
float moment_OXb;
float moment_OYb;
float moment_OZb;

//定义旋翼推力系数
float motor_Ct;

//定义当前俯仰角
float Cont_dist_pitch_cd;

Matrix3f aaa1;
    float aa1;
    float bb1;
    float cc1;
    float dddd1;//原 ahrs_view->pitch_sensor，但是感觉又不太一样，因为反映出来的效果不同，现在无法证明因为无法直接将ahrs_view->pitch_sensor在地面站实时显示。
    Matrix3f board_rotation1 {0, 0, -1,
                                0, 1, 0,
                                1, 0, 0};

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500

// init
void AP_MotorsTailsitter::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // setup default motor and servo mappings
    uint8_t chan;

    // right throttle defaults to servo output 1
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, CH_1);
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        motor_enabled[chan] = true;
    }

    // left throttle defaults to servo output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, CH_2);
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        motor_enabled[chan] = true;
    }

    // right servo defaults to servo output 3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, CH_3);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, SERVO_OUTPUT_RANGE);

    // left servo defaults to servo output 4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, CH_4);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, SERVO_OUTPUT_RANGE);

    _mav_type = MAV_TYPE_COAXIAL;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_TAILSITTER);
}


/// Constructor
AP_MotorsTailsitter::AP_MotorsTailsitter(uint16_t loop_rate, uint16_t speed_hz) :
    AP_MotorsMulticopter(loop_rate, speed_hz)
{
    set_update_rate(speed_hz);
}


// set update rate to motors - a value in hertz
void AP_MotorsTailsitter::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleLeft, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleRight, speed_hz);
}

void AP_MotorsTailsitter::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            _actuator[0] = 0.0f;
            _actuator[1] = 0.0f;
            _actuator[2] = 0.0f;
            break;
        case SpoolState::GROUND_IDLE:
            set_actuator_with_slew(_actuator[0], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[2], actuator_spin_up_to_ground_idle());
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            set_actuator_with_slew(_actuator[0], thrust_to_actuator(_thrust_left));
            set_actuator_with_slew(_actuator[1], thrust_to_actuator(_thrust_right));
            set_actuator_with_slew(_actuator[2], thrust_to_actuator(_throttle));
            break;
    }

    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(_actuator[0]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(_actuator[1]));

    // use set scaled to allow a different PWM range on plane forward throttle, throttle range is 0 to 100
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _actuator[2]*100);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, _tilt_left*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, _tilt_right*SERVO_OUTPUT_RANGE);

}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsTailsitter::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        motor_mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        motor_mask |= 1U << chan;
    }

    // add parent's mask
    motor_mask |= AP_MotorsMulticopter::get_motor_mask();

    return motor_mask;
}

// calculate outputs to the motors
void AP_MotorsTailsitter::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   thrust_max;                 // highest motor value
    float   thr_adj = 0.0f;             // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = _pitch_in + _pitch_in_ff;  //这里是否能找到期望俯仰角相关的线索
    yaw_thrust = _yaw_in + _yaw_in_ff;
    throttle_thrust = get_throttle() * compensation_gain;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }
    
 
    //aaa = ahrs.get_rotation_body_to_ned() * board_rotation; //此处为什么不能调用呢？因为.h文件当中没有 AP_AHRS_View *ahrs_view; 这样的声明或者定义 还有就是没有 #include "AP_AHRS/AP_AHRS_View.h" 这样的头文件声明
    aaa1 = ahrs_view->get_rotation_body_to_ned() * board_rotation1; //之后这种调用
    aaa1.to_euler(&aa1, &bb1, &cc1);
    dddd1 = degrees(bb1) * 100.0f;
    
    //为当前俯仰角赋值
    Cont_dist_pitch_cd = dddd1;

    //赋值 机体坐标系下旋翼在OXb与OZb轴上产生的力
    force_OXb = -1.0 * throttle_thrust * sinf(radians((Cont_dist_pitch_cd) / 100.0f));
    force_OZb = throttle_thrust * cosf(radians((Cont_dist_pitch_cd) / 100.0f));  

    //赋值 机体坐标系下旋翼在三个轴上产生的力矩
    moment_OXb = roll_thrust;
    moment_OYb = pitch_thrust;
    moment_OZb = yaw_thrust;

    motor1_2_sin1 = -1.0 * force_OXb + moment_OYb - moment_OZb;
    motor1_2_cos1 = -1.0 * force_OZb + moment_OXb;
    motor2_2_sin2 = -1.0 * force_OXb + moment_OYb + moment_OZb;
    motor2_2_cos2 = -1.0 * force_OZb - moment_OXb;

    //为电机推力系数赋值
    motor_Ct = 0.00000874; //因为后面要归一化处理，这个系数就先不乘试试看
    
    // calculate left and right throttle outputs
    //尾座式的控制分配矩阵两个电机的推力在这两句 0625到了就是看看是否又要进行归一化处理。
    _thrust_left  = sqrtf((motor1_2_sin1 * motor1_2_sin1) + (motor1_2_cos1 * motor1_2_cos1));
    _thrust_right = sqrtf((motor2_2_sin2 * motor2_2_sin2) + (motor2_2_cos2 * motor2_2_cos2));

    // if max thrust is more than one reduce average throttle
    thrust_max = MAX(_thrust_right,_thrust_left);
    if (thrust_max > 1.0f) {
        thr_adj = 1.0f - thrust_max;
        limit.throttle_upper = true;
        limit.roll = true;
        limit.pitch = true;
    }

    // Add adjustment to reduce average throttle
    _thrust_left  = constrain_float(_thrust_left  + thr_adj, 0.0f, 1.0f); //这里应该就是将推力进行归一化处理的位置
    _thrust_right = constrain_float(_thrust_right + thr_adj, 0.0f, 1.0f);
    _throttle = throttle_thrust + thr_adj;
    // compensation_gain can never be zero
    _throttle_out = _throttle / compensation_gain;

    // thrust vectoring
    //尾座式的控制分配矩阵两个倾转舵机的角度在这两句 0625到了就是看看是否又要进行归一化处理。
    _tilt_left  = constrain_float(degrees(atanf((motor1_2_sin1)/(motor1_2_cos1))) , -90.0f, 90.0f); //degrees(atanf((motor1_2_sin1)/(motor1_2_cos1)))求出的是倾转舵机应当偏转的角度，再进行归一化处理
    _tilt_right = constrain_float(degrees(atanf((motor2_2_sin2)/(motor2_2_cos2))) , -90.0f, 90.0f);
    
    //_tilt_left  = degrees(atanf((motor1_2_sin1)/(motor1_2_cos1))) * 100.0f; //有疑问，明天再看，需要再缩放
    //_tilt_right = degrees(atanf((motor2_2_sin2)/(motor2_2_cos2))) * 100.0f;
    
    
    /* 原语句
    // calculate left and right throttle outputs
    //尾座式的控制分配矩阵两个电机的推力在这两句
    _thrust_left  = throttle_thrust + roll_thrust * 0.5f;
    _thrust_right = throttle_thrust - roll_thrust * 0.5f;

    // if max thrust is more than one reduce average throttle
    thrust_max = MAX(_thrust_right,_thrust_left);
    if (thrust_max > 1.0f) {
        thr_adj = 1.0f - thrust_max;
        limit.throttle_upper = true;
        limit.roll = true;
        limit.pitch = true;
    }

    // Add adjustment to reduce average throttle
    _thrust_left  = constrain_float(_thrust_left  + thr_adj, 0.0f, 1.0f);
    _thrust_right = constrain_float(_thrust_right + thr_adj, 0.0f, 1.0f);
    _throttle = throttle_thrust + thr_adj;
    // compensation_gain can never be zero
    _throttle_out = _throttle / compensation_gain;

    // thrust vectoring
    //尾座式的控制分配矩阵两个倾转舵机的角度在这两句
    _tilt_left  = pitch_thrust - yaw_thrust;
    _tilt_right = pitch_thrust + yaw_thrust;
    */
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTailsitter::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // right throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        case 2:
            // right tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRight, pwm);
            break;
        case 3:
            // left throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            break;
        case 4:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorLeft, pwm);
            break;
        default:
            // do nothing
            break;
    }
}
