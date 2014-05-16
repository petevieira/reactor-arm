/*
Copyright (c) 2014, Pete Vieira
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <time.h>
#include "dynamixel.h"
#include "dynamixel_consts.h"
#include "dynamixel_io.h"

Dynamixel::Dynamixel()
{

}

Dynamixel::Dynamixel(DynamixelMotor dxl_params)
{

}

comm_status_t Dynamixel::set_id(int old_id, int new_id)
{
    dxl_write_word(old_id, DXL_ID, new_id);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_baud_rate(int servo_id, int baud_rate)
{
    dxl_write_word(servo_id, DXL_BAUD_RATE, baud_rate);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_return_delay_time(int servo_id, int delay)
{
    dxl_write_word(servo_id, DXL_RETURN_DELAY_TIME, delay);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_angle_max_enc(int servo_id, int max_angle)
{
    dxl_write_word(servo_id, DXL_CCW_ANGLE_LIMIT_L, max_angle);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_angle_min_enc(int servo_id, int min_angle)
{
    dxl_write_word(servo_id, DXL_CW_ANGLE_LIMIT_L, min_angle);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_drive_mode(int servo_id, int is_slave, int is_reverse)
{
    unsigned char drive_mode = (is_slave << 1) + is_reverse;

    dxl_write_word(servo_id, DXL_DRIVE_MODE, drive_mode);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_voltage_min(int servo_id, int min_voltage)
{
    if (min_voltage < 5)
        min_voltage = 0;
    int min_val = static_cast<int>(min_voltage * 10);
    dxl_write_word(servo_id, DXL_VOLTAGE_LIMIT_MIN, min_val);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_voltage_max(int servo_id, int max_voltage)
{
    if (max_voltage > 25)
        max_voltage = 25;
    int max_val = static_cast<int>(max_voltage * 10);
    dxl_write_word(servo_id, DXL_VOLTAGE_LIMIT_MAX, max_val);
    return dxl_get_result();
}

//-----------------------------------------------------------------
//   These functions can send a single command to a single servo
//-----------------------------------------------------------------
comm_status_t Dynamixel::set_torque_enabled(int servo_id, int enabled)
{
    if (enabled != 0) {
        fprintf(stderr, "[set_torque_enabled] Bad value passed in.\n");
        enabled = 1;
    }
    dxl_write_word(servo_id, DXL_TORQUE_ENABLE, enabled);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_compliance_margin_ccw(int servo_id, int margin)
{
    dxl_write_word(servo_id, DXL_CCW_COMPLIANCE_MARGIN, margin);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_compliance_margin_cw(int servo_id, int margin)
{
    dxl_write_word(servo_id, DXL_CW_COMPLIANCE_MARGIN, margin);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_compliance_slope_ccw(int servo_id, int slope)
{
    dxl_write_word(servo_id, DXL_CCW_COMPLIANCE_SLOPE, slope);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_compliance_slope_cw(int servo_id, int slope)
{
    dxl_write_word(servo_id, DXL_CW_COMPLIANCE_SLOPE, slope);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_p_gain(int servo_id, int p_gain)
{
    dxl_write_word(servo_id, DXL_P_GAIN, p_gain);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_i_gain(int servo_id, int i_gain)
{
    dxl_write_word(servo_id, DXL_I_GAIN, i_gain);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_d_gain(int servo_id, int d_gain)
{
    dxl_write_word(servo_id, DXL_D_GAIN, d_gain);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_punch(int servo_id, int punch)
{
    dxl_write_word(servo_id, DXL_PUNCH_L, punch);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_position_enc(int servo_id, int position)
{
    dxl_write_word(servo_id, DXL_GOAL_POSITION_L, position);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_speed_enc(int servo_id, int speed)
{
    dxl_write_word(servo_id, DXL_GOAL_SPEED_L, speed);
    return dxl_get_result();
}

comm_status_t Dynamixel::set_torque_limit(int servo_id, int torque_limit)
{
    dxl_write_word(servo_id, DXL_TORQUE_LIMIT_L, torque_limit);
    return dxl_get_result();
}


//------------------------------------
//   Servo Status Access Functions
//------------------------------------
int Dynamixel::get_model_number(int servo_id)
{
    return dxl_read_word(servo_id, DXL_MODEL_NUMBER_L);
}

int Dynamixel::get_firmware_version(int servo_id)
{
    return dxl_read_word(servo_id, DXL_VERSION);
}

int Dynamixel::get_return_delay_time(int servo_id)
{
    return dxl_read_word(servo_id, DXL_RETURN_DELAY_TIME);
}

int Dynamixel::get_angle_max(int servo_id)
{
    return dxl_read_word(servo_id, DXL_CCW_ANGLE_LIMIT_L);
}

int Dynamixel::get_angle_min(int servo_id)
{
    return dxl_read_word(servo_id, DXL_CW_ANGLE_LIMIT_L);
}

int Dynamixel::get_drive_mode(int servo_id)
{
    return dxl_read_word(servo_id, DXL_DRIVE_MODE);
}

int Dynamixel::get_voltage_max(int servo_id)
{
    return dxl_read_word(servo_id, DXL_VOLTAGE_LIMIT_MIN);
}

int Dynamixel::get_position(int servo_id)
{
    return dxl_read_word(servo_id, DXL_PRESENT_POSITION_L);
}

int Dynamixel::get_speed(int servo_id)
{
    return dxl_read_word(servo_id, DXL_PRESENT_SPEED_L);
}

int Dynamixel::get_voltage(int servo_id)
{
    return dxl_read_word(servo_id, DXL_PRESENT_VOLTAGE);
}

int Dynamixel::get_temperature(int servo_id)
{
    return dxl_read_word(servo_id, DXL_PRESENT_TEMPERATURE);
}

int Dynamixel::get_goal_position(int servo_id)
{
    return dxl_read_word(servo_id, DXL_GOAL_POSITION_L);
}

int Dynamixel::get_is_moving(int servo_id)
{
    return dxl_read_word(servo_id, DXL_MOVING);
}

int Dynamixel::get_load(int servo_id)
{
    return dxl_read_word(servo_id, DXL_PRESENT_LOAD_L);
}

double Dynamixel::get_time()
{
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return time.tv_sec + (1e-9)*time.tv_nsec;
}

dxl_motor_state_t Dynamixel::get_state(int servo_id)
{
    dxl_motor_state_t state;
    state.id = servo_id;
    state.goal = this->get_goal_position(servo_id);
    state.position = this->get_position(servo_id);
    state.error = state.position - state.goal;
    state.speed = this->get_speed(servo_id);
    state.load = this->get_load(servo_id);
    state.voltage = this->get_voltage(servo_id);
    state.temperature = this->get_temperature(servo_id);
    state.moving = this->get_is_moving(servo_id);
    state.time = this->get_time();
    return state;
}
