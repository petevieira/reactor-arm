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

#ifndef DYNAMIXEL_IO_H
#define DYNAMIXEL_IO_H

#include <math.h>
#include "dynamixel_consts.h"
#include "DynamixelMotor.h"

typedef struct dxl_motor_state {
    double time;
    int id;
    double goal;
    double position;
    double error;
    double speed;
    double load;
    double voltage;
    float temperature;
    int moving;
} dxl_motor_state_t;

/**
 * \class Dynamixel Dynamixel.h
 * \brief Class containing all the standard functions for setting
 * and getting parameters from the Dynamixel servos
 */
class Dynamixel {
public:
    /**
     * \brief Constructs a Dynamixel object
     */
    Dynamixel();

    Dynamixel(DynamixelMotor dxl_params);

    /**
     * \brief Destructs a Dynamixel object
     */
    ~Dynamixel();

    //------------------------------------------
    //   Setters for EEPROM Persistent Values
    //------------------------------------------
    /**
     * \brief Gets the model number of the servo
     * \param servo_id ID of the of which to get the model number
     * \return Model number
     */
    int get_model_number(int servo_id);

    /**
     * \brief Gets the firmware version of the servo
     * \param servo_id ID of the servo of which to get the firmware version
     * \return Firmware version
     */
    int get_firmware_version(int servo_id);

    /**
     * \brief Sets the ID of the Dynamixel device
     * \param old_id Current ID of the Dynamixel device
     * \param new_id New ID of the Dynamixel device
     * \return comm status
     */
    comm_status_t set_id(int old_id, int new_id);

    /**
     * \brief Sets the baudrate of a Dynamixel device
     * \param servo_id ID of the servo of which to set the baudrate
     * \param baud_rate New baud rate. Values are
     * 1:   1,000,000 (1 Mbps)
     * 3:     500,000 (500 kbps)
     * 4:     400,000 (400 kbps)
     * 7:     250,000 (250 kbps)
     * 9:     200,000 (200 kbps)
     * 16:    115,200 (115.2 kbps)
     * 34:     57,600 (56.6 kbps)
     * 103:    19,200 (19.2 kbps)
     * 207:     9,600 (9.6 kbps)
     * \return comm status
     */
    comm_status_t set_baud_rate(int servo_id, int baud_rate);

    /**
     * \brief Sets the delay time per data value that takes from the transmission
     * of Instruction Packet until the return of Status Packet.
     * 0 to 254 (0xFE) can be used, and the delay time per data value is 2 usec.
     * ie. Delay in microseconds = delay * 2 usec
     * \param servo_id ID of the servo of which to set the return delay time
     * \param delay Delay time, which ranges from 0 ~ 254.
     * \return comm status
     */
    comm_status_t set_return_delay_time(int servo_id, int delay);

    /**
     * \brief Sets the min angle (ie. max cw angle) of the servo
     * \param servo_id ID of the servo of which to set the min angle
     * \param min_angle Min angle in ticks that the servo can rotate to
     * Default value is 0.
     * \return comm status
     */
    comm_status_t set_angle_min_enc(int servo_id, int min_tick);

    /**
     * \brief Sets the max angle (ie. max ccw angle) of the servo
     * \param servo_id ID of the servo of which to set the max angle
     * \param max_angle Max angle in ticks that the servo can rotate to
     * DX, AX, RX Servos: 0 to 1023. Unit is 0.29 degrees
     * MX Servos: 0 to 4095. Unit is 0.09 degrees
     * EX Servos: 0 to 4095. Unit is 0.06 degrees
     * \return comm status
     */
    comm_status_t set_angle_max_enc(int servo_id, int max_tick);

    /**
     * \brief Sets the drive mode of the servo
     * \param servo_id ID of the servo of which to set the drive mode
     * \param is_slave Whether or not the servo is a slave (0 or 1)
     * \param is_reverse Whether or not position values are inversed (0 or 1)
     * \return comm status
     */
    comm_status_t set_drive_mode(int servo_id, int is_slave, int is_reverse);

    /**
     * \brief Sets the min allowed voltage of the servo
     * \param servo_id ID of the servo of which to set the min voltage
     * \param min_voltage Minimum allowed voltage of the servo
     * \return comm status
     */
    comm_status_t set_voltage_min(int servo_id, int min_voltage);

    /**
     * \brief Sets the max allowed voltage of the servo
     * \param servo_id ID of the servo of which to set the max voltage
     * \param max_voltage Maximum allowed voltage of the servo
     * \return comm status
     */
    comm_status_t set_voltage_max(int servo_id, int max_voltage);


    //-----------------------------------------------------------------
    //   These functions can send a single command to a single servo
    //-----------------------------------------------------------------
    /**
     * \brief Sets whether or not torque for the motor is enabled
     * \param servo_id ID of the servo of which to enable/disable torque
     * \param enabled 1 to enable, 0 to disable torque
     * \return comm status
     */
    comm_status_t set_torque_enabled(int servo_id, int enabled);

    /**
     * \brief Sets the compliance margin in the ccw direction
     * \param servo_id ID of the servo of which to set the compliance margin
     * \param margin The error between goal and present position.
     * Range is 0 ~ 255.
     * DX, AX, RX Servos: Unit is 0.29 degrees
     * Not applicable for EX and MX Series. PID gains are used instead
     * \return comm status
     */
    comm_status_t set_compliance_margin_ccw(int servo_id, int margin);

    /**
     * \brief Sets the compliance margin in the cw direction
     * \param servo_id ID of the servo of which to set the compliance margin
     * \param margin The error between goal and present position.
     * Range is 0 ~ 255.
     * DX, AX, RX Servos: Unit is 0.29 degrees
     * Not applicable for EX and MX Series. PID gains are used instead
     * \return comm status
     */
    comm_status_t set_compliance_margin_cw(int servo_id, int margin);

    /**
     * \brief Sets the level of torque near the goal position in the CCW direction
     * \param servo_id ID of the servo of which to set the slope of
     * 7 levels exist. The higher the level the more flexibility
     * Values: 2, 4, 8, 16, 32, 64, 128
     * \return comm status
     */
    comm_status_t set_compliance_slope_ccw(int servo_id, int slope);

    /**
     * \brief Sets the level of torque near the goal position in the CW direction
     * \param servo_id ID of the servo of which to set the slope of
     * 7 levels exist. The higher the level the more flexibility
     * Values: 2, 4, 8, 16, 32, 64, 128
     * \return comm status
     */
    comm_status_t set_compliance_slope_cw(int servo_id, int slope);

    /**
     * \brief Sets the proportional gain of the servo
     * \param servo_id ID of the servo of which to set the p_gain
     * \param p_gain Proportional gain. 0 ~ 254. Only for MX Series servos.
     * \return comm status
     */
    comm_status_t set_p_gain(int servo_id, int p_gain);

    /**
     * \brief Sets the integral gain of the servo
     * \param servo_id ID of the servo of which to set the i_gain
     * \param i_gain Integral gain. 0 ~ 254. Only for MX Series servos.
     * \return comm status
     */
    comm_status_t set_i_gain(int servo_id, int i_gain);

    /**
     * \brief Sets the derivative gain of the servo
     * \param servo_id ID of the servo of which to set the d_gain
     * \param d_gain Derivative gain. 0 ~ 254. Only for MX Series servos.
     * \return comm status
     */
    comm_status_t set_d_gain(int servo_id, int d_gain);

    /**
     * \brief Sets the current to drive the motor when near the goal position
     * \param servo_id ID of the servo of which to set the punch
     * \param punch The current to drive the motor when near the goal position
     * Range is 32 ~ 1023 (0x20 ~ 0x3FF)
     * \return comm status
     */
    comm_status_t set_punch(int servo_id, int punch);

    /**
     * \brief Sets the goal position of the servo in ticks
     * \param servo_id ID of the servo of which to set the position
     * \param position Goal position of the servo in encoder ticks
     * DX, AX, RX Servos: 0 to 1023. Unit is 0.29 degrees
     * MX Servos: 0 to 4095. Unit is 0.09 degrees
     * EX Servos: 0 to 4095. Unit is 0.06 degrees
     * \return comm status
     */
    comm_status_t set_position_enc(int servo_id, int position);

    /**
     * \brief Sets the speed to move to the goal position
     * \param servo_id ID of the servo of which to set the moving speed
     * \param speed Moving speed of the servo
     * Joint Mode: range is 0 ~ 1023
     * Wheel Mode: range is 0 ~ 2047
     * \return comm status
     */
    comm_status_t set_speed_enc(int servo_id, int speed);

    /**
     * \brief Sets the max torque limit of the servo
     * \param servo_id ID of the servo of which to set the max torque
     * \param torque_limit Torque limit. 0 ~ 1023 (0x3FF)
     * \return comm status
     */
    comm_status_t set_torque_limit(int servo_id, int torque_limit);


    //------------------------------------
    //   Servo Status Access Functions
    //------------------------------------
    /**
     * \brief Gets the return delay time of the servo
     * \param servo_id ID of the servo of which to get the delay time
     * \return return delay time
     */
    int get_return_delay_time(int servo_id);

    /**
     * \brief Gets the max angle
     */
    int get_angle_max(int servo_id);

    int get_angle_min(int servo_id);

    int get_drive_mode(int servo_id);

    int get_voltage_min(int servo_id);

    int get_voltage_max(int servo_id);

    int get_position(int servo_id);

    int get_speed(int servo_id);

    int get_voltage(int servo_id);

    int get_temperature(int servo_id);

    int get_goal_position(int servo_id);

    int get_is_moving(int servo_id);

    int get_load(int servo_id);

    double get_time();

    dxl_motor_state_t get_state(int servo_id);

protected:
    inline double radToDeg(double radians) { return radians * M_PI / 180.0; }
    inline double degToRad(double degrees) { return degrees / M_PI * 180.0; }
};

#endif // DYNAMIXEL_IO_H
