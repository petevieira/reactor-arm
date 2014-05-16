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

#ifndef DYNAMIXEL_CONSTS_H
#define DYNAMIXEL_CONSTS_H

// Dynamixel Constants

typedef enum {
    DEFAULT_BAUDNUMBER  = 1,
    ID                  = 2,
    LENGTH              = 3,
    INSTRUCTION         = 4,
    ERRBIT              = 4,
    PARAMETER           = 5
} packet_params_t;

// Control Table Constants
typedef enum {
  // EEPROM Values
    DXL_MODEL_NUMBER_L          = 0,
    DXL_MODEL_NUMBER_H          = 1,
    DXL_VERSION                 = 2,
    DXL_ID                      = 3,
    DXL_BAUD_RATE               = 4,
    DXL_RETURN_DELAY_TIME       = 5,
    DXL_CW_ANGLE_LIMIT_L        = 6,
    DXL_CW_ANGLE_LIMIT_H        = 7,
    DXL_CCW_ANGLE_LIMIT_L       = 8,
    DXL_CCW_ANGLE_LIMIT_H       = 9,
    DXL_DRIVE_MODE              = 10,
    DXL_LIMIT_TEMPERATURE       = 11,
    DXL_VOLTAGE_LIMIT_MIN       = 12,
    DXL_VOLTAGE_LIMIT_MAX       = 13,
    DXL_MAX_TORQUE_L            = 14,
    DXL_MAX_TORQUE_H            = 15,
    DXL_RETURN_LEVEL            = 16,
    DXL_ALARM_LED               = 17,
    DXL_ALARM_SHUTDOWN          = 18,
  // RAM Values
    DXL_OPERATING_MODE          = 19,
    DXL_DOWN_CALIBRATION_L      = 20,
    DXL_DOWN_CALIBRATION_H      = 21,
    DXL_UP_CALIBRATION_L        = 22,
    DXL_UP_CALIBRATION_H        = 23,
    DXL_TORQUE_ENABLE           = 24,
    DXL_LED                     = 25,
    DXL_CW_COMPLIANCE_MARGIN    = 26,
    DXL_CCW_COMPLIANCE_MARGIN   = 27,
    DXL_CW_COMPLIANCE_SLOPE     = 28,
    DXL_CCW_COMPLIANCE_SLOPE    = 29,
    DXL_D_GAIN                  = 26,
    DXL_I_GAIN                  = 27,
    DXL_P_GAIN                  = 28,
    DXL_GOAL_POSITION_L         = 30,
    DXL_GOAL_POSITION_H         = 31,
    DXL_GOAL_SPEED_L            = 32,
    DXL_GOAL_SPEED_H            = 33,
    DXL_TORQUE_LIMIT_L          = 34,
    DXL_TORQUE_LIMIT_H          = 35,
    DXL_PRESENT_POSITION_L      = 36,
    DXL_PRESENT_POSITION_H      = 37,
    DXL_PRESENT_SPEED_L         = 38,
    DXL_PRESENT_SPEED_H         = 39,
    DXL_PRESENT_LOAD_L          = 40,
    DXL_PRESENT_LOAD_H          = 41,
    DXL_PRESENT_VOLTAGE         = 42,
    DXL_PRESENT_TEMPERATURE     = 43,
    DXL_REGISTERED_INSTRUCTION  = 44,
    DXL_PAUSE_TIME              = 45,
    DXL_MOVING                  = 46,
    DXL_LOCK                    = 47,
    DXL_PUNCH_L                 = 48,
    DXL_PUNCH_H                 = 49,
    DXL_SENSED_CURRENT_L        = 56,
    DXL_SENSED_CURRENT_H        = 57
} control_table_consts_t;

// Status Return Levels
typedef enum {
    DXL_RETURN_NONE = 0,
    DXL_RETURN_READ = 1,
    DXL_RETURN_ALL  = 2
} status_return_levels_t;

// Instruction Set
typedef enum {
    DXL_PING        = 1,
    DXL_READ_DATA   = 2,
    DXL_WRITE_DATA  = 3,
    DXL_REG_WRITE   = 4,
    DXL_ACTION      = 5,
    DXL_RESET       = 6,
    DXL_SYNC_WRITE  = 131
} instructions_t;

typedef enum {
    DXL_MAXNUM_RXPARAMS = 60,
    DXL_MAXNUM_TXPARAMS = 150
} max_params_t;

// Broadcast Constant
typedef enum {
    DXL_BROADCAST_ID = 254
} broadcast_t;

// Error Codes
typedef enum {
    DXL_NO_ERROR            = 0,
    DXL_INPUT_VOLTAGE_ERROR = 1,
    DXL_ANGLE_LIMIT_ERROR   = 2,
    DXL_OVERHEATING_ERROR   = 4,
    DXL_RANGE_ERROR         = 8,
    DXL_CHECKSUM_ERROR      = 16,
    DXL_OVERLOAD_ERROR      = 32,
    DXL_INSTRUCTION_ERROR   = 64
} rx_packet_error_t;

// Static parameters
typedef enum {
    DXL_MIN_COMPLIANCE_MARGIN   = 0,
    DXL_MAX_COMPLIANCE_MARGIN   = 255,
    DXL_MIN_COMPLIANCE_SLOPE    = 1,
    DXL_MAX_COMPLIANCE_SLOPE    = 254
} compliance_t;

// These are guesses as Dynamixel documentation doesn't have any info about it
typedef enum {
    DXL_MIN_PUNCH       = 32,   // minimum voltage applied when just outside compliance margin
    DXL_MAX_PUNCH       = 1023, // maximum voltage applied when just outside compliance margin
    DXL_MAX_SPEED_TICK  = 1023, // maximum speed in encoder units
    DXL_MAX_TORQUE_TICK = 1023  // maximum torque in encoder units
} torque_limits_t;

typedef enum comm_status {
    DXL_COMM_TXSUCCESS  = 0,
    DXL_COMM_RXSUCCESS  = 1,
    DXL_COMM_TXFAIL     = 2,
    DXL_COMM_RXFAIL     = 3,
    DXL_COMM_TXERROR    = 4,
    DXL_COMM_RXWAITING  = 5,
    DXL_COMM_RXTIMEOUT  = 6,
    DXL_COMM_RXCORRUPT  = 7
} comm_status_t;

static const double KGCM_TO_NM     = 0.0980665;    // 1 kg-cm is that many N-m
static const double RPM_TO_RADSEC  = 0.104719755;  // 1 RPM is that many rad/sec

// maximum holding torque is in N-m per volt
// maximum velocity is in rad/sec per volt

#endif // DYNAMIXEL_CONST_H
