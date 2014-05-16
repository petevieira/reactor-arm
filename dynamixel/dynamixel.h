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

/*
 * Communication Protocol
 *
 * Instruction Packet (TX from CM-510):
 * ------------------------------------
 * 2 Start Bytes:       0xFF 0xFF
 * Dynamixel ID:    id (single byte, OxFE = broadcast)
 * Packet Length:       number of parameters + 2
 * Instruction:         Instr (single byte, see AX-12 manual)
 * Parameters:          depend on instruction
 * Checksum:            ~(ID+Length+Instruction+Parameters)
 *
 * Status Packet (RX from Dynamixel):
 * ------------------------------------
 * 2 Start Bytes:       0xFF 0xFF
 * Packet Length:       number of parameters + 2
 * Error Code:      Bit 0..6 encode the errors (see manual)
 * Parameters:          depends on instruction sent
 * Checksum:            ~(ID+Length+Error+Parameters)
 *
 */

/*
User point of view:
    List of IDs, Model #s
*/

// Dynamixel SDK platform independent header
#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "dynamixel_consts.h"
#include <stdint.h>

//--------------------------------
//    device control methods
//--------------------------------
/**
 * \brief It initializes the communication devices and makes ready status.
 * \param deviceIndex It is the number of currently connected communication devices
 * \param baudnum It is Baud rate number to be set.  It is the same number used by Dynamixel.
 * \return 1 is success, 0 is failure
 */
int dxl_initialize(int deviceIndex, int baudnum);

/**
 * \breif It terminates the communication devices.
 */
void dxl_terminate(void);


//---------------------------------
//    set/get packet methods
//---------------------------------
/**
 * \brief Inputs the ID into the instruction packet
 * \param Dynamixel ID to be transmitted to the instruction packet
 */
void dxl_set_txpacket_id(int id);

/**
 * \brief Inputs the command code into the instruction packet
 * \param instruction Instruction code from instructions_t
 */
void dxl_set_txpacket_instruction(int instruction);

/**
 * \brief Inputs the parameter into the instruction packet
 * \param index The parameter number. Ranges from 0 ~ DXL_MAXNUM_TXPARAM-1
 * \param value The parameter value. Ranges from 0 ~ 255
 */
void dxl_set_txpacket_parameter(int index, int value);

/**
 * \brief Set the length of the instruction packet
 * \param length Length of the instruction packet
 */
void dxl_set_txpacket_length(int length);

/**
 * \brief Gets the error included in the status packet
 * \param errbit The Bit Flag to check whether errors occur or not. See rx_packet_error_t
 * \return 1 if an error occurs, 0 is no errors occur.
 */
int dxl_get_rxpacket_error(int errbit);

/**
 * \brief Gets the parameter of the status packet
 * \param index The parameter number to be checked
 * \return The parameter value is the "index"(th) of the status packet
 */
int dxl_get_rxpacket_parameter(int index);

/**
 * \brief Get the length of the status packet
 * \return The length of the status packet
 */
int dxl_get_rxpacket_length(void);

/**
 * \brief Converts two-byte type to WORD-type data
 * \param lowbyte Lower byte to be made into WORD-type
 * \param highbyte Higher byte to be made into WORD-type
 * \return WORD-type data made out of lower and higher bytes
 */
int dxl_makeword(int lowbyte, int highbyte);

/**
 * \brief Gets the lower byte of WORD-type data
 * \param word WORD-type data to get lower byte from
 * \return Lower byte
 */
int dxl_get_lowbyte(int word);

/**
 * \brief Gets the higher byte of WORD-type data
 * \param word WORD-type data to get higher byte from
 * \return Higher byte
 */
int dxl_get_highbyte(int word);


//---------------------------------
//  packet communication methods
//---------------------------------
/**
 * \brief Transmits the instruction packet to the Dynamixel device
 */
void dxl_tx_packet(void);

/**
 * \brief Gets the status packet from the driver buffer.
 * After calling this function, the result must always be check by
 * calling the dxl_get_result function
 */
void dxl_rx_packet(void);

/**
 * \brief Gets the tx and rx packets together
 * After calling this function, the result must always be check by
 * calling the dxl_get_result function
 */
void dxl_txrx_packet(void);

/**
 * \brief Gets the result of the communication
 * \return Value of the communication result. See comm_status_t
 */
comm_status_t dxl_get_result();


//---------------------------------
//   high communication methods
//---------------------------------
/**
 * \brief Checks the existence of a Dynamixel device with selected ID
 * \param id ID of the device to check
 */
void dxl_ping(int id);

/**
 * \brief Reads one byte from a Dynamixel device
 * \param id ID of the device to read from
 * \param address Address of value to read.
 * \return Value read
 */
int dxl_read_byte(int id, int address);

/**
 * \brief Writes one byte to a Dynamixel device
 * \param id ID of the device to write to
 * \param address Address of the value to read.
 * \param value Value to write to that address
 */
void dxl_write_byte(int id, int address, int value);

/**
 * \brief Reads two bytes from a Dynamixel device
 * \param id ID of the device to read from
 * \param address Address of value to read.
 * \return Value read
 */
int dxl_read_word(int id, int address);

/**
 * \brief Writes two bytes to a Dynamixel device
 * \param id ID of the device to write to
 * \param address Address of the value to read.
 * \param value Value to write to that address
 */
void dxl_write_word(int id, int address, int value);


//-------------------------------
//        String Functions
//-------------------------------
// Print communication result
char* dxl_comm_status_to_string(int commStatus);

/**
 * \brief The string equivalent of the error code
 * \return char* string
 */
char* dxl_error_to_string();


//------------------------------------
//      Sync Writing Function
//------------------------------------
/**
 * \brief Function for controlling several Dynamixel actuators at the same time.
 * The communication time decreases by using the Sync Write instruction
 * since many instructions can be transmitted by a single instruction.
 * However, you can use this instruction only when the lengths and addresses
 * of the control table to be written to are the same.
 * The broadcast ID (0xFE) needs to be used for transmitting.
 * ID: 0xFE
 * Length: (L + 1) * N + 4 (L: Data length for each Dynamixel actuator, N: The number of Dynamixel actuators)
 * Instruction: 0x83
 * Parameter1 Starting address of the location where the data is to be written
 * Parameter2 The length of the data to be written (L)
 * Parameter3 The ID of the 1st Dynamixel actuator
 * Parameter4 The 1st data for the 1st Dynamixel actuator
 * Parameter5 The 2nd data for the 1st Dynamixel actuator
 * ParameterL+4 The ID of the 2nd Dynamixel actuator
 * ParameterL+5 The 1st data for the 2nd Dynamixel actuator
 * ParameterL+6 The 2nd data for the 2nd Dynamixel actuator
 * \param num_actuators Number of actuators to control
 * \param address Address of parameter to control
 * \param ids Array of servo ID numbers to control
 * \param values Array of values to set on the servos
 * \return Comm status
 * \note NOTE: this function only allows 2 bytes of data per actuator
 */
int dxl_sync_write_word(int num_actuators, int address,
                        const uint8_t ids[], int16_t values[]);

/**
 * \brief Function setting goal and speed for all Dynamixel actuators
 * at the same time. Uses the Sync Write instruction
 * (also see dxl_sync_write_word)
 * \param num_actuators number of Dynamixel servos
 * \param ids array of Dynamixel ids to write to
 * \param goal array of goal positions
 * \param speed array of moving speeds
 * \return Comm status
 */
int dxl_set_goal_speed(int num_actuators, const uint8_t ids[],
                       uint16_t goal[], uint16_t speed[]);

#ifdef __cplusplus
}
#endif

#endif // DYNAMIXEL_H
