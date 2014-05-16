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

#include "dxl_hal.h"
#include "dynamixel.h"
#include "dynamixel_consts.h"
#include <stdio.h>
#include <stdint.h>

unsigned char gbInstructionPacket[DXL_MAXNUM_TXPARAMS+10] = {0};
unsigned char gbStatusPacket[DXL_MAXNUM_RXPARAMS+10] = {0};
unsigned char gbRxPacketLength = 0;
unsigned char gbRxGetLength = 0;
comm_status_t gbCommStatus = DXL_COMM_RXSUCCESS;
int giBusUsing = 0;


int dxl_initialize(int devIndex, int baudnum)
{
    // convert baudrate number to baudrate in bits/sec
	float baudrate;	
	baudrate = 2000000.0f / (float)(baudnum + 1);
	
    // connect with the device
    if (dxl_hal_open(devIndex, baudrate) == 0)
		return 0;

    // show success and bus as free
    gbCommStatus = DXL_COMM_RXSUCCESS;
	giBusUsing = 0;
	return 1;
}

// terminate communication
void dxl_terminate(void)
{
	dxl_hal_close();
}

// send an instruction packet
void dxl_tx_packet()
{
	unsigned char i;
	unsigned char TxNumByte, RealTxNumByte;
	unsigned char checksum = 0;

    // do nothing if bus is busy
    if (giBusUsing == 1)
		return;
	
    // set bus as busy
	giBusUsing = 1;

    // check packet does not exceed max number of parameters
    if (gbInstructionPacket[LENGTH] > (DXL_MAXNUM_TXPARAMS+2)) {
        gbCommStatus = DXL_COMM_TXERROR;
		giBusUsing = 0;
		return;
	}
	
    // check instruction is valid
    if (gbInstructionPacket[INSTRUCTION] != DXL_PING
        && gbInstructionPacket[INSTRUCTION] != DXL_READ_DATA
        && gbInstructionPacket[INSTRUCTION] != DXL_WRITE_DATA
        && gbInstructionPacket[INSTRUCTION] != DXL_REG_WRITE
        && gbInstructionPacket[INSTRUCTION] != DXL_ACTION
        && gbInstructionPacket[INSTRUCTION] != DXL_RESET
        && gbInstructionPacket[INSTRUCTION] != DXL_SYNC_WRITE)
	{
        gbCommStatus = DXL_COMM_TXERROR;
		giBusUsing = 0;
		return;
	}
	
    // create packet header (Two 0xFF bytes indicate start of an incoming packet)
	gbInstructionPacket[0] = 0xff;
	gbInstructionPacket[1] = 0xff;

    // calculate the checksum
    for (i=0; i<(gbInstructionPacket[LENGTH]+1); i++)
		checksum += gbInstructionPacket[i+2];
	gbInstructionPacket[gbInstructionPacket[LENGTH]+3] = ~checksum;
	
    // if timeout or corrupt, clear the buffer
    if (gbCommStatus == DXL_COMM_RXTIMEOUT || gbCommStatus == DXL_COMM_RXCORRUPT)
		dxl_hal_clear();

    // transmit the packet
	TxNumByte = gbInstructionPacket[LENGTH] + 4;
    RealTxNumByte = dxl_hal_tx((unsigned char*)gbInstructionPacket, TxNumByte);

    // check that all bytes were sent
    if (TxNumByte != RealTxNumByte) {
        // communication failure. not all bytes were sent
        gbCommStatus = DXL_COMM_TXFAIL;
		giBusUsing = 0;
		return;
	}

    // for read instructions we expect a reply within the timeout period
    if (gbInstructionPacket[INSTRUCTION] == DXL_READ_DATA)
        dxl_hal_set_timeout(gbInstructionPacket[PARAMETER+1] + 6);
	else
        dxl_hal_set_timeout(6);

    // declare success
    gbCommStatus = DXL_COMM_TXSUCCESS;
}

// receive a status packet
void dxl_rx_packet()
{
	unsigned char i, j, nRead;
	unsigned char checksum = 0;

    // return if bus is busy
    if (giBusUsing == 0)
		return;

    // if instruction was broadcast there is nothing to wait for
    if (gbInstructionPacket[ID] == DXL_BROADCAST_ID) {
        gbCommStatus = DXL_COMM_RXSUCCESS;
		giBusUsing = 0;
		return;
	}
	
    // check transmission was successful and reset
    if (gbCommStatus == DXL_COMM_TXSUCCESS) {
		gbRxGetLength = 0;
		gbRxPacketLength = 6;
	}
	
    // start reading what has been received on the bus
    nRead = dxl_hal_rx((unsigned char*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength - gbRxGetLength);
	gbRxGetLength += nRead;
    // check for timeout
    if (gbRxGetLength < gbRxPacketLength) {
        if (dxl_hal_timeout() == 1) {
            // timeout, return and error
            if (gbRxGetLength == 0)
                gbCommStatus = DXL_COMM_RXTIMEOUT;
			else
                gbCommStatus = DXL_COMM_RXCORRUPT;
			giBusUsing = 0;
			return;
		}
	}
	
    // find packet header
    for (i=0; i<(gbRxGetLength-1); i++)
	{
        if (gbStatusPacket[i] == 0xff && gbStatusPacket[i+1] == 0xff) {
			break;
		}
        else if (i == gbRxGetLength-2 && gbStatusPacket[gbRxGetLength-1] == 0xff) {
			break;
		}
    }

    // remove the header and copy to beginning of array
    if (i > 0) {
        for (j=0; j<(gbRxGetLength-i); j++)
			gbStatusPacket[j] = gbStatusPacket[j + i];
			
		gbRxGetLength -= i;		
	}

    // if we haven't received all bytes yet we all still waiting
    if (gbRxGetLength < gbRxPacketLength) {
        gbCommStatus = DXL_COMM_RXWAITING;
		return;
	}

    // check id pairing, we only a want response from the right Dynamixel
    if (gbInstructionPacket[ID] != gbStatusPacket[ID]) {
        gbCommStatus = DXL_COMM_RXCORRUPT;
		giBusUsing = 0;
		return;
	}
	
    // check the lenght of the status packet to see if we expect more
	gbRxPacketLength = gbStatusPacket[LENGTH] + 4;
    if (gbRxGetLength < gbRxPacketLength) {
        // more to come, keep reading
        nRead = dxl_hal_rx((unsigned char*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength - gbRxGetLength);
		gbRxGetLength += nRead;
        if (gbRxGetLength < gbRxPacketLength) {
            gbCommStatus = DXL_COMM_RXWAITING;
			return;
		}
	}

    // check checksum
    for (i=0; i<(gbStatusPacket[LENGTH]+1); i++)
		checksum += gbStatusPacket[i+2];
	checksum = ~checksum;

    if (gbStatusPacket[gbStatusPacket[LENGTH]+3] != checksum) {
        // checksum doesn't match
        gbCommStatus = DXL_COMM_RXCORRUPT;
		giBusUsing = 0;
		return;
	}
	
    // everything is fine, declare success and declare bus as free
    gbCommStatus = DXL_COMM_RXSUCCESS;
	giBusUsing = 0;
}

// sends instruction packet and wait for reply
void dxl_txrx_packet()
{
    // send instruction packet
	dxl_tx_packet();

    // send was not successful, return error
    if (gbCommStatus != DXL_COMM_TXSUCCESS)
        return;
	
    // wait for reply within the timeout period
    do {
		dxl_rx_packet();		
    } while (gbCommStatus == DXL_COMM_RXWAITING);
}

// gets the last error status
comm_status_t dxl_get_result()
{
	return gbCommStatus;
}

// sets the Dynamixel ID for the instruction packet
void dxl_set_txpacket_id(int id)
{
	gbInstructionPacket[ID] = (unsigned char)id;
}

// sets the instruction for the instruction packet
void dxl_set_txpacket_instruction(int instruction)
{
	gbInstructionPacket[INSTRUCTION] = (unsigned char)instruction;
}

// sets parameter for the instruction packet
void dxl_set_txpacket_parameter(int index, int value)
{
	gbInstructionPacket[PARAMETER+index] = (unsigned char)value;
}

// sets the packet length for the instruction packet
void dxl_set_txpacket_length(int length)
{
	gbInstructionPacket[LENGTH] = (unsigned char)length;
}

// unpacks the error code received from the Dynamixel device
int dxl_get_rxpacket_error(int errbit)
{
    if (gbStatusPacket[ERRBIT] & (unsigned char)errbit)
		return 1;

	return 0;
}

// gets the status packet length
int dxl_get_rxpacket_length()
{
	return (int)gbStatusPacket[LENGTH];
}

// gets a parameter from the status packet
int dxl_get_rxpacket_parameter(int index)
{
	return (int)gbStatusPacket[PARAMETER+index];
}

// combines two bytes into a word
int dxl_makeword(int lowbyte, int highbyte)
{
	unsigned short word;

	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
	return (int)word;
}

// extracts lower byte from a word
int dxl_get_lowbyte(int word)
{
	unsigned short temp;

	temp = word & 0xff;
	return (int)temp;
}

// extracts higher byte from a word
int dxl_get_highbyte(int word)
{
	unsigned short temp;

	temp = word & 0xff00;
	temp = temp >> 8;
	return (int)temp;
}

// ping a Dynamixel device
void dxl_ping(int id)
{
    // wait for the bus to be free
    while (giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
    gbInstructionPacket[INSTRUCTION] = DXL_PING;
	gbInstructionPacket[LENGTH] = 2;
	
	dxl_txrx_packet();
}

// reads data from the control table of a Dynamixel device
// Length 0x04, Instruction 0x02
// Parameter1 Starting address of the location where the data is to be read
// Parameter2 Length of the data to be read (one byte in this case)
int dxl_read_byte(int id, int address)
{
    // wait for the bus to be free
    while (giBusUsing);

    // create a READ instruction packet and send
	gbInstructionPacket[ID] = (unsigned char)id;
    gbInstructionPacket[INSTRUCTION] = DXL_READ_DATA;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 1;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();

	return (int)gbStatusPacket[PARAMETER];
}

// Function to write data into the control table of the Dynamixel actuator
// Length N+3 (N is the number of data to be written)
// Instruction 0x03
// Parameter1 Starting address of the location where the data is to be written
// Parameter2 1st data to be written
// Parameter3 2nd data to be written, etc.
// In this case we only have a 1-byte parameter
void dxl_write_byte(int id, int address, int value)
{
    // wait for the bus to be free
    while (giBusUsing);

    // create a WRITE instruction packet and send
	gbInstructionPacket[ID] = (unsigned char)id;
    gbInstructionPacket[INSTRUCTION] = DXL_WRITE_DATA;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)value;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();
}

// Read data from the control table of a Dynamixel device
// Length 0x04, Instruction 0x02
// Parameter1 Starting address of the location where the data is to be read
// Parameter2 Length of the data to be read (2 bytes in this case)
int dxl_read_word(int id, int address)
{
    // wait for the bus to be free
    while (giBusUsing);

    // create a READ instruction packet and send
	gbInstructionPacket[ID] = (unsigned char)id;
    gbInstructionPacket[INSTRUCTION] = DXL_READ_DATA;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 2;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();

    // combine the 2 bytes into a word and return
	return dxl_makeword((int)gbStatusPacket[PARAMETER], (int)gbStatusPacket[PARAMETER+1]);
}

// Function to write data into the control table of the Dynamixel actuator
// Length N+3 (N is the number of data to be written)
// Instruction 0x03
// Parameter1 Starting address of the location where the data is to be written
// Parameter2 1st data to be written
// Parameter3 2nd data to be written, etc.
// In this case we have a two 1-byte parameters
void dxl_write_word(int id, int address, int value)
{
    // wait for the bus to be free
    while (giBusUsing);

    // create a WRITE instruction packet and send
	gbInstructionPacket[ID] = (unsigned char)id;
    gbInstructionPacket[INSTRUCTION] = DXL_WRITE_DATA;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)dxl_get_lowbyte(value);
	gbInstructionPacket[PARAMETER+2] = (unsigned char)dxl_get_highbyte(value);
	gbInstructionPacket[LENGTH] = 5;
	
	dxl_txrx_packet();
}

// Print communication result
char* dxl_comm_status_to_string(int commStatus)
{
        switch (commStatus) {
        case DXL_COMM_TXFAIL:
                return "DXL_COMM_TXFAIL: Failed transmitting instruction packet!";
                break;
        case DXL_COMM_TXERROR:
                return "DXL_COMM_TXERROR: Incorrect instruction packet!";
                break;
        case DXL_COMM_RXFAIL:
                return "DXL_COMM_RXFAIL: Failed to get status packet from device!";
                break;
        case DXL_COMM_RXWAITING:
                return "DXL_COMM_RXWAITING: Waiting to receive status packet!";
                break;
        case DXL_COMM_RXTIMEOUT:
                return "DXL_COMM_RXTIMEOUT: Status packet not received!";
                break;
        case DXL_COMM_RXCORRUPT:
                return "COMM_RXCORRUPT: Incorrect status packet!";
                break;
        default:
                return "Unknown error code!";
                break;
        }
}

// Print error bit of status packet
char* dxl_error_to_string()
{
        if(dxl_get_rxpacket_error(DXL_INPUT_VOLTAGE_ERROR) == 1)
                return "Input voltage error!";

        if(dxl_get_rxpacket_error(DXL_ANGLE_LIMIT_ERROR) == 1)
                return "Angle limit error!";

        if(dxl_get_rxpacket_error(DXL_OVERHEATING_ERROR) == 1)
                return "Overheat error!";

        if(dxl_get_rxpacket_error(DXL_RANGE_ERROR) == 1)
                return "Out of range error!";

        if(dxl_get_rxpacket_error(DXL_CHECKSUM_ERROR) == 1)
                return "Checksum error!";

        if(dxl_get_rxpacket_error(DXL_OVERLOAD_ERROR) == 1)
                return "Overload error!";

        if(dxl_get_rxpacket_error(DXL_INSTRUCTION_ERROR) == 1)
                return "Instruction code error!";
}

// Function for controlling several Dynamixel actuators at the same time.
int dxl_sync_write_word(int num_actuators, int address, const uint8_t ids[], int16_t values[])
{
        int i = 0;

        // wait for the bus to be free
        while(giBusUsing);

        // check how many actuators are to be broadcast to
        if (num_actuators == 0) {
                // nothing to do, return
                return 0;
        } else if (num_actuators == 1) {
                // easy, we can use dxl_write_word for a single actuator
                dxl_write_word( ids[0], address, values[0] );
                return 0;
        }

        // Multiple values, create sync write packet
        // ID is broadcast id
        dxl_set_txpacket_id(DXL_BROADCAST_ID);
        // Instruction is sync write
        dxl_set_txpacket_instruction(DXL_SYNC_WRITE);
        // Starting address where to write to
        dxl_set_txpacket_parameter(0, address);
        // Length of data to be written (each word = 2 bytes)
        dxl_set_txpacket_parameter(1, 2);
        // Loop over the active Dynamixel id's
        for( i=0; i<num_actuators; i++ )
        {
                // retrieve the id and value for each actuator and add to packet
                dxl_set_txpacket_parameter(2+3*i, ids[i]);
                dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(values[i]));
                dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(values[i]));
        }

        // total length is as per formula above with L=2
        dxl_set_txpacket_length((2+1)*num_actuators + 4);

        // all done, send the packet
        dxl_txrx_packet();

        // there is no status packet return, so return the CommStatus
        return gbCommStatus;
}


// Function setting goal and speed for all Dynamixel actuators at the same time
// Uses the Sync Write instruction (also see dxl_sync_write_word)
// Inputs:      num_actuators - number of Dynamixel servos
//                      ids - array of Dynamixel ids to write to
//                      goal - array of goal positions
//                      speed - array of moving speeds
//Returns:      commStatus
int dxl_set_goal_speed(int num_actuators, const uint8_t ids[], uint16_t goal[], uint16_t speed[])
{
        int i = 0;

        // wait for the bus to be free
        while (giBusUsing);

        // check how many actuators are to be broadcast to
        if (num_actuators == 0) {
                // nothing to do, return
                return 0;
        }

        // Multiple values, create sync write packet
        // ID is broadcast id
        dxl_set_txpacket_id(DXL_BROADCAST_ID);

        // Instruction is sync write
        dxl_set_txpacket_instruction(DXL_SYNC_WRITE);

        // Starting address where to write to
        dxl_set_txpacket_parameter(0, DXL_GOAL_POSITION_L);

        // Length of data to be written (2 words = 4 bytes)
        dxl_set_txpacket_parameter(1, 4);

        // Loop over the active Dynamixel id's
        for ( i=0; i<num_actuators; i++ )
        {
                // retrieve the id and value for each actuator and add to packet
                dxl_set_txpacket_parameter(2+5*i, ids[i]);
                dxl_set_txpacket_parameter(2+5*i+1, dxl_get_lowbyte(goal[i]));
                dxl_set_txpacket_parameter(2+5*i+2, dxl_get_highbyte(goal[i]));
                dxl_set_txpacket_parameter(2+5*i+3, dxl_get_lowbyte(speed[i]));
                dxl_set_txpacket_parameter(2+5*i+4, dxl_get_highbyte(speed[i]));
        }

        // total length is as per formula above with L=4
        dxl_set_txpacket_length((4+1)*num_actuators + 4);

        // all done, send the packet
        dxl_txrx_packet();

        // there is no status packet return, so return the CommStatus
        return gbCommStatus;
}



/**
 * 0xFF 0xFF:       Two 0xFF bytes indicate start of an incoming packet
 * ID:              Unique ID of a Dynamixel unit. Range is 0 ~ 254 (0xFD)
 * LENGTH:          Length of the packet where its value is "Number of Parameters (N) + 2"
 * INSTURCTION:     Instruction for the Dynamixel actuator to perform
 * PARAMETER 0..N:  Used if there is more info to be sent besides instruction itself
 * CHECK SUM:       Computation for check sum is
                    Check Sum = ~(ID + LENGTH + INSTRUCTION + PARAM 1 + ... + PARAM N)
                    If the value is larger than 255, the lower byte is the checksum value
                    ~ represents the logical NOT operator
*/
