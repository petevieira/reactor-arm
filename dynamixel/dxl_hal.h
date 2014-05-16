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

/**
 * \file dxl_hal.h
 * \brief Dynamixel Hardware Abstraction Layer
 */

#ifndef DYNAMIXEL_HAL_H
#define DYNAMIXEL_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Opens a connection with the serial device using
 * the deviceIndex and baudrate as parameters
 * \param deviceIndex The "#" in ttyUSB# device name
 * \param baudrate Baudrate in bits/sec that the device communicates at
 * \return 1 on success, 0 on failure
 */
int dxl_hal_open(int deviceIndex, float baudrate);

/**
 * \brief Closes the COM port
 */
void dxl_hal_close();

/**
 * \brief Sets the baud rate
 */
int dxl_hal_set_baud(float baudrate);

/**
 * \brief Clears the communication buffer
 */
void dxl_hal_clear();

/**
 * \brief Sends a packet of data of numPacket bytes
 * \param pPacket Packet to send
 * \param numPacket Packet size in bytes
 * \return Positive number on success, -1 on failure
 */
int dxl_hal_tx(unsigned char *pPacket, int numPacket);

/**
 * \brief Receive a packet of data of numPacket bytes
 * \param pPacket Packet to receive
 * \param numPacket Packet size in bytes
 * \return
 */
int dxl_hal_rx(unsigned char *pPacket, int numPacket);

/**
 * \brief Sets the max waiting time for a given number
 * of bytes to be received
 * \param NumRcvByte Number of bytes to be received
 */
void dxl_hal_set_timeout(int NumRcvByte);

/**
 * \brief Checks for timeout during recieve operation
 * \return 1 if timedout, 0 if not
 */
int dxl_hal_timeout();


#ifdef __cplusplus
}
#endif

#endif
