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
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include "dxl_hal.h"

int     gSocket_fd	= -1;
long	glStartTime	= 0;
float	gfRcvWaitTime	= 0.0f;
float	gfByteTransTime	= 0.0f;
char	gDeviceName[20];

int dxl_hal_open(int deviceIndex, float baudrate)
{
	struct termios newtio;
	struct serial_struct serinfo;
	char dev_name[100] = {0, };

    // Set the device name and check for failure
    if (sprintf(dev_name, "/dev/ttyUSB%d", deviceIndex) < 0) {
        fprintf(stderr, "device name buffer overflow\n");
        return 0;
    }

	strcpy(gDeviceName, dev_name);
	memset(&newtio, 0, sizeof(newtio));
	dxl_hal_close();
	
    // Open the device for read/write (O_RDWR), don't let terminal control
    // process (O_NOCTTY), and non-blocking (O_NONBLOCK)
	if((gSocket_fd = open(gDeviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
		fprintf(stderr, "device open error: %s\n", dev_name);
        dxl_hal_close();
        return 0;
	}

    // Set baudrate (B38400), char size mask (CS8),
    // ignore modem control lines (CLOCAL), enable receiver (CREAD),
    // ignore framing errors and parity errors (IGNPAR)
    // VTIME = 0 and VMIN = 0 sets reading to "polling read"
    newtio.c_cflag		= B38400|CS8|CLOCAL|CREAD;  // control modes
    newtio.c_iflag		= IGNPAR;                   // input modes
    newtio.c_oflag		= 0;                        // output modes
    newtio.c_lflag		= 0;                        // local modes
    newtio.c_cc[VTIME]	= 0;    // Timeout in deciseconds for non-canonical read
    newtio.c_cc[VMIN]	= 0;    // Min # of chars for non-canonical read

    // Flush pending data on gSocket_fd file descriptor and set the state
    // of the file descriptor to TERMIO_P (raw mode)
	tcflush(gSocket_fd, TCIFLUSH);
    tcsetattr(gSocket_fd, TCSANOW, &newtio);
	
	if(gSocket_fd == -1)
		return 0;
	
    // Perform the I/O control operation specified by REQUEST on FD
	if(ioctl(gSocket_fd, TIOCGSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot get serial info\n");
		return 0;
	}
	
    // Set serial flags
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = serinfo.baud_base / baudrate;
	
    // Perform the I/O control operation specified by REQUEST on FD
	if(ioctl(gSocket_fd, TIOCSSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot set serial info\n");
		return 0;
	}
	
    // Close the connection
	dxl_hal_close();
	
	gfByteTransTime = (float)((1000.0f / baudrate) * 12.0f);
	
	strcpy(gDeviceName, dev_name);
	memset(&newtio, 0, sizeof(newtio));
	dxl_hal_close();
	
	if((gSocket_fd = open(gDeviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
		fprintf(stderr, "device open error: %s\n", dev_name);
        dxl_hal_close();
        return 0;
	}

    // Set baudrate (B38400), char size mask (CS8),
    // ignore modem control lines (CLOCAL), enable receiver (CREAD),
    // ignore framing errors and parity errors (IGNPAR)
    // VTIME = 0 and VMIN = 0 sets reading to "polling read"
    newtio.c_cflag		= B38400|CS8|CLOCAL|CREAD;  // control modes
    newtio.c_iflag		= IGNPAR;                   // input modes
    newtio.c_oflag		= 0;                        // output modes
    newtio.c_lflag		= 0;                        // local modes
    newtio.c_cc[VTIME]	= 0;    // Timeout in deciseconds for non-canonical read
    newtio.c_cc[VMIN]	= 0;    // Min # of chars for non-canonical read

    // Flush pending data on gSocket_fd file descriptor and set the state
    // of the file descriptor to TERMIO_P (raw mode)
	tcflush(gSocket_fd, TCIFLUSH);
	tcsetattr(gSocket_fd, TCSANOW, &newtio);
	
	return 1;
}

void dxl_hal_close()
{
    // Close the connection
	if(gSocket_fd != -1)
		close(gSocket_fd);
	gSocket_fd = -1;
}

int dxl_hal_set_baud( float baudrate )
{
	struct serial_struct serinfo;
	
	if(gSocket_fd == -1)
		return 0;
	
	if(ioctl(gSocket_fd, TIOCGSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot get serial info\n");
		return 0;
	}
	
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = serinfo.baud_base / baudrate;
	
	if(ioctl(gSocket_fd, TIOCSSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot set serial info\n");
		return 0;
	}
	
	//dxl_hal_close();
	//dxl_hal_open(gDeviceName, baudrate);
	
	gfByteTransTime = (float)((1000.0f / baudrate) * 12.0f);
	return 1;
}

void dxl_hal_clear(void)
{
	tcflush(gSocket_fd, TCIFLUSH);
}

int dxl_hal_tx( unsigned char *pPacket, int numPacket )
{
	return write(gSocket_fd, pPacket, numPacket);
}

int dxl_hal_rx( unsigned char *pPacket, int numPacket )
{
	memset(pPacket, 0, numPacket);
	return read(gSocket_fd, pPacket, numPacket);
}

static inline long myclock()
{
	struct timeval tv;
	gettimeofday (&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

void dxl_hal_set_timeout( int NumRcvByte )
{
	glStartTime = myclock();
	gfRcvWaitTime = (float)(gfByteTransTime*(float)NumRcvByte + 5.0f);
}

int dxl_hal_timeout(void)
{
	long time;
	
	time = myclock() - glStartTime;
	
	if(time > gfRcvWaitTime)
		return 1;
	else if(time < 0)
		glStartTime = myclock();
		
	return 0;
}
