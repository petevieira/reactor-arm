#include "dynamixel.h"
#include "dynamixel_consts.h"
#include "dxl_hal.h"
#include <stdio.h>

int main(int argc, char **argv)
{
    // Open USB2Dynamixel /dev/ttyUSB0
    if (dxl_initialize(0, 1) == 0) {
        fprintf(stderr, "Failed to open USB2Dynamixel!\n");
//        return 0;
    } else {
        fprintf(stderr, "Successfully opend USB2Dynamixel!\n");
    }

    int pos = dxl_read_word(0, DXL_PRESENT_POSITION_L);
    comm_status_t status = dxl_get_result();

    if (status == DXL_COMM_RXSUCCESS) {
        fprintf(stderr, "Present Position: %d\n", pos);
    } else {
        fprintf(stderr, "STATUS: %s\n", dxl_comm_status_to_string(status));
    }

    int word = -4;
    unsigned short temp;

    temp = word & 0xff;
    fprintf(stderr, "temp: %d\n", (int)temp);

    return 0;
}
