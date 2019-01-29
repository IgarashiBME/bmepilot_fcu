/**
 * This program is a control unit for receiving the waypoint from QGroundControl.
 * 
 * C
 *
 * @author   Sho Igarashi <igarashi@bme.en.a.u-tokyo.ac.jp>
 * @version  0.1
 * gcc -I c_library_v1/common -o uart uart_test.c
**/

/* C standard library */
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include "fcntl.h"
#include "string.h"
#include "termios.h"
#include "time.h"

/* mavlink library */
#include "mavlink.h"

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
#define DEV_NAME "/dev/ttyUSB0"
#define BAUD_RATE B57600

void serial_init(int fd){
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_cc[VTIME] = 100;
    // baud rate setting
    cfsetispeed(&tio, BAUD_RATE);
    cfsetospeed(&tio, BAUD_RATE);

    tcsetattr(fd, TCSANOW, &tio);
}

uint64_t microsSinceEpoch(){
    struct timeval tv;
    uint64_t micros = 0;

    gettimeofday(&tv, NULL);  
    micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

    return micros;
}

int main(int argc, char **argv){
    int fd;
    int pre_time = 0;
    int time_interval = 1000000; //1 second
    mavlink_message_t msg;
    mavlink_status_t status;

    int latitude = 34.5001348 * 10000000;  // latitude
    int longitude = 133.5591136 * 10000000;  // longitude
    int altitude = 10000;  // altitude above elliposid
    int fix_type = 0;
    int satellites = 12; // number of satellites visible. If unknown, set to 255.

    // device open
    fd = open(DEV_NAME, O_RDWR);
    if(fd<0){
        perror(argv[1]);
        exit(1);
    }

    //serial initialize
    serial_init(fd);

    // loop
    while(1){
        int i;
        int len;
        int bytes_sent;
        unsigned char buf[BUFFER_LENGTH];

        // write with time_interval
        if (microsSinceEpoch() - pre_time > time_interval){
            mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA, 
                                       0, 0, MAV_STATE_ACTIVE);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            bytes_sent = write(fd, buf, len);

            /* Send GPS */
            mavlink_msg_gps_raw_int_pack(1, 200, &msg, 0, GPS_FIX_TYPE_RTK_FIXED, 
                                         latitude, longitude, altitude, 65535, 65535, 
                                         65535, 65535, satellites);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            bytes_sent = write(fd, buf, len);
        }

        // receive
        len = read(fd, buf, BUFFER_LENGTH);
        if(len==0){
            continue;
        }
        if(len<0){
            perror("");
            exit(2);
        }
        for(i=0; i<len; i++){
            printf("%02X", buf[i]);
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)){
                // Packet decode
                printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
                       msg.sysid, msg.compid, msg.len, msg.msgid);
            }
        }
        printf("\n");
        pre_time = microsSinceEpoch();
    }
}

