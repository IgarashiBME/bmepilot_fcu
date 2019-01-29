/**
 * This program is a control unit for receiving the waypoint from QGroundControl.
 * 
 * C
 *
 * @author   Sho Igarashi <igarashi@bme.en.a.u-tokyo.ac.jp>
 * @version  0.1
 * gcc -I c_library_v1/common -o uart_bmepilot uart_bmepilot.c
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

    // GNSS variable
    int latitude = 34.5001348 * 10000000;  // latitude
    int longitude = 133.5591136 * 10000000;  // longitude
    int altitude = 10000;  // altitude above elliposid
    int fix_type = 0;
    int satellites = 12; // number of satellites visible. If unknown, set to 255.

    // mission variable
    unsigned int mission_total_seq = 0;
    unsigned int mission_seq = 0;
    int pre_mission_seq = -1;

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
        // Mission Request
        if (mission_total_seq > 0 && pre_mission_seq != mission_seq){
            mavlink_msg_mission_request_int_pack(1, 200, &msg, 0, 0, mission_seq);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            bytes_sent = write(fd, buf, len);
            pre_mission_seq = mission_seq;
        }

        // receive section
        len = read(fd, buf, BUFFER_LENGTH);
        if(len > 0){
            for(i=0; i<len; i++){
                printf("%02X", buf[i]);
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)){
                    // Packet decode
                    printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
                           msg.sysid, msg.compid, msg.len, msg.msgid);
                }
            }

            // mission count decoder
            if (msg.msgid == 44){
                mavlink_mission_count_t mavmc;
                printf("mission count was received\n");
                
                mavlink_msg_mission_count_decode(&msg, &mavmc);
                mission_total_seq = mavmc.count;
                mission_seq = 0;
                printf("%i\n", mission_total_seq);
            }

            /* mission receiver */
            if (msg.msgid == 73){
                mavlink_mission_item_int_t mavmii;
                //printf("mission item was received\n");

                // decode MISSION_ITEM_INT message
                mavlink_msg_mission_item_int_decode(&msg, &mavmii);
                float waypoint_x = mavmii.x/10000000.0;
                float waypoint_y = mavmii.y/10000000.0;
                printf("%i, %i, %i, %.9f, %.9f\n", mavmii.seq, mission_total_seq, mavmii.command,
                       waypoint_x, waypoint_y);
                    
                // next mission sequence
                mission_seq = mavmii.seq+1;

                // if mission sequence is end, send mission ack
                if (mission_seq == mission_total_seq){
                    mavlink_msg_mission_ack_pack(1, 200, &msg, 0, 0, MAV_MISSION_TYPE_MISSION);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    bytes_sent = write(fd, buf, len);

                    mission_total_seq = 0;
                }
            }
        }
        //printf("\n");
        pre_time = microsSinceEpoch();
    }
}

