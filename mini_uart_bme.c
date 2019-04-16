/**
 * This program is a control unit for receiving the waypoint from QGroundControl.
 * 
 * C
 *
 * @author   Sho Igarashi <igarashi@bme.en.a.u-tokyo.ac.jp>
 * @version  0.1
 * gcc -I c_library_v1/common -o mini_uart mini_uart_bme.c
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

#define BUFFER_LENGTH 512 // minimum buffer size that can be used with qnx (I don't know why)
#define DEV_NAME "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DN0129JR-if00-port0"
//#define DEV_NAME "/dev/serial/by-id/usb-TOCOS_TWE-Lite-USB_AHY1WO3U-if00-port0"
#define BAUD_RATE B57600

/* interval */
#define ONESEC_INTERVAL 1000000
#define LIGHT_INTERVAL 100000
#define REQUEST_INTERVAL 50000

void serial_init(int fd){
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_cc[VTIME] = 0;
    //tio.c_cc[VMIN] = 10;
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
    long int pre_time = microsSinceEpoch();
    long int current_time;
    long int start_time = microsSinceEpoch();
    long int time_interval = 1000000; //1 second
    mavlink_message_t msg;
    uint8_t test;
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

    uint64_t pre_request_time = microsSinceEpoch();
    int request_interval = 30000; // 0.03 second

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
        int x;
        int len;
        int bytes_sent;
        unsigned char buf[BUFFER_LENGTH];

        // heartbeat time_interval
        current_time = microsSinceEpoch() - start_time;

        //printf("%ld\n", current_time % ONESEC_INTERVAL);
        if (current_time % ONESEC_INTERVAL == 0){
            //mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA, 
            //                           0, 0, MAV_STATE_ACTIVE);
            //len = mavlink_msg_to_send_buffer(buf, &msg);
            //bytes_sent = write(fd, buf, len);

            mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA, 
                                       0, 0, MAV_STATE_ACTIVE);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            bytes_sent = write(fd, buf, len);

            printf("send: ");
            for(x=0; x<len; x++){
                printf("%02x", buf[x]);
            }
            printf(" (HEARTBEAT)\n");

            /* Send Status */
            //mavlink_msg_sys_status_pack(1, 200, &msg, 1, 1, 1, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
            //len = mavlink_msg_to_send_buffer(buf, &msg);
            //bytes_sent = write(fd, buf, len);

            /* Send GPS */
            //mavlink_msg_gps_raw_int_pack(1, 200, &msg, 0, GPS_FIX_TYPE_RTK_FIXED, 
            //                             latitude, longitude, altitude, 65535, 65535, 
            //                             65535, 65535, satellites);
            //len = mavlink_msg_to_send_buffer(buf, &msg);
            //bytes_sent = write(fd, buf, len);
        }

        /* Mission Request */
        if (mission_total_seq > 0 && current_time % REQUEST_INTERVAL == 0){
            //printf("request\n %i", mission_seq);
            mavlink_msg_mission_request_int_pack(1, 200, &msg, 0, 0, mission_seq);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            bytes_sent = write(fd, buf, len);

            printf("send: ");
            for(x=0; x<len; x++){
                printf("%02x", buf[x]);
            }
            printf(" (MISSION_REQUEST)\n");
        }

        // receive section
        len = read(fd, buf, BUFFER_LENGTH);
        if(len <= 0){
            continue;
        }

        for(i=0; i<len; i++){
            //usleep(1000);
            //printf("%02x", buf[i]);
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)){
                // Packet decode
                //printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
                //       msg.sysid, msg.compid, msg.len, msg.msgid);
            }
            else {
                continue;
            }
            //printf("%i\n", buf[i]);

            printf("recv: ");
            for(x=i-msg.len-7; x<i+1; x++){
                printf("%02x", buf[x]);
            }
            if (msg.msgid == 0x00){
                printf(" (QGC HEARTBEAT)\n");
            }
        }
        printf("\n");

        switch (msg.msgid){
            // mission count decoder
            case 44:{
                mavlink_mission_count_t mavmc;
            
                mavlink_msg_mission_count_decode(&msg, &mavmc);
                mission_total_seq = mavmc.count;
                mission_seq = 0;
                printf("mission count was received\n");
                printf("total mission_sequence: %i\n", mission_total_seq);
                break;
            }

            /* mission receiver */
            case 73:{
                mavlink_mission_item_int_t mavmii;
                //printf("mission item was received\n");
                // decode MISSION_ITEM_INT message
                mavlink_msg_mission_item_int_decode(&msg, &mavmii);
                long int waypoint_x = mavmii.x;
                long int waypoint_y = mavmii.y;

                if (pre_mission_seq != mavmii.seq && mission_seq == mavmii.seq){
                    printf("decode: seq:%i, total_seq:%i, command:%i, lat:%ld, lon:%ld\n\n", mavmii.seq, mission_total_seq, mavmii.command,
                           waypoint_x, waypoint_y);
                    //printf("%i, %.09f, %.09f\n", mavmii.seq, waypoint_x, waypoint_y);
                    pre_mission_seq = mavmii.seq;
                    mission_seq = mavmii.seq+1;
                }
 
                // if mission sequence is end, send mission ack
                if (mission_seq == mission_total_seq){
                    mavlink_msg_mission_ack_pack(1, 200, &msg, 0, 0, MAV_MISSION_TYPE_MISSION);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    bytes_sent = write(fd, buf, len);

                    printf("send: ");
                    for(x=0; x<len; x++){
                        printf("%02x", buf[x]);
                    }
                    printf(" (MISSION_ACK)\n");

                    mission_total_seq = 0;
                }
                break;
            }

            /*case 76:{
                mavlink_command_long_t mavcl;
                // decode SET_MODE message
                mavlink_msg_command_long_decode(&msg, &mavcl);
                printf("%i, %i, %f, %f", mavcl.command, mavcl.confirmation, mavcl.param1, mavcl.param2);
                if (mavcl.command == MAV_CMD_REQUEST_PROTOCOL_VERSION){
                    mavlink_msg_command_ack_pack(1, 1, &msg, mavcl.command,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    bytes_sent = write(fd, buf, len);
                }
                if (mavcl.command == MAV_CMD_REQUEST_PROTOCOL_VERSION){
                    mavlink_msg_command_ack_pack(1, 1, &msg, mavcl.command,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    bytes_sent = write(fd, buf, len);

                    mavlink_msg_autopilot_version_pack(1, 1, &msg, MAV_PROTOCOL_CAPABILITY_MISSION_INT,
                                                       1, 1, 1, 1, &test, &test, &test, 1, 1, 1);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    bytes_sent = write(fd, buf, len);
                    break;
                }
            } */           
        }
        //printf("\n");
    }
}

