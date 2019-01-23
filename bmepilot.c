/**
 * This program is a control unit for receiving the waypoint from QGroundControl.
 * 
 * C
 *
 * @author   Sho Igarashi <igarashi@bme.en.a.u-tokyo.ac.jp>
 * @version  0.1
 * gcc -I c_library_v1/common -o bme_pilot bme_pilot.c
**/

/* C standard library */
#include "stdio.h"
#include "errno.h"
#include "string.h"
#include "sys/socket.h"
#include "sys/types.h"
#include "netinet/in.h"
#include "unistd.h"
#include "stdlib.h"
#include "fcntl.h"
#include "time.h"
#include "sys/time.h"
#include "arpa/inet.h"

/* mavlink library */
#include "mavlink.h"

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

uint64_t microsSinceEpoch();

int main(int argc, char **argv){
    char target_ip[100];
	
    float position[6] = {};
    int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in gcAddr; 
    struct sockaddr_in locAddr;
    uint8_t buf[BUFFER_LENGTH];
    ssize_t recsize;
    socklen_t fromlen;
    int bytes_sent;
    mavlink_message_t mavmsg;
    uint16_t len;
    int i = 0;
    unsigned int temp = 0;
    unsigned int mission_total_seq = 0;
    unsigned int mission_seq = 0;
    int pre_mission_seq = -1;
    uint64_t pre_time = -1000000;
    int time_interval = 900000; //0.9 second
    int mav_mode;
    mav_mode = MAV_MODE_GUIDED_DISARMED;
    int custom_mode;
    custom_mode = 0;

    int latitude = 35.717736 * 10000000;  // latitude
    int longitude = 139.759512 * 10000000;  // longitude
    int altitude = 10000;  // altitude above elliposid
    int fix_type = 0;
    int satellites = 12; // number of satellites visible. If unknown, set to 255.

    // Change the target ip if parameter was given
    strcpy(target_ip, "127.0.0.1");
    if (argc == 2){
        strcpy(target_ip, argv[1]);
    }

    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(14551);
	
    /* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
    if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr))){
        perror("error bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    } 
	
    /* Attempt to make it non blocking */
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0){
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sock);
        exit(EXIT_FAILURE);
    }

    memset(&gcAddr, 0, sizeof(gcAddr));
    gcAddr.sin_family = AF_INET;
    gcAddr.sin_addr.s_addr = inet_addr(target_ip);
    gcAddr.sin_port = htons(14550);

    pre_time = microsSinceEpoch();
    for(;;){

        /* time interval */        
        if (microsSinceEpoch() - pre_time > time_interval){
            /* time print */
            //std::cout << microsSinceEpoch() - pre_time << std::endl;
            //std::cout << mav_mode << std::endl;
            //printf("%i\n", mav_mode);
            /*Send Heartbeat */
            // https://github.com/mavlink/c_library_v1/blob/3da9db30f3ea7fe8fa8241a74ab343b9971e7e9a/common/common.h#L166
            //mavlink_msg_heartbeat_pack(1, 1, &mavmsg, type, autopilot, 
            //                           base_mode, custom_mode, system_status);
            //mavlink_msg_heartbeat_pack(1, 1, &mavmsg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 
            //                           mav_mode, 3, MAV_STATE_STANDBY);
            mavlink_msg_heartbeat_pack(1, 1, &mavmsg, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA, 
                                       mav_mode, custom_mode, MAV_STATE_ACTIVE);
            //mavlink_msg_heartbeat_pack(1, 1, &mavmsg, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA, 
            //                           mav_mode, 0, MAV_STATE_STANDBY);

            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
            /* Send Status */
            mavlink_msg_sys_status_pack(1, 200, &mavmsg, 1, 1, 1, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));

            /* Send Local Position */
            mavlink_msg_local_position_ned_pack(1, 200, &mavmsg, microsSinceEpoch(), 
                                                position[0], position[1], position[2],
                                                position[3], position[4], position[5]);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
            /* Send attitude */ 
            mavlink_msg_attitude_pack(1, 200, &mavmsg, microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
            /* Send GPS */
            mavlink_msg_gps_raw_int_pack(1, 200, &mavmsg, 0, GPS_FIX_TYPE_RTK_FIXED, 
                                         latitude, longitude, altitude, 65535, 65535, 
                                         65535, 65535, satellites);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

            pre_time = microsSinceEpoch();
        }

        // Mission Request
        if (mission_total_seq > 0 && pre_mission_seq != mission_seq){
            mavlink_msg_mission_request_int_pack(1, 200, &mavmsg, 0, 0, mission_seq);
            len = mavlink_msg_to_send_buffer(buf, &mavmsg);
            bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
            pre_mission_seq = mission_seq;
        }

        /* receiver section */
        memset(buf, 0, BUFFER_LENGTH);
        recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);

        if (recsize > 0){
            // Something received - print out all bytes and parse packet
            mavlink_message_t mavmsg;
            mavlink_status_t status;

            //printf("Bytes Received: %d\nDatagram: ", (int)recsize);

            for (i = 0; i < recsize; ++i){
                temp = buf[i];
                /* Packet received */
                //printf("%02x ", (unsigned char)temp);
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mavmsg, &status)){
                    // Packet decode
                    printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
                           mavmsg.sysid, mavmsg.compid, mavmsg.len, mavmsg.msgid);
                }
            }
            if (mavmsg.msgid == 44){
                mavlink_mission_count_t mavmc;
                printf("mission count was received\n");
                    
                mavlink_msg_mission_count_decode(&mavmsg, &mavmc);
                mission_total_seq = mavmc.count;
                mission_seq = 0;
                printf("%i\n", mission_total_seq);
            }

            /* mission receiver */
            if (mavmsg.msgid == 73){
                mavlink_mission_item_int_t mavmii;

                //printf("mission item was received\n");

                // decode MISSION_ITEM_INT message
                mavlink_msg_mission_item_int_decode(&mavmsg, &mavmii);
                float waypoint_x = mavmii.x/10000000.0;
                float waypoint_y = mavmii.y/10000000.0;
                printf("%i, %i, %i, %f, %f\n", mavmii.seq, mission_total_seq, mavmii.command,
                       waypoint_x, waypoint_y);
                    
                // output waypoint
                //if (mavmii.command == 16){
                //    fstream fs;
                //    fs.open("/home/nouki/waypoint.csv", ios::out | ios::app);
                //    fs << mission_seq << ",";
                //    fs << fixed << setprecision(8) << waypoint_x << "," << waypoint_y << endl; 
                //    fs.close();
                //}

                // next waypoint
                mission_seq = mavmii.seq+1;

                // if mission sequence is end, send mission ack
                if (mission_seq == mission_total_seq){
                    mavlink_msg_mission_ack_pack(1, 200, &mavmsg, 0, 0, MAV_MISSION_TYPE_MISSION);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

                    mission_total_seq = 0;
                }
            }
            /* SET_MODE decoder */
            if (mavmsg.msgid == 11){
                mavlink_set_mode_t mavsm;

                // decode SET_MODE message
                mavlink_msg_set_mode_decode(&mavmsg, &mavsm);
                printf("%i, %i, %i", mavsm.custom_mode, mavsm.target_system, mavsm.base_mode);
                if (mavsm.base_mode == mavsm.base_mode){
                    mav_mode = mavsm.base_mode;
                }
                custom_mode = mavsm.custom_mode;
            }

            /* COMMAND_LONG decoder */
            if (mavmsg.msgid == 76){
                mavlink_command_long_t mavcl;

                // decode SET_MODE message
                mavlink_msg_command_long_decode(&mavmsg, &mavcl);
                printf("%i, %i, %f, %f", mavcl.command, mavcl.confirmation, mavcl.param1, mavcl.param2);

                /* Send COMMAND_ACK */ 
                if (mavcl.command == MAV_CMD_COMPONENT_ARM_DISARM && mavcl.param1 == 1.0){
                    mavlink_msg_command_ack_pack(1, 200, &mavmsg, MAV_CMD_COMPONENT_ARM_DISARM,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
                    mav_mode = MAV_MODE_GUIDED_ARMED;
                }
                if (mavcl.command == MAV_CMD_COMPONENT_ARM_DISARM && mavcl.param1 == 0.0){
                    mavlink_msg_command_ack_pack(1, 200, &mavmsg, MAV_CMD_COMPONENT_ARM_DISARM,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
                    mav_mode = MAV_MODE_GUIDED_DISARMED;
                }
                if (mavcl.command == MAV_CMD_REQUEST_PROTOCOL_VERSION){
                    mavlink_msg_command_ack_pack(1, 200, &mavmsg, mavcl.command,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
                }
                if (mavcl.command == MAV_CMD_MISSION_START){
                    mavlink_msg_command_ack_pack(1, 200, &mavmsg, mavcl.command,
                                                 MAV_RESULT_ACCEPTED);
                    len = mavlink_msg_to_send_buffer(buf, &mavmsg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
                }
            }                
            //printf("\n");
        }
        memset(buf, 0, BUFFER_LENGTH);
        usleep(1000); // Sleep 10 msec
        //sleep(1);
    }
}

uint64_t microsSinceEpoch(){
    struct timeval tv;
    uint64_t micros = 0;

    gettimeofday(&tv, NULL);  
    micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

    return micros;
}
