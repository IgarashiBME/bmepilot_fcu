/**
 * This program is a control unit for receiving the waypoint from QGroundControl.
 * 
 * C
 *
 * @author   Sho Igarashi <igarashi@bme.en.a.u-tokyo.ac.jp>
 * @version  0.1
 * gcc -I c_library_v1/common -o uart_bmepilot uart_bmepilot.c
 * gcc -o nolib_bmepilot nolib_bmepilot.c
**/

/* C standard library */
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include "fcntl.h"
#include "string.h"
#include "termios.h"
#include "time.h"
#include "stdint.h"
//#include "mavlink.h"

#define BUFFER_LENGTH 512 // minimum buffer size that can be used with qnx (I don't know why)
#define DEV_NAME "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DN0128SM-if00-port0"
#define BAUD_RATE B57600

/* interval */
#define LIGHT_INTERVAL 100000
#define REQUEST_INTERVAL 10000

/* HEARTBEAT */
unsigned char heartbeat[] = {
0xFE, 0x09, 0x00, 0x01, 0x01, 0x00,
0x00, 0x00, 0x00, 0x00, 
0x0A, 0x03, 0x00, 0x04, 0x03,
0x00, 0x00
};

/*
header, length, seq, system-id, component-id, message-id,
restricted, restricted, restricted, restricted, 
type, autopilot, base_mode, custom_mode, system_status,
check_sum, check_sum
*/
unsigned char heartbeat_crckey = 0x32;

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

int mavlink_crc(unsigned char seq, unsigned char crc_key){
    unsigned char length = heartbeat[1];
    unsigned short sum = 0xFFFF;
    unsigned char i, stoplen;

    stoplen = length+6;
    heartbeat[2] = seq;
    heartbeat[length+6] = crc_key;
    stoplen++;

    i = 1;
    while (i<stoplen){
        unsigned char temp;
        temp = heartbeat[i] ^ (unsigned char)(sum&0xFF);
        temp ^= (temp<<4);
        sum = (sum>>8) ^ (temp<<8) ^ (temp<<3) ^ (temp>>4);
        i++;
    }
    heartbeat[length+6] = sum&0xFF;
    heartbeat[length+7] = sum>>8;
}

int main(int argc, char **argv){
    int fd;
    //mavlink_message_t msg;
    //mavlink_status_t status;

    uint64_t pre_time = 0;
    uint64_t current_time = 0;
    uint64_t time_interval = 1000000; //1 second

    /* mavlink */
    unsigned char seq = 0;

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

    uint64_t pre_request_time;
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
        int len;
        int bytes_sent;
        unsigned char buf[BUFFER_LENGTH];

        // write with time_interval
        current_time = microsSinceEpoch(); 
        if ((current_time - pre_time) > time_interval){
            //mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA, 
            //                           0, 0, MAV_STATE_ACTIVE);
            //len = mavlink_msg_to_send_buffer(buf, &msg);
            //bytes_sent = write(fd, buf, len);
            mavlink_crc(seq, heartbeat_crckey);
            int x;
            for(x=0; x<sizeof(heartbeat); x++){
                printf("%02X", heartbeat[x]);
            }
            printf("\n");
            bytes_sent = write(fd, heartbeat, sizeof(heartbeat));

            pre_time = microsSinceEpoch();
            seq++;
        }

        /* Mission Request */
        if (mission_total_seq > 0 && microsSinceEpoch() - pre_request_time > REQUEST_INTERVAL){
            pre_request_time = microsSinceEpoch();
        }

        // receive section
        //len = read(fd, buf, BUFFER_LENGTH);
        if(len <= 0){
            continue;
        }

        for(i=0; i<len; i++){
            //printf("%02X", buf[i]);
            //if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)){
            //    // Packet decode
            //    printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
            //           msg.sysid, msg.compid, msg.len, msg.msgid);
            //}
        }
        //printf("\n");
        if (seq>255){
            seq = 0;
        }
    }
}

