/**
 * This program is a control unit for receiving the waypoint from QGroundControl.
 * 
 * C
 *
 * @author   Sho Igarashi <igarashi@bme.en.a.u-tokyo.ac.jp>
 * @version  0.1
 * gcc -I c_library_v1/common -o uart_bmepilot uart_bmepilot.c
 * gcc -o minimum minimum_nolib.c
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
#define DEV_NAME "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DN0129JR-if00-port0"
#define BAUD_RATE B57600

/* interval */
#define LIGHT_INTERVAL 100000
#define REQUEST_INTERVAL 10000

/* HEARTBEAT */
unsigned char heartbeat[] = {
0xfe, 0x09, 0x00, 0x01, 0x01, 0x00,
0x00, 0x00, 0x00, 0x00, 
0x0a, 0x03, 0x00, 0x04, 0x03,
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
    tio.c_cc[VTIME] = 1;
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

char *mavlink_crc(unsigned char data[], unsigned char seq, unsigned char crc_key, unsigned char* test){
    test = data;
    unsigned char length = test[1];
    unsigned short sum = 0xffff;
    unsigned char i, stoplen;

    stoplen = length+6;
    test[2] = seq;
    test[length+6] = crc_key;
    stoplen++;

    i = 1;
    while (i<stoplen){
        unsigned char temp;
        temp = test[i] ^ (unsigned char)(sum&0xff);
        temp ^= (temp<<4);
        sum = (sum>>8) ^ (temp<<8) ^ (temp<<3) ^ (temp>>4);
        i++;
    }
    test[length+6] = sum&0xff;
    test[length+7] = sum>>8;

    int x;
    //for(x=0; x<test[1]+8; x++){
    //    printf("%02x", test[x]);
    //}
    //printf("\n");
}

int main(int argc, char **argv){
    int fd;
    unsigned int temp = 0;

    uint64_t pre_time = 0;
    uint64_t current_time = 0;
    uint64_t time_interval = 1000000; //1 second

    /* mavlink */
    unsigned char seq = 0;

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
        unsigned char* test[BUFFER_LENGTH];

        // write with time_interval
        current_time = microsSinceEpoch(); 
        if ((current_time - pre_time) > time_interval){
            mavlink_crc(heartbeat, seq, heartbeat_crckey, test);
            int x;
            printf("send: ");
            for(x=0; x<test[1]+8; x++){
                printf("%02x", test[x]);
            }
            printf(" (HEARTBEAT)\n");
            bytes_sent = write(fd, test, sizeof(test));

            pre_time = microsSinceEpoch();
            seq++;
        }

        /* Mission Request */
        if (mission_total_seq > 0 && microsSinceEpoch() - pre_request_time > REQUEST_INTERVAL){
            pre_request_time = microsSinceEpoch();
        }

        // receive section
        len = read(fd, buf, BUFFER_LENGTH);
        if(len <= 0){
            continue;
        }

        printf("recv: ");
        for(i=0; i<len; i++){
            temp = buf[i];
            /* Packet received */
            printf("%02x", (unsigned char)temp);
        }
        if (buf[0] == 0xfe && buf[1] == 0x09 && buf[5] == 0x00){
            printf(" (QGC HEARTBEAT)\n");
            mavlink_crc(buf, buf[2], heartbeat_crckey, test);

            int x;
            for(x=0; x<test[1]+7; x++){
                //printf("%02x", test[x]);
            }            
            unsigned char buf[BUFFER_LENGTH];
            printf("\n");
        }
        else if (buf[0] == 0xfe && buf[1] == 0x04 && buf[5] == 0x2c){
            printf(" (MISSION_COUNT)\n");
            unsigned char buf[BUFFER_LENGTH];
        }
        else{
            printf("\n");
        }
        if (seq>255){
            seq = 0;
        }
    }
}

