/* C standard library */
//gcc -o crc_test crc_test.c
#include "stdio.h"

unsigned char heartbeat[] = {
0xFE, 0x09, 0x00, 0x01, 0x01, 0x00,
0x00, 0x00, 0x00, 0x00, 
0x0A, 0x03, 0x00, 0x04, 0x03,
0x00, 0x00
};

unsigned char heartbeat_crckey = 0x32;

/*
header, length, seq, system-id, component-id, message-id,
restricted, restricted, restricted, restricted, 
type, autopilot, base_mode, custom_mode, system_status,
check_sum, check_sum
*/

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

int main(void){
    int x;

    mavlink_crc(1, heartbeat_crckey);
    for(x=0; x<sizeof(heartbeat); x++){
        printf("%02X", heartbeat[x]);
    }
    printf("\n");
}

