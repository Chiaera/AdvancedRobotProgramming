/* this file contains the function for the network comunication
    - send message
    - receive message
    - send ack
    - receive ack
    - coordinate conversion
*/

#include "network.h"
#include <unistd.h>
#include <string.h>
#include <stdio.h>

//--------------------------------------------------------MESSAGE
//send message
int send_msg(int fd, const char *msg) {
    char buffer[BUFFER_SIZE];
    int len = snprintf(buffer, BUFFER_SIZE, "%s\n", msg);

    int total = 0; 
    while (total < len) { //write all bytes
        int n = write(fd, buffer + total, len - total);
        if (n < 0) return -1;
        total += n;
    }
    return 0;
}

//receive message
int recv_msg(int fd, char *buffer, size_t maxlen) {
    size_t i = 0;
    char c;

    while (i < maxlen - 1) { //read leaving space for null terminator
        int n = read(fd, &c, 1);
        if(n < 0) return -1; //error
        if (n == 0) return 1; //connection closed
        if (c == '\n') break;
        buffer[i++] = c;
    }
    buffer[i] = '\0'; //null terminate
    return 0;
}


//--------------------------------------------------------ACKNOWLEDGEMENT
//send ack
int send_ack(int fd, const char *ack) {
    return send_msg(fd, ack);
}

//receive ack
int recv_ack(int fd, const char *expected) {
    char buffer[BUFFER_SIZE];
    if (recv_msg(fd, buffer, BUFFER_SIZE) < 0)
        return -1;
    if (strcmp(buffer, expected) != 0)
        return -1;
    return 0;
}

//-------------------------------------------------------- CONVERSION
void convert_to_virtual(int x, int y, int *vx, int *vy, int W, int H, Rotation rot) {
    switch (rot) {
        case ROT_0:
            *vx = x;
            *vy = y;
            break;

        case ROT_90:
            *vx = y;
            *vy = W - 1 - x;
            break;

        case ROT_180:
            *vx = W - 1 - x;
            *vy = H - 1 - y;
            break;

        case ROT_270:
            *vx = H - 1 - y;
            *vy = x;
            break;
    }
}

void convert_from_virtual(int vx, int vy, int *x, int *y, int W, int H, Rotation rot) {
    switch (rot) {
        case ROT_0:
            *x = vx;
            *y = vy;
            break;

        case ROT_90:
            *x = W - 1 - vy;
            *y = vx;
            break;

        case ROT_180:
            *x = W - 1 - vx;
            *y = H - 1 - vy;
            break;

        case ROT_270:
            *x = vy;
            *y = H - 1 - vx;
            break;

        default:
            rot = ROT_0;
    }
}
