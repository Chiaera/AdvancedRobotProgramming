/* this file contains the function for the network comunication
    - send message
    - receive message
    - send ack
    - receive ack
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
int recv_msg(int fd, char *buffer, int maxlen) {
    int i = 0;
    char c;

    while (i < maxlen - 1) { //leave space for null terminator
        int n = read(fd, &c, 1);
        if (n <= 0) return -1;
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
