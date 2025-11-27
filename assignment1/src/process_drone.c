#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include "process_drone.h"

typedef struct  {
    char type;
    int x, y;
} msgDrone;

void move_drone(int fd){
    while(1){
        msgDrone msg = {'D', 0, 0};
        write(fd, &msg, sizeof(msg));
        usleep(20000);
    }
}


int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <write_fd>\n", argv[0]);
        return 1;
    }

    int fd = atoi(argv[1]);
    move_drone(fd);
    close(fd);
    return 0;
}
