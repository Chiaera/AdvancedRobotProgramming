/* this file contains the function for the drone process
    - send a message every 0.2 s to update the drone position
*/

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include "process_drone.h"

typedef struct  { //define the drone struct, divide the position in direction x and y
    char type;
    int x, y;
} msgDrone;

//send a tick to update the position
void move_drone(int fd){ 
    while(1){
        msgDrone msg = {'D', 0, 0};
        write(fd, &msg, sizeof(msg));
        usleep(20000);
    }
}


int main(int argc, char *argv[])
{
    if (argc < 2) { //check to see if there is another argument but the name
        fprintf(stderr, "Usage: %s <write_fd>\n", argv[0]);
        return 1;
    }

    int fd = atoi(argv[1]); //from string to integer
    move_drone(fd); //update position
    close(fd);
    return 0;
}
