#include <unistd.h>
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
