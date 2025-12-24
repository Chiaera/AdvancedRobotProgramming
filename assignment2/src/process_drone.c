/* this file contains the function for the drone process
    - send a message periodically to the blackboard to update the drone position

    - use for the watchdog
        - maps the posix shared memory (heartbeat table)
        - periodically updates its slow with monotonic timestamp
*/

#define _POSIX_C_SOURCE 200809L

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>      
#include <sys/mman.h>  
#include <sys/stat.h>  

#include "process_drone.h"
#include "heartbeat.h"

typedef struct  { //define the drone struct, divide the position in direction x and y
    char type;
    int x, y;
} msgDrone;

//send a tick to update the position 
void move_drone(int fd, HeartbeatTable *hb, int slot){ 

    hb->entries[slot].pid = getpid(); //save PID (used for the watchdog)

    while(1){
        hb->entries[slot].last_seen_ms = now_ms(); //tells to watchdog it is stil active

        msgDrone msg = {'D', 0, 0};
        write(fd, &msg, sizeof(msg));
        //used for the 'nanosleep' function
        struct timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = 20 * 1000 * 1000; // 20 ms
        nanosleep(&ts, NULL);
    }
}


int main(int argc, char *argv[])
{
    if (argc < 4) { 
        /*expected args:
            1. write_fd
            2. shm_name ('/heartbeat')
            3. slot index ('2' for HB_SLOT_DRONE)
        */
        fprintf(stderr, "Usage: %s <write_fd> <shm_name> <slot>\n", argv[0]);
        return 1;
    }

    //read the argv
    int fd = atoi(argv[1]);
    const char *shm_name = argv[2];
    int slot = atoi(argv[3]);

    //open existing shared memory created by blackboard
    int hb_fd = shm_open(shm_name, O_RDWR, 0666);
    if (hb_fd < 0) { 
        perror("process_drone shm_open"); 
        return 1; 
    }

    //map heartbeat table
    HeartbeatTable *hb = mmap(NULL, sizeof(HeartbeatTable), PROT_READ | PROT_WRITE, MAP_SHARED, hb_fd, 0);
    if (hb == MAP_FAILED) { 
        perror("process_drone mmap"); 
        close(hb_fd); 
        return 1; 
    }

    move_drone(fd, hb, slot); //update position

    munmap(hb, sizeof(*hb));
    close(hb_fd);
    close(fd);
    return 0;
}
