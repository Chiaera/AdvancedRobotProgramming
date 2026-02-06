/* this file contains the function for positioning the target (used in process world)
    - define the number of targets
    - pass the targets coordinate to the server
    - respawn the targets after 30 seconds
    
    - use for the watchdog
        - maps the posix shared memory (heartbeat table)
        - periodically updates its slow with monotonic timestamp
*/

#define _POSIX_C_SOURCE 200809L
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>

#include "map.h"  
#include "heartbeat.h"
#include "logger.h"

typedef struct { //for the target message, define the number of targets
    char type;  
    int num;
    Target targets[MAX_TARGETS];
} msgTargets;


//----------------------------------------------------------------------------------------------------------FUNCTION
//read the number of obstacles from the config file
static void load_config(const char *path, Config *cfg){
    memset(cfg, 0, sizeof(Config));

    //read from the file config
    FILE *f = fopen(path, "r");
    if(!f){
        fprintf(stderr, "process_targets cannot open %s, using the default values\n", path);
        return;
    }

    //parsing to define the values
    char line[256];
    while (fgets(line, sizeof(line), f)){
        char key[128], value[128];
        if (sscanf(line, "%127[^=]=%127s", key, value)==2){
            if (!strcmp(key, "WORLD_WIDTH"))  cfg->world_width  = atoi(value);
            else if (!strcmp(key, "WORLD_HEIGHT")) cfg->world_height = atoi(value);
            else if (!strcmp(key, "NUM_TARGETS")) cfg->num_targets = atoi(value);
            else if (!strcmp(key, "RELOC_PERIOD_ms")) cfg->target_reloc = atoi(value);
        }
    }
    fclose(f);
}

//update the heartbeat when the process 'sleeping' betwen the tick -> the watchdog will see the process is still active
static void sleep_with_heartbeat(HeartbeatTable *hb, int slot, uint64_t total_ms) {
    const uint64_t step_ms = 100; //100ms 

    while (total_ms > 0) {
        hb->entries[slot].last_seen_ms = now_ms();

        uint64_t cur = (total_ms > step_ms) ? step_ms : total_ms;

        struct timespec ts = { //used to sleep
            .tv_sec  = cur / 1000,
            .tv_nsec = (cur % 1000) * 1000000L
        };
        nanosleep(&ts, NULL);

        total_ms -= cur;
    }
}


//send tick to relocate targets
static void relocation_targets(int fd, const Config *cfg, int n_targets, HeartbeatTable *hb, int slot){
    while (1) {
        sleep_with_heartbeat(hb, slot, (uint64_t)cfg->target_reloc); //not used 'usleep' because we want to tells the activity during the sleep status

        sem_wait(&hb->mutex); //lock the heartbeat table
        hb->entries[slot].last_seen_ms = now_ms(); //update the slot to tell it is active
        sem_post(&hb->mutex); //unlock the heartbeat table

        msgTargets msgR;
        msgR.type = 'R';  //'R' = respawn
        msgR.num  = n_targets;
        int x,y;

        for (int i = 0; i < msgR.num; i++) {
            int valid = 0;
            while (!valid) { //random position of target until the position is valid
                valid = 1;
                x = rand() % cfg->world_width;
                y = rand() % cfg->world_height;

                for (int j = 0; j < i; j++) {
                    //check overlap with other targets
                    if (msgR.targets[j].x == x && msgR.targets[j].y == y) { 
                        valid = 0; 
                        break; 
                    }
                }
            }
            //save new position
            msgR.targets[i].x = x;
            msgR.targets[i].y = y;
        }
        log_message("TARGETS", "Targets relocated");

        ssize_t Rw = write(fd, &msgR, sizeof(msgR));
        if (Rw != sizeof(msgR)) { //debug
            perror("Failed to send relocation message of targets");
            break;  
        }
    }
}



//----------------------------------------------------------------------------------------------------------MAIN
int main(int argc, char *argv[]){
    
    //for the WATCHDOG: 
    if (argc < 4) {
        /*expected args:
            1. write_fd
            2. shm_name ('/heartbeat')
            3. slot index ('3' for HB_SLOT_TARGETS)
        */
        fprintf(stderr, "Usage: %s <write_fd> <shm_name> <slot>\n", argv[0]);
        return 1;
    }
    //read the argv
    int fd = atoi(argv[1]);
    const char *shm_name = argv[2];
    int slot = atoi(argv[3]);
    //start log
    log_message("TARGETS", "Targets process awakes (PID: %d, slot: %d)", getpid(), slot); 
    register_process("TARGETS"); //register target process pid in the pid file
    //open existing shared memory created by blackboard
    int hb_fd = shm_open(shm_name, O_RDWR, 0666);
    if (hb_fd < 0) { 
        perror("process_targets shm_open"); 
        return 1; 
    }
    //map heartbeat table
    HeartbeatTable *hb = mmap(NULL, sizeof(HeartbeatTable), PROT_READ | PROT_WRITE, MAP_SHARED, hb_fd, 0);
    if (hb == MAP_FAILED) { 
        perror("process_targets mmap"); 
        close(hb_fd); 
        return 1; 
    }
    //save PID and initialize activity
    sem_wait(&hb->mutex); //lock the heartbeat table
    //hb->entries[slot].last_seen_ms = now_ms();
    hb->entries[slot].pid = getpid();
    sem_post(&hb->mutex); //unlock the heartbeat table

    //initialize the parameters file 
    Config cfg;
    load_config("bin/parameters.config", &cfg);

    //targets messages
    msgTargets msg;
    msg.type = 'T';
    msg.num = cfg.num_targets;
    if(msg.num > MAX_TARGETS) msg.num = MAX_TARGETS; //check on the number of targets

    srand(time(NULL)^getpid()); //function to generate random values

    for(int i=0; i<msg.num; i++){ //for every targets send a message with its position
        int target_x, target_y;
        int valid_position=0;

        while(!valid_position){ //define new coordinates until the position is valid
            valid_position = 1;
            //coordinates are discrete integers in range [0, world_width) x [0, world_height)
            target_x = rand() % cfg.world_width;
            target_y = rand() % cfg.world_height;

            //check overlap with another target 
            for (int j = 0; j < i; j++) {
                if (msg.targets[j].x == target_x && msg.targets[j].y == target_y) {
                    valid_position = 0;
                    break;
                }
            }
        }
        
        //save new positions
        msg.targets[i].x = target_x;
        msg.targets[i].y = target_y;
    }
    log_message("TARGETS", "Spawned %d targets initially", msg.num);

    ssize_t written = write(fd, &msg, sizeof(msg)); //spawn the targets
    if (written != sizeof(msg)) {
        perror("write failed");
        log_message("TARGETS", "ERROR: write returned %zd", written);
    }

    relocation_targets(fd, &cfg, msg.num, hb, slot); //after tick - respawn
    close(fd); 

    log_message("TARGETS", "Targets process shutdown");

    return 0;
}