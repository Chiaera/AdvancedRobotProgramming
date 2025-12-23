/* this file contains the function for positioning the target (used in process world)
    - define the number of targets
    - pass the targets coordinate to the server
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#include "map.h"  

typedef struct { //for the target message, define the number of targets
    char type;  
    int num;
    int x, y;
    Target targets[MAX_TARGETS];
} msgTargets;


//----------------------------------------------------------------------------------------------------------FUNCTION
//read the number of obstacles from the config file
static void load_config(const char *path, Config *cfg){
    memset(cfg, 0, sizeof(Config));

    //dafault values
    cfg->world_width = 100;
    cfg->world_height = 30;
    cfg->num_targets = 0;

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
        }
    }
    fclose(f);
}

//send tick to relocate targets
static void relocation_targets(int fd, const Config *cfg, int n_targets){
    const int RELOC_PERIOD_MS = 30000; //30 seconds

    while (1) {
        msgTargets msgR;
        msgR.type = 'R';
        msgR.num  = n_targets;

        for (int i = 0; i < msgR.num; i++) {
            int valid = 0;
            while (!valid) { //random position of target until the position is valid
                valid = 1;
                msgR.x = rand() % cfg->world_width;
                msgR.y = rand() % cfg->world_height;

                for (int j = 0; j < i; j++) {
                    //check overlap with other targets
                    if (msgR.targets[j].x == msgR.x && msgR.targets[j].y == msgR.y) { 
                        valid = 0; 
                        break; 
                    }
                }
            }
            //save new position
            msgR.targets[i].x = msgR.x;
            msgR.targets[i].y = msgR.y;
        }

        write(fd, &msgR, sizeof(msgR));
        usleep(RELOC_PERIOD_MS * 1000);
    }
}



//----------------------------------------------------------------------------------------------------------MAIN
int main(int argc, char *argv[]){
    if(argc < 2){ //check if there are arguments but the name
        fprintf(stderr, "use %s <write_fd>\n", argv[0]);
        return 1;
    }
    int fd = atoi(argv[1]); //from string to int

    //initialize the parameters file 
    Config cfg;
    load_config("bin/parameters.config", &cfg);

    //obstacles messages
    msgTargets msg;
    msg.type = 'T';
    msg.num = cfg.num_targets;
    if(msg.num > MAX_TARGETS) msg.num = MAX_TARGETS; //check on the number of targets

    srand(time(NULL)^getpid()); //function to generate random values

    for(int i=0; i<msg.num; i++){ //for every targets send a message with its position
        int target_x, target_y;
        int valid_position=0;

        while(!valid_position){ //define new coordinates until the position is valid
            //coordinates are discrete integers in range [0, world_width) x [0, world_height)
            target_x = rand() % cfg.world_width;
            target_y = rand() % cfg.world_height;

            //check overlap with another target
            int overlap_t = 0;
            for(int j=0; j<i; j++){
                if(msg.targets[j].x==target_x && msg.targets[j].y==target_y){
                    overlap_t = 1;
                    break;
                }
            }
            if(overlap_t) continue; //if the (x,y) coordinates are free, the position is valid 
            valid_position = 1;
        }

        msg.targets[i].x = target_x;
        msg.targets[i].y = target_y;
    }

    write(fd, &msg, sizeof(msg)); //spawn the targets
    relocation_targets(fd, &cfg, msg.num); //after tick - respawn
    //close(fd); - NO closure: it is needed for the tick
    return 0;
}