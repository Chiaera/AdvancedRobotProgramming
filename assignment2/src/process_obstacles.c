/* this file contains the function for positioning the obstacles (world proces)
    - define the number of obstacles
    - pass the obstales coordinate to the server
    - respawn the obstacles after 30 seconds
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#include "map.h"  

typedef struct { //for the obstacle message, define the number of obstacles
    char type;  
    int num;
    Obstacle obstacles[MAX_OBSTACLES];
} msgObstacles;


//--------------------------------------------------------------------------------------------------------FUNCTIONS
//read the number of obstacles from the config file
static void load_config(const char *path, Config *cfg){
    memset(cfg, 0, sizeof(Config));

    //read from the file config file
    FILE *f = fopen(path, "r");
    if(!f){
        fprintf(stderr, "process_obstacles cannot open %s, using the default values\n", path);
        return;
    }

    //parsing to define the values
    char line[256];
    while (fgets(line, sizeof(line), f)){
        char key[128], value[128];
        if (sscanf(line, "%127[^=]=%127s", key, value)==2){
            if (!strcmp(key, "WORLD_WIDTH"))  cfg->world_width  = atoi(value);
            else if (!strcmp(key, "WORLD_HEIGHT")) cfg->world_height = atoi(value);
            else if (!strcmp(key, "NUM_OBSTACLES")) cfg->num_obstacles = atoi(value);
            else if (!strcmp(key, "RELOC_PERIOD_ms")) cfg->obstacle_reloc = atoi(value);
        }
    }
    fclose(f);
}

//send tick to relocate obstacles
static void relocation_obstacles(int fd, const Config *cfg, int n_obstacles){
    while (1) {
        usleep(cfg-> obstacle_reloc*1000); // 30000 ms -> 30 seconds

        msgObstacles msgR;
        msgR.type = 'R'; //'R' = respawn
        msgR.num  = n_obstacles;
        int x,y;

        for (int i = 0; i < msgR.num; i++) {
            int valid = 0;
            while (!valid) { //random position of obstacle until the position is valid
                valid = 1;
                x = rand() % cfg->world_width;
                y = rand() % cfg->world_height;

                for (int j = 0; j < i; j++) {
                    //check overlap with other obstacles
                    if (msgR.obstacles[j].x == x && msgR.obstacles[j].y == y) { 
                        valid = 0; 
                        break; 
                    }
                }
            }
            //save new position
            msgR.obstacles[i].x = x;
            msgR.obstacles[i].y = y;
        }

        ssize_t Rw = write(fd, &msgR, sizeof(msgR));
        if (Rw != sizeof(msgR)) { //debug
            perror("Failed to send relocation message of obstacles");
            break;  
        }
    }
}


//--------------------------------------------------------------------------------------------------------MAIN
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
    msgObstacles msg;
    msg.type = 'O'; 
    msg.num = cfg.num_obstacles;
    if(msg.num > MAX_OBSTACLES) msg.num = MAX_OBSTACLES; //check on the number of obstacles

    srand(time(NULL)^getpid()); //function to generate random values

    for(int i=0; i<msg.num; i++){  //for every obstacles send a message with its position
        int obstacle_x, obstacle_y;
        int valid_position=0;

        while(!valid_position){ //define new coordinates until the position is valid
            //coordinates are discrete integers in range [0, world_width) x [0, world_height)
            obstacle_x = rand() % cfg.world_width;
            obstacle_y = rand() % cfg.world_height;

            //check position: if there another obstacle in that position
            int overlap_o = 0;
            for(int j=0; j<i; j++){
                if(msg.obstacles[j].x==obstacle_x && msg.obstacles[j].y==obstacle_y){
                    overlap_o = 1;
                    break;
                }
            }
            if(overlap_o) continue; //if the (x,y) coordinates are free, the position is valid 
            valid_position = 1;
        }

        msg.obstacles[i].x = obstacle_x;
        msg.obstacles[i].y = obstacle_y;
    }

    write(fd, &msg, sizeof(msg));
    relocation_obstacles(fd, &cfg, msg.num); //after tick - respawn
    close(fd);
    return 0;
}