#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#include "map.h"  

typedef struct {
    char type;  
    int num;
    Obstacle obstacles[MAX_OBSTACLES];
} msgObstacles;


static void load_config(const char *path, Config *cfg){
    memset(cfg, 0, sizeof(Config));

    //dafault values
    cfg->world_width = 100;
    cfg->world_height = 30;
    cfg->num_obstacles = 0;

    FILE *f = fopen(path, "r");
    if(!f){
        fprintf(stderr, "process_obstacles cannot open %s, using the default values\n", path);
        return;
    }

    //parsing
    char line[256];
    while (fgets(line, sizeof(line), f)){
        char key[128], value[128];
        if (sscanf(line, "%127[^=]=%127s", key, value)==2){
            if (!strcmp(key, "WORLD_WIDTH"))  cfg->world_width  = atoi(value);
            else if (!strcmp(key, "WORLD_HEIGHT")) cfg->world_height = atoi(value);
            else if (!strcmp(key, "NUM_OBSTACLES")) cfg->num_obstacles = atoi(value);
        }
    }
    fclose(f);
}


int main(int argc, char *argv[]){
    if(argc < 2){
        fprintf(stderr, "use %s <write_fd>\n", argv[0]);
        return 1;
    }
    int fd = atoi(argv[1]);

    //parameters
    Config cfg;
    load_config("bin/parameters.config", &cfg);

    //obstacles messages
    msgObstacles msg;
    msg.type = 'O';
    msg.num = cfg.num_obstacles;
    if(msg.num > MAX_OBSTACLES) msg.num = MAX_OBSTACLES;

    srand(time(NULL)^getpid());

    for(int i=0; i<msg.num; i++){
        int obstacle_x, obstacle_y;
        int valid_position=0;

        while(!valid_position){
            obstacle_x = rand() % cfg.world_width;
            obstacle_y = rand() % cfg.world_height;

            //check position
            int overlap_o = 0;
            for(int j=0; j<i; j++){
                if(msg.obstacles[j].x==obstacle_x && msg.obstacles[j].y==obstacle_y){
                    overlap_o = 1;
                    break;
                }
            }
            if(overlap_o) continue;
            valid_position = 1;
        }

        msg.obstacles[i].x = obstacle_x;
        msg.obstacles[i].y = obstacle_y;
    }

    write(fd, &msg, sizeof(msg));
    close(fd);
    return 0;
}