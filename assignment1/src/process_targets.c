#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#include "map.h"  

typedef struct {
    char type;  
    int num;
    Target targets[MAX_TARGETS];
} msgTargets;


static void load_config(const char *path, Config *cfg){
    memset(cfg, 0, sizeof(Config));

    //dafault values
    cfg->world_width = 100;
    cfg->world_height = 30;
    cfg->num_targets = 0;

    FILE *f = fopen(path, "r");
    if(!f){
        fprintf(stderr, "process_targets cannot open %s, using the default values\n", path);
        return;
    }

    //parsing
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
    msgTargets msg;
    msg.type = 'T';
    msg.num = cfg.num_targets;
    if(msg.num > MAX_TARGETS) msg.num = MAX_TARGETS;

    srand(time(NULL)^getpid());

    for(int i=0; i<msg.num; i++){
        int target_x, target_y;
        int valid_position=0;

        while(!valid_position){
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
            if(overlap_t) continue;
            valid_position = 1;
        }

        msg.targets[i].x = target_x;
        msg.targets[i].y = target_y;
    }

    write(fd, &msg, sizeof(msg));
    close(fd);
    return 0;
}