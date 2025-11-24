#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>  
#include <sys/select.h>
#include <string.h>
#include <ncurses.h>

#include <math.h>
#include <time.h>

#include "map.h"
#include "process_input.h"
#include "process_drone.h"


typedef struct {
    char type;
    int dx, dy;
} msgInput;

typedef struct  {
    char type;
    int x, y;
} msgDrone;


// read Config file
static void load_config(const char *path, Config *cfg) {

    memset(cfg, 0, sizeof(Config));

    //default value
    cfg->mass = 1.0;
    cfg->k = 1.0;
    cfg->dt = 0.1;
    cfg->command_force = 10.0;
    cfg->drone_start_x = 0;
    cfg->drone_start_y = 0;
    cfg->target_x = 0;
    cfg->target_y = 0;
    cfg->num_obstacles = 0;
    cfg->world_width  = 100;  
    cfg->world_height = 30;
    cfg->max_force = 50;

    //debug
    FILE *f = fopen(path, "r");
    if (!f) { 
        fprintf(stderr, "Error in reading parameters.config %s\n", path); 
        fprintf(stderr, "Use default values.\n");
        return;
    }

    //parsig
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char key[128], value[128];

        if (sscanf(line, "%127[^=]=%127s", key, value) == 2) {
            //physics
            if (!strcmp(key, "MASS")) cfg->mass = atof(value);
            else if (!strcmp(key, "K")) cfg->k = atof(value);
            else if (!strcmp(key, "DT")) cfg->dt = atof(value);
            else if (!strcmp(key, "COMMAND_FORCE")) cfg->command_force = atof(value);

            //drone
            else if (!strcmp(key, "DRONE_START_X")) cfg->drone_start_x = atoi(value);
            else if (!strcmp(key, "DRONE_START_Y")) cfg->drone_start_y = atoi(value);

            //target
            else if (!strcmp(key, "TARGET_X")) cfg->target_x = atoi(value);
            else if (!strcmp(key, "TARGET_Y")) cfg->target_y = atoi(value);

            //obstacles
            else if (!strcmp(key, "NUM_OBSTACLES")) cfg->num_obstacles = atoi(value);
            else if (strncmp(key, "OBSTACLE", 8) == 0) {
                int index;
                char axis;
                if (sscanf(key, "OBSTACLE%d_%c", &index, &axis) == 2) {
                    if (axis == 'X') cfg->obstacle_x[index] = atoi(value);
                    if (axis == 'Y') cfg->obstacle_y[index] = atoi(value);
                }
            }

            //size
            else if (!strcmp(key, "WORLD_WIDTH"))  cfg->world_width  = atoi(value);
            else if (!strcmp(key, "WORLD_HEIGHT")) cfg->world_height = atoi(value);
        }
    }

    fclose(f);
}


// ----------------------------------------------------------- MAIN

int main()
{
    // ncurses
    Screen screen;
    GameState gs;
    int old_lines = LINES;
    int old_cols  = COLS;

    initscr();
    noecho();
    curs_set(0);
    srand(time(NULL));

    //parameters
    Config cfg;
    load_config("bin/parameters.config", &cfg);

    init_screen(&screen);
    init_game(&gs, &cfg);

    // pipe
    int pipe_input[2];
    int pipe_drone[2];
    pipe(pipe_input);
    pipe(pipe_drone);

    // input
    pid_t pid_input = fork();
    if(pid_input == 0){
        close(pipe_input[0]);
        set_input(pipe_input[1]);
        exit(0);
    }

    // drone
    pid_t pid_drone = fork();
    if(pid_drone == 0){
        close(pipe_drone[0]);
        move_drone(pipe_drone[1]);
        exit(0);
    }

    close(pipe_input[1]);
    close(pipe_drone[1]);

    fd_set set;
    int maxfd = (pipe_input[0] > pipe_drone[0] ? pipe_input[0] : pipe_drone[0]) + 1;
    
    while (1){
        FD_ZERO(&set);
        FD_SET(pipe_input[0], &set);
        FD_SET(pipe_drone[0], &set);

        select(maxfd, &set, NULL, NULL, NULL);

        // INPUT
        if (FD_ISSET(pipe_input[0], &set)) {
            msgInput m;
            read(pipe_input[0], &m, sizeof(m));

            if (m.type == 'Q') break;
            else if (m.type == 'I') {
                gs.fx_cmd += gs.command_force * m.dx;  
                gs.fy_cmd += gs.command_force * m.dy;

                if (gs.fx_cmd > gs.max_force) gs.fx_cmd = gs.max_force;
                if (gs.fx_cmd < -gs.max_force) gs.fx_cmd = -gs.max_force;
                if (gs.fy_cmd > gs.max_force) gs.fy_cmd = gs.max_force;
                if (gs.fy_cmd < -gs.max_force) gs.fy_cmd = -gs.max_force;
            }
            else if (m.type == 'B') {
                const double brake_factor = 0.5;

                gs.fx_cmd *= brake_factor;
                gs.fy_cmd *= brake_factor;
                gs.drone.vx *= brake_factor;
                gs.drone.vy *= brake_factor;
            }
        }

        // DRONE
        if(FD_ISSET(pipe_drone[0], &set)){
            msgDrone m;
            read(pipe_drone[0], &m, sizeof(m));

            //resultant forces
            double fx = gs.fx_cmd;
            double fy = gs.fy_cmd;

            // a = (F - k*v)/M
            double ax = (fx - gs.k*gs.drone.vx) / gs.mass;
            double ay = (fy - gs.k*gs.drone.vy) / gs.mass;

            // v = a*dt
            gs.drone.vx += ax*gs.dt;
            gs.drone.vy += ay*gs.dt;

            // Euler - position
            double new_x = gs.drone.x + gs.drone.vx*gs.dt;
            double new_y = gs.drone.y + gs.drone.vy*gs.dt;
            gs.drone.x = (int)round(new_x);
            gs.drone.y = (int)round(new_y);
        }

        if (gs.drone.x < 0) gs.drone.x = 0;
        if (gs.drone.y < 0) gs.drone.y = 0;
        if (gs.drone.x >= gs.world_width) gs.drone.x = gs.world_width - 1;
        if (gs.drone.y >= gs.world_height) gs.drone.y = gs.world_height - 1;

        //resize
        if (LINES != old_lines || COLS != old_cols) {
        old_lines = LINES;
        old_cols  = COLS;
        refresh_screen(&screen);
        }

        //debug
        mvprintw(0, 0, "M=%.2f fx=%.2f fy=%.2f vx=%.2f vy=%.2f  ", gs.mass, gs.fx_cmd, gs.fy_cmd, gs.drone.vx, gs.drone.vy);
        clrtoeol(); 
        refresh();

        render(&screen, &gs);
    }

    endwin();
    return 0;
}