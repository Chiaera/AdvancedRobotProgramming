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
#include "world.h"
#include "world_physics.h"

// --------------------------------------------------------------- STRUCT
typedef struct {
    char type;
    int dx, dy;
} msgInput;

typedef struct  {
    char type;
    int x, y;
} msgDrone;

typedef struct {
    char type;
    int num;
    Target targets[MAX_TARGETS];
} msgTargets;

typedef struct  {
    char type;
    int num;
    Obstacle obstacles[MAX_OBSTACLES];
} msgObstacles;


// read Config file
static void load_config(const char *path, Config *cfg) {

    memset(cfg, 0, sizeof(Config));
    
//-------------------------------------------------------------- READ CONFIG
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
            //size
            if (!strcmp(key, "WORLD_WIDTH"))  cfg->world_width  = atoi(value);
            else if (!strcmp(key, "WORLD_HEIGHT")) cfg->world_height = atoi(value);

            //physics
            else if (!strcmp(key, "MASS")) cfg->mass = atof(value);
            else if (!strcmp(key, "K")) cfg->k = atof(value);
            else if (!strcmp(key, "DT")) cfg->dt = atof(value);
            else if (!strcmp(key, "COMMAND_FORCE")) cfg->command_force = atof(value);
            else if (!strcmp(key, "MAX_FORCE")) cfg->max_force = atof(value);
            else if (!strcmp(key, "RHO")) cfg->rho = atof(value);
            else if (!strcmp(key, "ETA")) cfg->eta = atof(value);
            else if (!strcmp(key, "ZETA")) cfg->zeta = atof(value);
            else if (!strcmp(key, "TANGENT_GAIN")) cfg->tangent_gain = atof(value);            

            //drone
            else if (!strcmp(key, "DRONE_START_X")) cfg->drone_start_x = atoi(value);
            else if (!strcmp(key, "DRONE_START_Y")) cfg->drone_start_y = atoi(value);

            //target
            else if (!strcmp(key, "NUM_TARGETS")) cfg->num_targets = atoi(value);

            //obstacles
            else if (!strcmp(key, "NUM_OBSTACLES")) cfg->num_obstacles = atoi(value);
        }
    }

    fclose(f);
}

//new read of the parameters
void apply_new_parameters(GameState *gs, Config *cfg) {
    gs->mass = cfg->mass;
    gs->k = cfg->k;
    gs->dt = cfg->dt;
    gs->command_force = cfg->command_force;
    gs->max_force = cfg->max_force;
    gs->rho = cfg->rho;
    gs->eta = cfg->eta;
    gs->zeta = cfg->zeta;
    gs->tangent_gain = cfg->tangent_gain;
    gs->world_width = cfg->world_width;
    gs->world_height = cfg->world_height;
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

    start_color();
    //yellow target
    init_pair(1, COLOR_YELLOW, COLOR_BLACK);
    //magenta obstacles
    init_pair(2, COLOR_MAGENTA, COLOR_BLACK);
    //green drone
    init_pair(3, COLOR_GREEN, COLOR_BLACK);

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
    int pipe_obstacles[2];
    int pipe_targets[2];
    pipe(pipe_input);
    pipe(pipe_drone);
    pipe(pipe_obstacles);
    pipe(pipe_targets);

    // FORK ---------------------------------------------------
    // input
    pid_t pid_input = fork();
    if(pid_input == 0){
        close(pipe_input[0]); 
        char fd_str[16];
        snprintf(fd_str, sizeof(fd_str), "%d", pipe_input[1]);
        execlp("konsole",
            "konsole",
            "-e",
            "./build/bin/process_input",
            fd_str,
            (char *)NULL);
        perror("execlp process_input failed");
        exit(1);
    }

    // drone
    pid_t pid_drone = fork();
    if(pid_drone == 0){
        char fd_str[16];
        snprintf(fd_str, sizeof(fd_str), "%d", pipe_drone[1]);
        execlp(//"konsole", "konsole", "-e",
            "./build/bin/process_drone", "./build/bin/process_drone",
            fd_str,
            (char *)NULL);
        perror("execlp process_drone failed");
        exit(1);
    }

    // target
    pid_t pid_targets = fork();
    if (pid_targets == 0) {
        close(pipe_targets[0]);
        char fd_str[16];
        snprintf(fd_str, sizeof(fd_str), "%d", pipe_targets[1]);
        execlp("konsole",
            "konsole",
            "-e",
            "./build/bin/process_targets", 
            fd_str,
            (char *)NULL);
        perror("execlp process_targets failed");
        exit(1);
    }
    
    // obstacles
    pid_t pid_obstacles = fork();
    if(pid_obstacles == 0){
        char fd_str[16];
        snprintf(fd_str, sizeof(fd_str), "%d", pipe_obstacles[1]);
        execlp("konsole",
            "konsole",
            "-e",
            "./build/bin/process_obstacles",
            fd_str,
            (char *)NULL);
        perror("execlp process_obstacles failed");
        exit(1);
    }

    //close write 
    close(pipe_input[1]);
    close(pipe_drone[1]);
    close(pipe_obstacles[1]);
    close(pipe_targets[1]);

    // READ MESSAGES -------------------------------------------------------------

    // messagge by process_obstacles
    msgObstacles msg_obstacles;
    ssize_t n = read(pipe_obstacles[0], &msg_obstacles, sizeof(msg_obstacles));
    if (n == sizeof(msg_obstacles) && msg_obstacles.type == 'O') {
        gs.num_obstacles = msg_obstacles.num;
        if (gs.num_obstacles > MAX_OBSTACLES) gs.num_obstacles = MAX_OBSTACLES;

        for (int i = 0; i < gs.num_obstacles; i++) {
            gs.obstacles[i].x = msg_obstacles.obstacles[i].x;
            gs.obstacles[i].y = msg_obstacles.obstacles[i].y;
        }
    } else {
        gs.num_obstacles = 0;
    }
    // position check
    for (int i = 0; i < gs.num_obstacles; i++) {
        if (gs.obstacles[i].x == gs.drone.x && gs.obstacles[i].y == gs.drone.y) {
            respawn_obstacle(&gs, i);
        }
    }
    
    // massage by process_targets 
    msgTargets msg_t;
    ssize_t nt = read(pipe_targets[0], &msg_t, sizeof(msg_t));
    if (nt == sizeof(msg_t) && msg_t.type == 'T') {
        gs.num_targets = msg_t.num;
        if (gs.num_targets > MAX_TARGETS) gs.num_targets = MAX_TARGETS;

        for (int i = 0; i < gs.num_targets; i++) {
            gs.targets[i].x = msg_t.targets[i].x;
            gs.targets[i].y = msg_t.targets[i].y;
        }
    } else {
        gs.num_targets = 0;
    }
    close(pipe_targets[0]);
    //position check
    for (int i = 0; i < gs.num_targets; i++) {
        int tx = gs.targets[i].x;
        int ty = gs.targets[i].y;
        for (int j = 0; j < gs.num_obstacles; j++) {
            if (gs.obstacles[j].x == tx && gs.obstacles[j].y == ty) {
                respawn_target(&gs, i);
            }
        }
    }

    // PHYSICS ---------------------------------------------------------------
    // select
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

            if (m.type == 'Q') break; //quit
            else if (m.type == 'I') {  //direction
                int mx = m.dx;
                int my = m.dy;
                add_direction(&gs, mx, my);
            }
            else if (m.type == 'B') {  //brake
                use_brake(&gs);
            }/* else if (m.type == 'P'){ //read parameters
                load_config("bin/parameters.config", &cfg);
                apply_new_parameters(&gs, &cfg);
            }*/
        }

        // DRONE - drone dynamics
        if(FD_ISSET(pipe_drone[0], &set)){
            msgDrone m;
            read(pipe_drone[0], &m, sizeof(m));

            add_drone_dynamics(&gs);
        }

        //check position
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
        mvprintw(0, 0, "cmd:   fx=%.2f fy=%.2f", gs.fx_cmd, gs.fy_cmd);
        mvprintw(1, 0, "obst:  fx=%.2f fy=%.2f", gs.fx_obst, gs.fy_obst);
        mvprintw(2, 0, "fence: fx=%.2f fy=%.2f", gs.fx_fence, gs.fy_fence);
        mvprintw(3, 0, "vel:   vx=%.2f vy=%.2f", gs.drone.vx, gs.drone.vy);
        mvprintw(4, 0, "pos:   x=%6.2f y=%6.2f", gs.drone.x, gs.drone.y);
        clrtoeol();
        refresh();

        // drone - target collide 
        drone_target_collide(&gs);

        render(&screen, &gs);
    }

    endwin();
    return 0;
}