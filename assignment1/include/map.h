#ifndef MAP_H
#define MAP_H

#define MAX_OBSTACLES 20
#define MAX_TARGETS 10

#include <ncurses.h>

//------------------------------------------------------------------------STRUCTS

// Window struct
typedef struct {
    WINDOW *win;
    int width, height;
    int startx, starty;
} Screen;


// Drone struct
typedef struct{
	char ch;
	double x,y;
    double vx, vy; 
} Drone;


//obstacles
typedef struct {
    int x, y;
} Obstacle;


//targets
typedef struct {
    int x, y;
} Target;


// Gamestate
typedef struct {
    //drone
    Drone drone;

    //physics
    double mass;
    double k;
    double dt;
    
    double rho;
    double eta;
    double zeta;
    double tangent_gain;

    double command_force; 
    double fx_cmd; 
    double fy_cmd; 
    double max_force;
    double fx_obst;
    double fy_obst;
    double fx_fence;
    double fy_fence;
    double fx_tot;
    double fy_tot;


    //window size
    int world_width;
    int world_height;

    //obstacles
    int num_obstacles;
    Obstacle obstacles[MAX_OBSTACLES];

    //target
    int num_targets;
    Target targets[MAX_TARGETS];

    //score
    int score;
} GameState;


// static parameters from file
typedef struct {
    //physics
    double mass;
    double k;
    double dt;
    double command_force;
    double max_force;
    double rho;
    double eta;
    double zeta;
    double tangent_gain;


    //window size
    int world_width;
    int world_height;

    //drone
    int drone_start_x;
    int drone_start_y;

    //target
    int num_targets;

    //obstacles
    int num_obstacles;
} Config;

//------------------------------------------------------------------------FUNCTIONS

void init_screen(Screen *s);
void refresh_screen(Screen *s);
void init_game(GameState *g, Config *cfg);
void render(Screen *s, GameState *g);

#endif