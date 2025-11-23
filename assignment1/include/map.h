#ifndef MAP_H
#define MAP_H

#define MAX_OBSTACLES 20

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
	int x,y; //(step)
    double vx, vy; //(step/s)
} Drone;


//obstacles
typedef struct {
    int x, y;
} Obstacle;


// Gamestate
typedef struct {
    //drone
    Drone drone;

    //physics
    double mass;
    double k;
    double dt;
    double command_force; 
    double fx_cmd; 
    double fy_cmd; 

    //obstacles
    int num_obstacles;
    Obstacle obstacles[MAX_OBSTACLES];

    //target
    int target_x, target_y;

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

    //drone
    int drone_start_x;
    int drone_start_y;

    //target
    int target_x;
    int target_y;

    //obstacles
    int num_obstacles;
    int obstacle_x[MAX_OBSTACLES];
    int obstacle_y[MAX_OBSTACLES];
} Config;

//------------------------------------------------------------------------FUNCTIONS

void init_screen(Screen *s);
void refresh_screen(Screen *s, GameState *g);
void init_game(GameState *g, Screen *s, Config *cfg);
void render(Screen *s, GameState *g);

#endif