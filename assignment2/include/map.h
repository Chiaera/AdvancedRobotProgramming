/* this file contains the map process
    - all the structs for the project
    - function to initializate the ncurses window
*/

#ifndef MAP_H
#define MAP_H

#define MAX_OBSTACLES 20
#define MAX_TARGETS 10

#include <ncurses.h>

//------------------------------------------------------------------------STRUCTS

// Window struct
typedef struct {
    WINDOW *win;
    int width, height; //dimension
    int startx, starty; //start position of the drone
} Screen;


// Drone struct
typedef struct{
	char ch; //type
	double x,y; //coordinates of the drone
    double vx, vy; //velocities along the directions
} Drone;


//obstacles struct
typedef struct {
    int x, y; //coordinates of the obstacles
} Obstacle;


//targets stryct
typedef struct {
    int x, y; //coordinates of the targets
} Target;


// Gamestate struct
typedef struct {
    //drone
    Drone drone;

    //physics
    double mass;
    double k;
    double dt;
    
    //repulsive (and actattive) forces
    double rho;
    double eta;
    double zeta;
    double tangent_gain;

    //input forces
    double command_force; 
    double fx_cmd; 
    double fy_cmd; 
    double max_force;

    //type of forces
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
    double total_distance;
    int obstacles_hit;
    int targets_collected;
    int fence_collision;
    int was_on_fence;
    int was_on_obstacles;
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
    int target_reloc;   

    //obstacles
    int num_obstacles;
    int obstacle_reloc; 
} Config;

//------------------------------------------------------------------------FUNCTIONS

void init_screen(Screen *s);
void refresh_screen(Screen *s);
void init_game(GameState *g, Config *cfg);
void render(Screen *s, GameState *g);

#endif