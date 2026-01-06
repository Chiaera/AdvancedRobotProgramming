/* this file contains the function for the map process
    - create and managment the ncurses window
*/

#include "map.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

//create the window
WINDOW *create_newwin(int height, int width, int starty, int startx) {
    WINDOW *local_win = newwin(height, width, starty, startx);
    box(local_win, 0, 0);
    wrefresh(local_win);
    return local_win;
}

//function to destroy the window
void destroy_win(WINDOW *local_win){
    if(!local_win) return;
    werase(local_win);
    wrefresh(local_win);
    delwin(local_win);
}

// initializate the window dimension
void init_screen(Screen*s){
    s-> starty = 14;
    s-> startx = 1;
    s-> height = LINES-s -> starty-1;
    s-> width = COLS-2;
    s-> win = create_newwin(s->height, s->width, s->starty, s->startx);
}

// resize - re-draw the window
void refresh_screen(Screen *s) { 
    s->height = LINES-s -> starty-1; //re-define the border
    s->width  = COLS-2;

    destroy_win(s->win); //destroy the window
    s->win = create_newwin(s->height, s->width, s->starty, s->startx); //draw the window with the new dimensions

    clrtoeol();   
    refresh();
}


// setting the game to the zero state
void init_game(GameState *g, Config *cfg){

    memset(g, 0, sizeof(GameState));
    
    // physics parameters
    g->mass = cfg->mass;
    g->k = cfg->k;
    g->dt = cfg->dt;
    g->command_force = cfg->command_force;
    g->fx_cmd = 0;
    g->fy_cmd = 0;
    g->max_force = cfg->max_force;
    g->rho = cfg->rho;
    g->eta = cfg->eta;
    g->zeta = cfg->zeta;
    g->tangent_gain = cfg->tangent_gain;

    //size
    g->world_width  = cfg->world_width;
    g->world_height = cfg->world_height;

    // drone 
    g->drone.ch = '+';
    if (cfg->drone_start_x == 0 && cfg->drone_start_y == 0) {
        g->drone.x = g->world_width  / 2.0;
        g->drone.y = g->world_height / 2.0;
    } else {
        g->drone.x = (double)cfg->drone_start_x;
        g->drone.y = (double)cfg->drone_start_y;
    }
    g->drone.vx = 0;
    g->drone.vy = 0;

    // target
    g->num_targets = 0;
    g->total_targets = cfg->num_targets;
    g->current_target_index = 0; 
    for (int i = 0; i < MAX_TARGETS; i++) {
        g->targets[i].x = 0;
        g->targets[i].y = 0;
    }

    //obstacles
    g->num_obstacles = 0;
    for (int i = 0; i < MAX_OBSTACLES; i++) {
        g->obstacles[i].x = 0;
        g->obstacles[i].y = 0;
    }

    //score 
    g->score = 0;
    g->obstacles_hit = 0;
    g->targets_collected = 0;
    g->total_target_collected = 0;
    g->fence_collision = 0;
    g->was_on_fence = 0;
    g->was_on_obstacles = 0;
    g->obstacles_hit_tot = 0;
    g->fence_collision_tot = 0;
}

// draw the window
void render(Screen *s, GameState *g){
    werase(s->win); //dedine the border of the map
    box(s->win, 0, 0);

    mvwprintw(s->win, 0, 2, "Score: %d | Targets: %d/%d", g->score, g->total_target_collected, g->total_targets); //print the score of the game

    double sx = 1.0, sy = 1.0; //resize the map
    if (g->world_width  > 1)
        sx = (double)(s->width  - 2) / (g->world_width  - 1);
    if (g->world_height > 1)
        sy = (double)(s->height - 2) / (g->world_height - 1);

    // show only the current target
    if (g->num_targets > 0 && g->current_target_index < g->num_targets) {
        int i = g->current_target_index;

        int tx = 1 + (int)round(g->targets[i].x * sx);
        int ty = 1 + (int)round(g->targets[i].y * sy);
        if (tx < 1) tx = 1;
        if (tx > s->width-2) tx = s->width-2;
        if (ty < 1) ty = 1;
        if (ty > s->height-2) ty = s->height-2;

        //design of the target
        wattron(s->win, COLOR_PAIR(1)); 
        mvwaddch(s->win, ty, tx, 'T');
        wattroff(s->win, COLOR_PAIR(1));
    }

    // obstacles need to be inside the map 
    for (int i = 0; i < g->num_obstacles; i++) {
        int ox = 1 + (int)round(g->obstacles[i].x * sx);
        int oy = 1 + (int)round(g->obstacles[i].y * sy);
        if (ox < 1) ox = 1;
        if (ox > s->width-2) ox = s->width-2;
        if (oy < 1) oy = 1;
        if (oy > s->height-2) oy = s->height-2;

        //design of the obstacle
        wattron(s->win, COLOR_PAIR(2));
        mvwaddch(s->win, oy, ox, 'O');
        wattroff(s->win, COLOR_PAIR(2));
        
    }

    // drone need to be inside the map 
    int dx = 1 + (int)round(g->drone.x * sx);
    int dy = 1 + (int)round(g->drone.y * sy);
    if (dx < 1) dx = 1;
    if (dx > s->width-2) dx = s->width-2;
    if (dy < 1) dy = 1;
    if (dy > s->height-2) dy = s->height-2;

    //design of the drone
    wattron(s->win, COLOR_PAIR(3) | A_BOLD);
    mvwaddch(s->win, dy, dx, g->drone.ch);
    wattroff(s->win, COLOR_PAIR(3) | A_BOLD);

    wrefresh(s->win); 
}