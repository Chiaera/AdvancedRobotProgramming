#include "map.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>


WINDOW *create_newwin(int height, int width, int starty, int startx) {
    WINDOW *local_win = newwin(height, width, starty, startx);
    box(local_win, 0, 0);
    wrefresh(local_win);
    return local_win;
}

void destroy_win(WINDOW *local_win){
    if(!local_win) return;
    werase(local_win);
    wrefresh(local_win);
    delwin(local_win);
}

// initializate the window dimension
void init_screen(Screen*s){
    s-> starty = 1;
    s-> startx = 1;
    s-> height = LINES-2;
    s-> width = COLS-2;
    s-> win = create_newwin(s->height, s->width, s->starty, s->startx);
}

// resize and border
void refresh_screen(Screen *s) {
    s->height = LINES-2;
    s->width  = COLS-2;

    destroy_win(s->win);
    s->win = create_newwin(s->height, s->width, s->starty, s->startx);

    mvprintw(0, 0, "Press keys (w e r s d f x c v), q = quit");
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

    //size
    g->world_width  = cfg->world_width;
    g->world_height = cfg->world_height;

    // drone 
    g->drone.ch = '+';
    if (cfg->drone_start_x == 0 && cfg->drone_start_y == 0) {
        g->drone.x = g->world_width/2;
        g->drone.y = g->world_height/2;
    } else {
        g->drone.x = cfg->drone_start_x;
        g->drone.y = cfg->drone_start_y;
    }
    g->drone.vx = 0;
    g->drone.vy = 0;

    // target
    if (cfg->target_x == 0 && cfg->target_y == 0) {
        int tx, ty;
        int valid_position_target = 0;
        while (!valid_position_target) {
            tx = rand() % g->world_width;
            ty = rand() % g->world_height;
            if (tx == g->drone.x && ty == g->drone.y) continue;
            valid_position_target = 1;
        }
        g->target_x = tx;
        g->target_y = ty;
    } else {
        g->target_x = cfg->target_x;
        g->target_y = cfg->target_y;
    }

    //obstacles
    g->num_obstacles = cfg->num_obstacles;
    if (g->num_obstacles > MAX_OBSTACLES) g->num_obstacles = MAX_OBSTACLES;
    for (int i = 0; i < g->num_obstacles; i++) {
        int obstacle_x, obstacle_y;
        int valid_position_obstacle = 0;
        while (!valid_position_obstacle) {
            obstacle_x = rand() % g->world_width;
            obstacle_y = rand() % g->world_height;
            if (obstacle_x==g->drone.x && obstacle_y==g->drone.y) continue;
            if (obstacle_x==g->target_x && obstacle_y==g->target_y) continue;
            int another_obstacle = 0;
            for (int j = 0; j < i; j++) {
                if (g->obstacles[j].x==obstacle_x && g->obstacles[j].y==obstacle_y) {
                    another_obstacle = 1;
                    break;
                }
            }
            if (another_obstacle) continue;
            valid_position_obstacle = 1;
        }
        g->obstacles[i].x = obstacle_x;
        g->obstacles[i].y = obstacle_y;
    }

    //score 
    g->score = 0;
}

//clear window
void render(Screen *s, GameState *g){
    werase(s->win);
    box(s->win, 0, 0);

    mvwprintw(s->win, 0, 2, "Score: %d", g->score);

    double sx = 1.0, sy = 1.0;
    if (g->world_width  > 1)
        sx = (double)(s->width  - 2) / (g->world_width  - 1);
    if (g->world_height > 1)
        sy = (double)(s->height - 2) / (g->world_height - 1);

    // target
    int tx = 1 + (int)round(g->target_x * sx);
    int ty = 1 + (int)round(g->target_y * sy);
    if (tx < 1) tx = 1;
    if (tx > s->width-2) tx = s->width-2;
    if (ty < 1) ty = 1;
    if (ty > s->height-2) ty = s->height-2;
    mvwaddch(s->win, ty, tx, 'T');

    // ostacoli
    for (int i = 0; i < g->num_obstacles; i++) {
        int ox = 1 + (int)round(g->obstacles[i].x * sx);
        int oy = 1 + (int)round(g->obstacles[i].y * sy);
        if (ox < 1) ox = 1;
        if (ox > s->width-2) ox = s->width-2;
        if (oy < 1) oy = 1;
        if (oy > s->height-2) oy = s->height-2;
        mvwaddch(s->win, oy, ox, 'O');
    }

    // drone
    int dx = 1 + (int)round(g->drone.x * sx);
    int dy = 1 + (int)round(g->drone.y * sy);
    if (dx < 1) dx = 1;
    if (dx > s->width-2) dx = s->width-2;
    if (dy < 1) dy = 1;
    if (dy > s->height-2) dy = s->height-2;
    mvwaddch(s->win, dy, dx, g->drone.ch);

    wrefresh(s->win);
}