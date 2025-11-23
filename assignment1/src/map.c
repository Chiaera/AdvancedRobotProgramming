#include "map.h"

#include <string.h>
#include <stdlib.h>


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
void refresh_screen(Screen *s, GameState *g) {
    s->height = LINES-2;
    s->width  = COLS-2;

    destroy_win(s->win);
    s->win = create_newwin(s->height, s->width, s->starty, s->startx);

    mvprintw(0, 0, "Press keys (w e r s d f x c v), q = quit");
    clrtoeol();   
    refresh();

    if (g->drone.x >= s->width-1) g->drone.x = s->width-2;
    if (g->drone.y >= s->height-1) g->drone.y = s->height-2;
}


// setting the game to the zero state
void init_game(GameState *g, Screen *s, Config *cfg){

    memset(g, 0, sizeof(GameState));
    
    // physics parameters
    g->mass = cfg->mass;
    g->k = cfg->k;
    g->dt = cfg->dt;
    g->command_force = cfg->command_force;
    g->fx_cmd = 0.0;
    g->fy_cmd = 0.0;

    // drone 
    g->drone.ch = '+';
    if (cfg->drone_start_x == 0 && cfg->drone_start_y == 0) {
        g->drone.x = s->width / 2;
        g->drone.y = s->height / 2;
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
            tx = 1 + rand() % (s->width-2);
            ty = 1 + rand() % (s->height-2);
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
            obstacle_x = 1 + rand() % (s->width-2);
            obstacle_y = 1 + rand() % (s->height-2);
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

// clear window and config the border as obstacles
void render(Screen *s, GameState *g){
    werase(s->win);
    box(s->win, 0, 0);

    mvwprintw(s->win, 0, 2, "Score: %d", g->score);

    //target
    mvwaddch(s->win, g->target_y, g->target_x, 'T');

    //obstacles
    for (int i = 0; i < g->num_obstacles; i++) {
    int obstacle_x = g->obstacles[i].x;
    int obstacle_y = g->obstacles[i].y;

    if (obstacle_x>0 && obstacle_x<s->width-1 && obstacle_y>0 && obstacle_y<s->height-1) {
        mvwaddch(s->win, obstacle_y, obstacle_x, 'O');  
    }
}
    mvwaddch(s->win, g->drone.y, g->drone.x, g->drone.ch);

    wrefresh(s->win);
}