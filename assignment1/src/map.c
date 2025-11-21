#include "map.h"

#include <string.h>


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

void init_screen(Screen*s){
    s-> starty = 1;
    s-> startx = 1;
    s-> height = LINES-2;
    s-> width = COLS-2;
    s-> win = create_newwin(s->height, s->width, s->starty, s->startx);
}

void refresh_screen(Screen *s, GameState *g) {
    s->height = LINES - 2;
    s->width  = COLS  - 2;

    destroy_win(s->win);
    s->win = create_newwin(s->height, s->width, s->starty, s->startx);

    mvprintw(0, 0, "Press keys (w e r s d f x c v), F2 = quit");
    clrtoeol();   
    refresh();

    if (g->drone.x >= s->width-1) g->drone.x = s->width-2;
    if (g->drone.y >= s->height-1) g->drone.y = s->height-2;
}



void init_game(GameState *g, Screen *s){
    memset(g, 0, sizeof(GameState));
    g-> drone.ch = '+';
    g-> drone.x = s->width/2;
    g-> drone.y = s->height/2;

    g-> target_x = s-> width-6;
    g-> target_y = s-> height-4;

    g -> score = 0;
}

void render(Screen *s, GameState *g){
    werase(s->win);
    box(s->win, 0, 0);

    mvwprintw(s->win, 0, 2, "Score: %d", g->score);
    mvwaddch(s->win, g->target_y, g->target_x, '1');
    mvwaddch(s->win, g->drone.y, g->drone.x, g->drone.ch);

    wrefresh(s->win);
}