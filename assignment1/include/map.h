#ifndef MAP_H
#define MAP_H

#include <ncurses.h>

//------------------------------------------------------------------------STRUCTS

// Window
typedef struct {
    WINDOW *win;
    int width, height;
    int startx, starty;
} Screen;

// Drone 
typedef struct{
	char ch;
	int x,y;
	double M, F, K;
    double vx, vy;
} Drone;

// Gamestate
typedef struct {
    Drone drone;
    int obstacle_x, obstacle_y;
    int target_x, target_y;
    int score;
} GameState;


//------------------------------------------------------------------------FUNCTIONS

void init_screen(Screen *s);
void refresh_screen(Screen *s, GameState *g);
void init_game(GameState *g, Screen *s);
void render(Screen *s, GameState *g);

#endif