#ifndef PROCESS_INPUT_H
#define PROCESS_INPUT_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ncurses.h>
#include <ctype.h>

#include "process_input.h"

typedef struct {
    char type;
    int dx, dy;
} msgInput;

typedef struct {
    int row, col;
    char label;
} KeyBox;

KeyBox keymap[] = {
    {0, 1, 'w'}, {0, 2, 'e'}, {0, 3, 'r'},
    {1, 1, 's'}, {1, 2, 'd'}, {1, 3, 'f'},
    {2, 1, 'x'}, {2, 2, 'c'}, {2, 3, 'v'}
};

//border of keypress
void draw_key_box(WINDOW *win, int y, int x, char label, int highlighted)
{
    int h = 3;
    int w = 5;
    int top  = y - 1;
    int left = x - 2;

    //pressed key
    if (highlighted) {
        wattron(win, COLOR_PAIR(2) | A_BOLD);
    }

    //border
    mvwaddch(win, top,        left,        ACS_ULCORNER);
    mvwhline(win, top,        left + 1,    ACS_HLINE, w - 2);
    mvwaddch(win, top,        left + w -1, ACS_URCORNER);

    mvwvline(win, top + 1,    left,        ACS_VLINE, h - 2);
    mvwvline(win, top + 1,    left + w -1, ACS_VLINE, h - 2);

    mvwaddch(win, top + h -1, left,        ACS_LLCORNER);
    mvwhline(win, top + h -1, left + 1,    ACS_HLINE, w - 2);
    mvwaddch(win, top + h -1, left + w -1, ACS_LRCORNER);

    mvwaddch(win, y, x, label);

    if (highlighted) {
        wattroff(win, COLOR_PAIR(2) | A_BOLD);
    }
}

//keypress 
void draw_keys(WINDOW* win, char highlight)
{
    wclear(win);
    box(win, 0, 0);

    for (int i = 0; i < 9; i++) {
        int r = keymap[i].row;
        int c = keymap[i].col;
        char label = keymap[i].label;

        //centered key in the box
         int y = 3 + r * 4;  
        int x = 8 + (c - 1) * 6;

        int pressed = (label == highlight);

        draw_key_box(win, y, x, label, pressed);
    }

    wrefresh(win);
}

void set_input(int fd, WINDOW* win){
    nodelay(stdscr, TRUE);

    while(1){
        int ch = getch();
        if (ch == ERR) {        
            usleep(20000);
            continue;            
        }

        //keypress
        char key = tolower(ch);
        draw_keys(win, key);

        //keypress selected
        msgInput msg = {'I', 0, 0};
        if(ch == 'w' || ch=='W') { msg.dx = -1; msg.dy = -1; }
        if(ch == 'e' || ch=='E') { msg.dx =  0; msg.dy = -1; }
        if(ch == 'r' || ch=='R') { msg.dx = +1; msg.dy = -1; }
        if(ch == 's' || ch=='S') { msg.dx = -1; msg.dy =  0; }
        if(ch == 'd' || ch=='D') { msg.type = 'B'; } //brake
        if(ch == 'f' || ch=='F') { msg.dx = +1; msg.dy =  0; }
        if(ch == 'x' || ch=='X') { msg.dx = -1; msg.dy = +1; }
        if(ch == 'c' || ch=='C') { msg.dx =  0; msg.dy = +1; }
        if(ch == 'v' || ch=='V') { msg.dx = +1; msg.dy = +1; }
        
        //quit
        if(ch == 'q' || ch == 'Q') {
            msgInput quit_msg = {'Q', 0, 0};
            write(fd, &quit_msg, sizeof(quit_msg));
            break; 
        }

        /*//read the parameters
        if (ch == 'p' || ch == 'P') {
        msgInput msg = {'P', 0, 0};
        write(fd, &msg, sizeof(msg));
        }*/
        
        write(fd, &msg, sizeof(msg));
        usleep(20000);
    }
}


int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <write_fd>\n", argv[0]);
        return 1;
    }

    int fd = atoi(argv[1]);

    initscr();
    noecho();
    curs_set(0);

    start_color();
    init_pair(1, COLOR_WHITE, COLOR_BLACK);  
    init_pair(2, COLOR_BLACK, COLOR_GREEN);  //pressed

    WINDOW* win_keys = newwin(15, 30, 1, 1);

    mvprintw(0, 0, "Input window (w e r / s d f / x c v) or 'q' to quit");
    refresh();

    draw_keys(win_keys, 0);

    set_input(fd, win_keys);

    endwin();
    close(fd);
    return 0;
}


#endif