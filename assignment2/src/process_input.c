/* this file contains the function for the process input
    - read input from the keybord
    - pass the given input to the server

    - use for the watchdog
        - maps the posix shared memory (heartbeat table)
        - periodically updates its slow with monotonic timestamp
*/

#define _POSIX_C_SOURCE 200809L

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ncurses.h>
#include <ctype.h>
#include <fcntl.h>      
#include <sys/mman.h>  
#include <sys/stat.h>  
#include <time.h>

#include "process_input.h"
#include "heartbeat.h"


//-----------------------------------------------------------------------STRUCT
typedef struct { //for the input force in the x and y coordinates
    char type;
    int dx, dy;
} msgInput;

typedef struct { //for mapping the input, create the 'table' of directions
    int row, col;
    char label;
} KeyBox;

KeyBox keymap[] = { //input map
    {0, 1, 'w'}, {0, 2, 'e'}, {0, 3, 'r'},
    {1, 1, 's'}, {1, 2, 'd'}, {1, 3, 'f'},
    {2, 1, 'x'}, {2, 2, 'c'}, {2, 3, 'v'}
};

//-----------------------------------------------------------------------FUNCTIONS

//function to draw the tables
void draw_key_box(WINDOW *win, int y, int x, char label, int highlighted)
{
    //define the dimension of the box
    int h = 3;
    int w = 5;
    int top  = y - 1;
    int left = x - 2;

    //pressed key
    if (highlighted) {
        wattron(win, COLOR_PAIR(2) | A_BOLD); //color to highlight the word
    }

    //border to design the table of input
    mvwaddch(win, top,        left,        ACS_ULCORNER);
    mvwhline(win, top,        left + 1,    ACS_HLINE, w - 2);
    mvwaddch(win, top,        left + w -1, ACS_URCORNER);

    mvwvline(win, top + 1,    left,        ACS_VLINE, h - 2);
    mvwvline(win, top + 1,    left + w -1, ACS_VLINE, h - 2);

    mvwaddch(win, top + h -1, left,        ACS_LLCORNER);
    mvwhline(win, top + h -1, left + 1,    ACS_HLINE, w - 2);
    mvwaddch(win, top + h -1, left + w -1, ACS_LRCORNER);

    mvwaddch(win, y, x, label);

    if (highlighted) { //managment the press key
        wattroff(win, COLOR_PAIR(2) | A_BOLD);
    }
}

//funcition to draw the keys in the table
void draw_keys(WINDOW* win, char highlight)
{
    wclear(win);
    box(win, 0, 0);

    for (int i = 0; i < 9; i++) { //draw the table
        int r = keymap[i].row;
        int c = keymap[i].col;
        char label = keymap[i].label;

        //centered key in the box
        int y = 3 + r * 4;  
        int x = 8 + (c - 1) * 6;

        int pressed = (label == highlight); //selected key

        draw_key_box(win, y, x, label, pressed); //draw the border
    }

    wrefresh(win);
}

//function to define the key input
void set_input(int fd, WINDOW* win, HeartbeatTable *hb, int slot){
    nodelay(stdscr, TRUE);

    //used for the 'nanosleep' function
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 20 * 1000 * 1000; // 20 ms

    while(1){ 
        hb->entries[slot].last_seen_ms = now_ms();   //tells to watchdog it is active

        int ch = getch();
        if (ch == ERR) {        
            nanosleep(&ts, NULL);
            continue;            
        }

        char key = tolower(ch); //keypress
        draw_keys(win, key); //draw the table

        //keypress selected
        msgInput msg = {'I', 0, 0}; //messagge 'input'
        if(ch == 'w' || ch=='W') { msg.dx = -1; msg.dy = -1; } //north-west
        if(ch == 'e' || ch=='E') { msg.dx =  0; msg.dy = -1; } //north
        if(ch == 'r' || ch=='R') { msg.dx = +1; msg.dy = -1; } //north-east
        if(ch == 's' || ch=='S') { msg.dx = -1; msg.dy =  0; } //west
        if(ch == 'd' || ch=='D') { msg.type = 'B'; } //message 'brake'
        if(ch == 'f' || ch=='F') { msg.dx = +1; msg.dy =  0; } //east
        if(ch == 'x' || ch=='X') { msg.dx = -1; msg.dy = +1; } //south-west
        if(ch == 'c' || ch=='C') { msg.dx =  0; msg.dy = +1; } //south
        if(ch == 'v' || ch=='V') { msg.dx = +1; msg.dy = +1; } //south-east
        
        //message 'quit'
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
        nanosleep(&ts, NULL);
    }
}


int main(int argc, char *argv[])
{
    if (argc < 4) {
        /*expected args:
            1. write_fd
            2. shm_name ('/heartbeat')
            3. slot index ('1' for HB_SLOT_INPUT)
        */
        fprintf(stderr, "Usage: %s <write_fd> <shm_name> <slot>\n", argv[0]);
        return 1;
    }
    //read the argv
    int fd = atoi(argv[1]);
    const char *shm_name = argv[2];
    int slot = atoi(argv[3]);

    //open existing shared memory created by blackboard
    int hb_fd = shm_open(shm_name, O_RDWR, 0666);
    if (hb_fd < 0) { 
        perror("process_input shm_open"); 
        return 1; 
    }

    //map heartbeat table
    HeartbeatTable *hb = mmap(NULL, sizeof(HeartbeatTable), PROT_READ | PROT_WRITE, MAP_SHARED, hb_fd, 0);
    if (hb == MAP_FAILED) { 
        perror("process_input mmap"); 
        close(hb_fd); 
        return 1; 
    }

    //save PID
    hb->entries[slot].pid = getpid();


    //function to the ncurses window
    initscr();
    noecho();
    curs_set(0);

    //color design
    start_color(); 
    init_pair(1, COLOR_WHITE, COLOR_BLACK);  
    init_pair(2, COLOR_BLACK, COLOR_GREEN);  //pressed

    WINDOW* win_keys = newwin(15, 30, 1, 1); //initializate the ncurses window

    mvprintw(0, 0, "Input window (w e r / s d f / x c v) or 'q' to quit"); //print input legend
    refresh();

    draw_keys(win_keys, 0); //call the draw keys function

    set_input(fd, win_keys, hb, slot); //define the input in the message

    endwin();
    munmap(hb, sizeof(*hb));
    close(hb_fd);
    close(fd);

    return 0;
}