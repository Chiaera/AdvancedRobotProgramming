#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>  
#include <sys/select.h>
#include <ncurses.h>

#include "map.h"
#include "process_input.h"
#include "process_drone.h"


typedef struct {
    char type;
    int dx, dy;
} msgInput;

typedef struct  {
    char type;
    int x, y;
} msgDrone;


// ----------------------------------------------------------- MAIN

int main()
{
    // ncurses
    Screen screen;
    GameState gs;
    int old_lines = LINES;
    int old_cols  = COLS;


    initscr();
    noecho();
    curs_set(0);
    init_screen(&screen);
    init_game(&gs, &screen);


    // pipe
    int pipe_input[2];
    int pipe_drone[2];
    pipe(pipe_input);
    pipe(pipe_drone);

    // input
    pid_t pid_input = fork();
    if(pid_input == 0){
        close(pipe_input[0]);
        set_input(pipe_input[1]);
        exit(0);
    }

    // drone
    pid_t pid_drone = fork();
    if(pid_drone == 0){
        close(pipe_drone[0]);
        move_drone(pipe_drone[1]);
        exit(0);
    }

    close(pipe_input[1]);
    close(pipe_drone[1]);

    fd_set set;
    int maxfd = (pipe_input[0] > pipe_drone[0] ? pipe_input[0] : pipe_drone[0]) + 1;
    
    while (1){
        FD_ZERO(&set);
        FD_SET(pipe_input[0], &set);
        FD_SET(pipe_drone[0], &set);

        select(maxfd, &set, NULL, NULL, NULL);

        if (FD_ISSET(pipe_input[0], &set)) {
            msgInput m;
            read(pipe_input[0], &m, sizeof(m));

            if (m.type == 'Q') break;
            if (m.type == 'I') {
                gs.drone.x += m.dx;
                gs.drone.y += m.dy;
            }
        }

        if(FD_ISSET(pipe_drone[0], &set)){
            msgDrone m;
            read(pipe_drone[0], &m, sizeof(m));
            gs.drone.x += m.x;
            gs.drone.y += m.y;
        }

        // window border
        if (gs.drone.x < 1) gs.drone.x = 1;
        if (gs.drone.y < 1) gs.drone.y = 1;
        if (gs.drone.x > screen.width - 2)  gs.drone.x = screen.width - 2;
        if (gs.drone.y > screen.height - 2) gs.drone.y = screen.height - 2;

        //resize
        if (LINES != old_lines || COLS != old_cols) {
        old_lines = LINES;
        old_cols  = COLS;
        refresh_screen(&screen, &gs);
    }

        render(&screen, &gs);
    }

    endwin();
    return 0;
}



