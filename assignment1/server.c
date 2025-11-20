#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/select.h>
#include <ncurses.h>

#include "map.c" 

typedef struct {
    char type;
    int a;
    int b;
} msgSelect;


// ----------------------------------------------------------- FUNCTION


void process_input(int fd){
    nodelay(stdscr, TRUE);

    while(1){
        int ch = getch();
        msgSelect msg = {'I', 0, 0};

        if (ch == 'w'){
            msg.a = -1;
            msg.b = -1;
        }
        if (ch == 'e'){
            msg.a = 0;
            msg.b = -1;
        }
        if (ch == 'r'){
            msg.a = +1;
            msg.b = -1;
        }
        if (ch == 's'){
            msg.a = -1;
            msg.b = 0;
        }
        /*if (ch == 'd'){
            msg.a = 0;
            msg.b = 0;
        }*/
        if (ch == 'f'){
            msg.a = +1;
            msg.b = 0;
        }
        if (ch == 'x'){
            msg.a = -1;
            msg.b = +1;
        }
        if (ch == 'c'){
            msg.a = 0;
            msg.b = +1;
        }
        if (ch == 'v'){
            msg.a = +1;
            msg.b = +1;
        }

        write(fd, &msg, sizeof(msg));
        usleep(20000);
    }
}

void process_drone(int fd){
    int x = 10;
    int y = 10;

    while(1){
        //Euler

        msgSelect msg = {'D', 0, 0};
        write(fd, &msg, sizeof(msg));
        usleep(20000);
    }
}

// ----------------------------------------------------------- MAIN

int main()
{
    // ncurses
    Screen screen;
    GameState gs;

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

    fd_set readfds;
    int maxfd;
    int ready;
    msgSelect msg;

    // input
    pid_t pid_input = fork();
    if(pid_input == 0){
        close(pipe_input[0]);
        input_process(pipe_input[1]);
        exit(0);
    }

    // drone
    pid_t pid_drone = fork();
    if(pipe_drone == 0){
        close(pipe_drone[0]);
        drone_process(pipe_drone[1]);
        exit(0);
    }

    close(pipe_input[1]);
    close(pipe_drone[1]);

    maxfd = max(pipe_input[0], pipe_drone[0])+1;
    
    while (1){
        FD_ZERO(&readfds);
        FD_SET(pipe_input[0], &readfds);
        FD_SET(pipe_drone[0], &readfds);

        select(maxfd, &readfds, NULL, NULL, NULL);

        if(FD_ISSET(pipe_input[0], &readfds)){
            read(pipe_input[0], &msg, sizeof(msg));
            if(msg.type == 'I'){
                gs.drone.x += msg.a;
                gs.drone.y += msg.b;
            }
        }

        if(FD_ISSET(pipe_drone[0], &readfds)){
            read(pipe_drone[0], &msg, sizeof(msg));
            if(msg.type == 'D'){
                gs.drone.x += msg.a;
                gs.drone.y += msg.b;
            }
        }
    }

    endwin();
    render(&screen, &gs);
}



