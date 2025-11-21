#include <unistd.h>
#include <ncurses.h>

#include "process_input.h"

typedef struct {
    char type;
    int dx, dy;
} msgInput;

void set_input(int fd){
    nodelay(stdscr, TRUE);

    while(1){
        int ch = getch();
        msgInput msg = {'I', 0, 0};

        if(ch == 'w') { msg.dx = -1; msg.dy = -1; }
        if(ch == 'e') { msg.dx =  0; msg.dy = -1; }
        if(ch == 'r') { msg.dx = +1; msg.dy = -1; }
        if(ch == 's') { msg.dx = -1; msg.dy =  0; }
        if(ch == 'd') { msg.dx =  0; msg.dy =  0; }
        if(ch == 'f') { msg.dx = +1; msg.dy =  0; }
        if(ch == 'x') { msg.dx = -1; msg.dy = +1; }
        if(ch == 'c') { msg.dx =  0; msg.dy = +1; }
        if(ch == 'v') { msg.dx = +1; msg.dy = +1; }
        if (ch == 'q') {
            msgInput msg = {'Q', 0, 0};
            write(fd, &msg, sizeof(msg));
            break; 
        }
        
        write(fd, &msg, sizeof(msg));
        usleep(20000);
    }
}