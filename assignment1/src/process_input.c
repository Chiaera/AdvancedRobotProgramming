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
        if (ch == ERR) {        
            usleep(20000);
            continue;            
        }

        msgInput msg = {'I', 0, 0};
        if(ch == 'w' || ch=='W') { msg.dx = -1; msg.dy = -1; }
        if(ch == 'e' || ch=='E') { msg.dx =  0; msg.dy = -1; }
        if(ch == 'r' || ch=='R') { msg.dx = +1; msg.dy = -1; }
        if(ch == 's' || ch=='S') { msg.dx = -1; msg.dy =  0; }
        if(ch == 'd' || ch=='D') { msg.dx =  0; msg.dy =  0; }
        if(ch == 'f' || ch=='F') { msg.dx = +1; msg.dy =  0; }
        if(ch == 'x' || ch=='X') { msg.dx = -1; msg.dy = +1; }
        if(ch == 'c' || ch=='C') { msg.dx =  0; msg.dy = +1; }
        if(ch == 'v' || ch=='V') { msg.dx = +1; msg.dy = +1; }
        
        if(ch == 'q' || ch == 'Q') {
            msgInput quit_msg = {'Q', 0, 0};
            write(fd, &quit_msg, sizeof(quit_msg));
            break; 
        }
        
        write(fd, &msg, sizeof(msg));
        usleep(20000);
    }
}