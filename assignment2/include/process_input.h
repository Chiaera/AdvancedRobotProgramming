/* this file contains the process input
    - reads keyboard input and sends commands to the blackboard
    - updates the heartbeat slot to signal activity to the watchdog
*/

#ifndef PROCESS_INPUT_H
#define PROCESS_INPUT_H

#include <ncurses.h>
#include "heartbeat.h"

void set_input(int fd, WINDOW *win, HeartbeatTable *hb, int slot);
/* arguments
    - fd: write-end of the pipe toward the blackboard
    - win: ncurses window used to display input keys
    - hb: pointer to shared heartbeat table
    - slot: index in the heartbeat table assigned to this process
*/

#endif
