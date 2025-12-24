/* this file contains the process drone which moves the drone
    - sends tick message to the blackboard to trigger drone updates
    - updates the heartbeat slot to signal activity to the watchdog
*/

#ifndef PROCESS_DRONE_H
#define PROCESS_DRONE_H


#include "heartbeat.h"

void move_drone(int fd, HeartbeatTable *hb, int slot);
/* arguments
    - fd: write-end of the pipe toward the blackboard
    - hb: pointer to shared heartbeat table
    - slot:index in the heartbeat table assigned to this process
*/

#endif
