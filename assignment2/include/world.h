/* this file contains the process world which draw the world
    - function to position the obstacles
    - function to position the targets
    - function to send a tick for the targets to change targets position
    - function to send a tick for the obstacles to change targets position
    - collision between drone-target
*/

#ifndef WORLD_H
#define WORLD_H

#include "map.h"   

void respawn_obstacle(GameState *g, int i);
void respawn_target(GameState *g, int i);
void relocation_targets(int fd);
void relocation_obstacles(int fd);
void drone_target_collide(GameState *g);

#endif
