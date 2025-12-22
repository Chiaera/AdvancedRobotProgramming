/* this file contains the process world which draw the world
    - function to position the obstacles
    - function to position the targets
    - collision between drone-target
*/

#ifndef WORLD_H
#define WORLD_H

#include "map.h"   


void respawn_obstacle(GameState *g, int i);
void respawn_target(GameState *g, int i);
void drone_target_collide(GameState *g);

#endif
