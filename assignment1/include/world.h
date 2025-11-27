#ifndef WORLD_H
#define WORLD_H

#include "map.h"   


void respawn_obstacle(GameState *g, int i);
void respawn_target(GameState *g, int i);
void drone_target_collide(GameState *g);

#endif
