/* this file contains the process physics
    - compute the direction forces
    - compute the brake
    - compute the dynamics of the drone
*/

#ifndef WORLD_PHYSICS_H
#define WORLD_PHYSICS_H

#include "map.h"

void add_direction(GameState *g, int mx, int my);
void use_brake(GameState *g);
void add_drone_dynamics(GameState *g);

#endif