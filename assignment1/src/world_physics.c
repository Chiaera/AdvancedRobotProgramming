#include <math.h>

#include "world_physics.h"   
#include "map.h" 

// INPUT - direction
void add_direction(GameState *gs, int mx, int my){
    gs->fx_cmd += gs->command_force * mx;  
    gs->fy_cmd += gs->command_force * my;

    if (gs->fx_cmd > gs->max_force) gs->fx_cmd = gs->max_force;
    if (gs->fx_cmd < -gs->max_force) gs->fx_cmd = -gs->max_force;
    if (gs->fy_cmd > gs->max_force) gs->fy_cmd = gs->max_force;
    if (gs->fy_cmd < -gs->max_force) gs->fy_cmd = -gs->max_force;
}

void use_brake(GameState *gs){
    const double brake_factor = 0.5;

    gs->fx_cmd *= brake_factor;
    gs->fy_cmd *= brake_factor;
    gs->drone.vx *= brake_factor;
    gs->drone.vy *= brake_factor;
}



// DRONE - physics
void  add_drone_dynamics(GameState *gs){
    //resultant forces
    double fx = gs->fx_cmd;
    double fy = gs->fy_cmd;

    // a = (F - k*v)/M
    double ax = (fx - gs->k *gs ->drone.vx) / gs->mass;
    double ay = (fy - gs->k * gs->drone.vy) / gs->mass;

    // v = a*dt
    gs->drone.vx += ax * gs->dt;
    gs->drone.vy += ay * gs->dt;

    // Euler - position
    double new_x = gs-> drone.x + gs->drone.vx * gs->dt;
    double new_y = gs->drone.y + gs->drone.vy * gs->dt;
    gs->drone.x = (int)round(new_x);
    gs->drone.y = (int)round(new_y);
}