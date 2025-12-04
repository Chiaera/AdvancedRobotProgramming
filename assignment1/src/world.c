#include <stdlib.h>
#include <math.h>  

#include "world.h"


void respawn_obstacle(GameState *g, int i) {
    int ox, oy;
    int valid = 0;

    while (!valid) {
        valid = 1;  

        ox = rand() % g->world_width;
        oy = rand() % g->world_height;

        // no overlap between obstacles
        for (int j = 0; j < g->num_obstacles; j++) {
            if (j == i) continue;
            if (g->obstacles[j].x == ox && g->obstacles[j].y == oy) {
                valid = 0;
                break;
            }
        }

        // not on the drone
        if (valid && ox == (int)round(g->drone.x) && oy == (int)round(g->drone.y)) {
            valid = 0;
        }

        // not on the target
        for (int j = 0; valid && j < g->num_targets; j++) {
            if (g->targets[j].x == ox && g->targets[j].y == oy) {
                valid = 0;
                break;
            }
        }
    }

    g->obstacles[i].x = ox;
    g->obstacles[i].y = oy;
}


void respawn_target(GameState *g, int i) {
    int tx, ty;
    int valid = 0;
    while (!valid) {
        valid = 1; 

        tx = rand() % g->world_width;
        ty = rand() % g->world_height;

        // not overlap between targets
        for (int j = 0; j < g->num_targets; j++) {
            if (j == i) continue;
            if (g->targets[j].x == tx && g->targets[j].y == ty){
                valid = 0;
                break;
            }
        }

        // not on drone
        if (valid && tx == (int)round(g->drone.x) && ty == (int)round(g->drone.y)){
            valid = 0;
        }

        // not on obstacles
        for (int j = 0; valid && j < g->num_obstacles; j++) {
            if (g->obstacles[j].x == tx && g->obstacles[j].y == ty){
                valid = 0;
                break;
            }
        }
    }
    g->targets[i].x = tx;
    g->targets[i].y = ty;
}


void drone_target_collide(GameState *g){
    int drone_ix = (int)round(g->drone.x);
    int drone_iy = (int)round(g->drone.y);

    for (int i = 0; i < g->num_targets; i++) {
        if (drone_ix == g->targets[i].x &&
            drone_iy == g->targets[i].y) {

            //g->score += 1;

            for (int j = i; j < g->num_targets - 1; j++) {
                g->targets[j] = g->targets[j + 1];
            }
            g->num_targets--;
            i--;
        }
    }
}
