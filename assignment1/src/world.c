#include <stdlib.h>
#include "world.h"

void respawn_obstacle(GameState *g, int i) {
    int ox, oy;
    int valid = 0;
    while (!valid) {
        ox = rand() % g->world_width;
        oy = rand() % g->world_height;

        // no overlap
        for (int j = 0; j < g->num_obstacles; j++) {
            if (j == i) continue;
            if (g->obstacles[j].x == ox && g->obstacles[j].y == oy) {
                valid = 0;
                break;
            }
        }

        //obstacle on drone
        if (ox == g->drone.x && oy == g->drone.y){
            valid = 0;
            break;
        }

        // obstacle on targets
        for (int j=0; j<g->num_targets; j++) {
            if (g->targets[j].x == ox && g->targets[j].y == oy){
                valid = 0;
                break;
            }
        }
    }
    g->targets[i].x = ox;
    g->targets[i].y = oy;
}

void respawn_target(GameState *g, int i) {
    int tx, ty;
    int valid = 0;
    while (!valid) {
        tx = rand() % g->world_width;
        ty = rand() % g->world_height;

        // no overlap
        for (int j = 0; j < g->num_targets; j++) {
            if (j == i) continue;
            if (g->targets[j].x == tx && g->targets[j].y == ty){
                valid = 0;
                break;
            }
        }

        //drone-target collide
        if (tx == g->drone.x && ty == g->drone.y){
            valid = 0;
            break;
        }

        // target on obstacles
        for (int j=0; j<g->num_obstacles; j++) {
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
    for (int i = 0; i < g->num_targets; i++) {
        if (g->drone.x == g->targets[i].x && g->drone.y == g->targets[i].y){
            g->score += 1;
            respawn_target(g, i);
        }
    }
}
