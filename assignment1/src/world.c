/* this file contains the function for the world process
    - spawn of obstacles and check the position
    - spaw of targets and check the position
    - drone-target collision
*/

#include <stdlib.h>
#include <math.h>  

#include "world.h"

//spawn the obstacle in a valid position
void respawn_obstacle(GameState *g, int i) { 
    int ox, oy;
    int valid = 0;

    while (!valid) { //loop until the position is valid
        valid = 1;  

        //random position for the drone
        ox = rand() % g->world_width;
        oy = rand() % g->world_height;

        // no overlap between obstacles
        for (int j = 0; j < g->num_obstacles; j++) {
            if (j == i) continue;
            if (g->obstacles[j].x == ox && g->obstacles[j].y == oy) { //obstacle already exist in that coordinates
                valid = 0;
                break;
            }
        }

        // no overlap with the drone
        if (valid && ox == (int)round(g->drone.x) && oy == (int)round(g->drone.y)) {//there is the drone in that position
            valid = 0;
        }

        // no overlap with the target
        for (int j = 0; valid && j < g->num_targets; j++) {
            if (g->targets[j].x == ox && g->targets[j].y == oy) { //target already exist in that position
                valid = 0;
                break;
            }
        }
    }

    //if the position is free we can save it for the obstacle i-th
    g->obstacles[i].x = ox;
    g->obstacles[i].y = oy;
}


//spawn the target in a valid position
void respawn_target(GameState *g, int i) {
    int tx, ty;
    int valid = 0;
    while (!valid) { //loop until the position is valid
        valid = 1; 

        //random position for the drone
        tx = rand() % g->world_width;
        ty = rand() % g->world_height;

        // not overlap between targets
        for (int j = 0; j < g->num_targets; j++) {
            if (j == i) continue;
            if (g->targets[j].x == tx && g->targets[j].y == ty){ //target already exists in that position
                valid = 0;
                break;
            }
        }

        //no overlap with the drone
        if (valid && tx == (int)round(g->drone.x) && ty == (int)round(g->drone.y)){ //there is the drone in that position
            valid = 0;
        }

        //no overlap with obstacle
        for (int j = 0; valid && j < g->num_obstacles; j++) {
            if (g->obstacles[j].x == tx && g->obstacles[j].y == ty){ //obstacle already exists in that position
                valid = 0;
                break;
            }
        }
    }

    //save valid position
    g->targets[i].x = tx;
    g->targets[i].y = ty;
}


//managment the collision between drone and target
void drone_target_collide(GameState *g){
    //save the 'old' position
    int drone_ix = (int)round(g->drone.x);
    int drone_iy = (int)round(g->drone.y);

    for (int i = 0; i < g->num_targets; i++) {
        if (drone_ix == g->targets[i].x && drone_iy == g->targets[i].y) { //drone-target collision

            g->score += 1;

            for (int j = i; j < g->num_targets - 1; j++) { //new indices for the target vector
                g->targets[j] = g->targets[j + 1];
            }

            //new number of targets and indices for the vector
            g->num_targets--;
            i--;
        }
    }
}
