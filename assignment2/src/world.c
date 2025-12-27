/* this file contains the function for the world process
    - spawn of obstacles and check the position
    - spaw of targets and check the position
    - drone-target collision
*/

#include <stdlib.h>
#include <math.h>  

#include "world.h"
#include "logger.h"

//spawn the obstacle in a valid position
void respawn_obstacle(GameState *g, int i) { 
    int ox, oy;
    int valid = 0;

    while (!valid) { 
        valid = 1;  

        //random position within world bounds
        // Used for respawning to avoid overlaps
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
    while (!valid) { 
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
    double r_pick = 1.5; //pickup radius (in "cells")
    double r2 = r_pick * r_pick;

    for (int i = 0; i < g->num_targets; i++) {
        double dx = g->drone.x - (double)g->targets[i].x;
        double dy = g->drone.y - (double)g->targets[i].y;
        double d2 = dx*dx + dy*dy;

        if(d2 < r2){ // drone is close enough to "collect" the target
            g->targets_collected += 1;
        
            log_message("DRONE_PHYSICS", "Drone collects a target"); //write in the system.log

            for (int j = i; j < g->num_targets - 1; j++) { //shift: remove the collected target from array
                g->targets[j] = g->targets[j + 1];
            }

            //number of targets and indices remains for new the vector targets
            g->num_targets--;
            i--;
        }        
    }
}


//compute the total score
int calculate_final_score(GameState *g) {
    int target_point = 10; 
    int obstacle_penalty = 3; 
    int fence_penalty = 1; 
    int score = 0; 
    
    //colected target
    score += g->targets_collected * target_point; 
    
    //penality for obstacles and fences collision
    score -= g->obstacles_hit * obstacle_penalty; 
    score -= g->fence_collision * fence_penalty; 
    
    if (score < 0) score = 0; //minimum score is zero
    
    g->score = score; //save score
    return score;
}