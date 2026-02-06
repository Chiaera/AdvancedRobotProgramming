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

    //check if there are targets
    if (g->num_targets <= 0 || g->current_target_index >= g->total_targets) {
        return;   
    }

    int i = g->current_target_index; // only check the current target
    double dx = g->drone.x - (double)g->targets[i].x;
    double dy = g->drone.y - (double)g->targets[i].y;
    double d2 = dx*dx + dy*dy;

    if(d2 < r2){ // drone is close enough to "collect" the target
        g->targets_collected += 1;
        g->total_target_collected += 1;

        log_message("DRONE_PHYSICS", "Drone collects %d° target", g->current_target_index+1); //write in the system.log

        /*for (int j = i; j < g->num_targets - 1; j++) { //shift: remove the collected target from array
            g->targets[j] = g->targets[j + 1];
        }*/

        //prepare the next target
        if (g->current_target_index < g->total_targets) { 
            respawn_target(g, g->current_target_index); 
        }

        g->current_target_index++; //next target to collect
    }        
}


//compute the total score
int calculate_final_score(GameState *g) {
    int target_point = 10; 
    int obstacle_penalty = 5; 
    int fence_penalty = 3; 

    //actual score as 'base' of the computation
    int score = g->score; 
    
    //add collected target points
    score += g->targets_collected * target_point; 
    
    //add penality for obstacles and fences collision
    score -= g->obstacles_hit * obstacle_penalty; 
    score -= g->fence_collision * fence_penalty; 
    
    if (score < 0) score = 0; //minimum score is zero

    //update score
    g->score = score;

    //rreset counters to avoid multiple count
    g->targets_collected = 0; 
    g->obstacles_hit = 0; 
    g->fence_collision = 0;
    
    return score;
}