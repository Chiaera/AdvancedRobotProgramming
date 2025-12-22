/* this file contains the function for the physics process
    - compute the input force
    - compute the repulsive force form the obstacles
    - compute the repulsive force from the fence
    - calculate the total force
*/

#include <math.h>

#include "drone_physics.h"   
#include "map.h" 

typedef struct{ //for save the values of the forces in the directions x and y
    double fx;
    double fy;
} Force;

// INPUT - direction
void add_direction(GameState *gs, int mx, int my){
    //update the forces based on direction
    gs->fx_cmd += gs->command_force * mx;  
    gs->fy_cmd += gs->command_force * my;

    //check if the forces is in the range correct range
    if (gs->fx_cmd > gs->max_force) gs->fx_cmd = gs->max_force;
    if (gs->fx_cmd < -gs->max_force) gs->fx_cmd = -gs->max_force;
    if (gs->fy_cmd > gs->max_force) gs->fy_cmd = gs->max_force;
    if (gs->fy_cmd < -gs->max_force) gs->fy_cmd = -gs->max_force;
}

// brake
void use_brake(GameState *gs){ 
    //linear decrease of the drone velocities and the command forces about a factor
    const double brake_factor = 0.5;

    gs->fx_cmd *= brake_factor;
    gs->fy_cmd *= brake_factor;
    gs->drone.vx *= brake_factor;
    gs->drone.vy *= brake_factor;
}


// OBSTACLES - repulsion (using Khatib's potential field)
//radial: distance from obstacle
//tangential: 'swirling' effect
static inline double pow_distance(double dx, double dy){ 
    double pow_d = dx*dx + dy*dy; //compute the pow distance d^2
    return pow_d;
}

static Force add_obstacles_repulsion(GameState *gs){
    Force F = {0,0}; //default force

    //set the variables with the config values
    double rho = gs->rho;
    double eta = gs->eta;
    double beta = gs->tangent_gain;

    for(int i=0; i<gs->num_obstacles; i++){ 
        //distance drone - obstacle
        double dx = (double)gs->drone.x - (double)gs->obstacles[i].x; 
        double dy = (double)gs->drone.y - (double)gs->obstacles[i].y;

        double pow_d = pow_distance(dx, dy); //find the pow distance d^2
        if (pow_d < 1e-6){  //consider a minimal distance - prevent division by zero when drone exactly on obstacle center
            pow_d = 1e-6;
        }
        double d = sqrt(pow_d); //find the vector distance d
        
        if(d<rho){
            //Khatib potential field: F_repulsion = eta * (1/d - 1/rho) * (1/d^2) * ((q - q_obs)/d), d=ρ(q)
            double F_repulsion = eta * (1/d - 1/rho) * (1/ pow_d);

            //radiant component (versor: (q - q_obs)/d)
            double nx = dx/d; //normalization
            double ny = dy/d;
            double Fr_x = F_repulsion*nx; //correct radiant force along x
            double Fr_y = F_repulsion*ny; //correct radiant force along y

            //tangent component (creates "swirling" effect around obstacles)
            double tx = -ny; //default swirl to left
            double ty = nx;
            double Ft_mag = beta * fabs(F_repulsion); //magnitude proportional to radial repulsion (perpendicular direction)
            double Ft_x = Ft_mag * tx; //correct tangent force along x
            double Ft_y = Ft_mag * ty; //correct tangent force along y

            //repulsive force
            F.fx += Fr_x + Ft_x; //correct repulsive force along x
            F.fy += Fr_y  +Ft_y; //correct repulsive force along y
        }
        //update the repulsion force from the obstacles in the GameState struct
        gs->fx_obst = F.fx; 
        gs->fy_obst = F.fy;
    }
    return F;
}


// FENCE - repulsion
static Force add_fence_repulsion(GameState *gs){
    Force F = {0,0}; //default force
    //add scale factor to prevent fence force from overwhelming other forces
    double rho = gs->rho*0.5; //distance of wall's influence
    double eta = gs->eta*0.2; //gain of wall's repulsion

    //border distance
    double bl = (double)gs->drone.x; //left
    double br = (double)(gs->world_width - 1 - gs->drone.x); //right
    double bt = (double)gs->drone.y; //top
    double bb = (double)(gs->world_height - 1 - gs->drone.y); //bottom

    if (bl < rho) { //near the border left 
        double d = bl; 
        if (d < 1e-3) d = 1e-3; //prevent division by zero
        // F_fence = eta * (1/d - 1/rho) * (1/d) * (+1.0)
        double F_fence  = eta * (1.0/d - 1.0/rho) * (1.0/(d*d));
        F.fx += F_fence * (+1.0); //to right (+x)
    }
    if (br < rho) {  //near the right border
        double d = br;
        if (d < 1e-3) d = 1e-3;
        double F_fence  = eta * (1.0/d - 1.0/rho) * (1.0/(d*d));
        F.fx += F_fence * (-1.0); //to left (-x)
    }
    if (bt < rho) { //near the top border
        double d = bt;
        if (d < 1e-3) d = 1e-3;
        double F_fence  = eta * (1.0/d - 1.0/rho) * (1.0/(d*d));
        F.fy += F_fence * (+1.0); //to down (+y)
    }
    if (bb < rho) { //near the bottom border
        double d = bb;
        if (d < 1e-3) d = 1e-3;
        double F_fence  = eta * (1.0/d - 1.0/rho) * (1.0/(d*d));
        F.fy += F_fence * (-1.0); //to up (-y)
    }

    //control max value of the fence force
    double max_fance = gs->max_force * 2.0; 
    double magnitude  = sqrt(F.fx*F.fx + F.fy*F.fy);
    if (magnitude > max_fance && magnitude > 1e-9){ //normalization
        F.fx *= max_fance / magnitude; //correct fence force along x
        F.fy *= max_fance / magnitude; //correct fence force along y
    }

    //save the forces in the GameState struct
    gs->fx_fence = F.fx;
    gs->fy_fence = F.fy;

    return F;
}


// TARGET - attraction
/*static Force add_targets_attraction(GameState *gs){
    
    Force F = {0,0}; //default force
    if (gs->num_targets <= 0) {
        return F;
    }
    int near_target = 0;
    double best_pow_d = 1e12;

    //nearest target
    for(int i=0; i<gs->num_targets; i++){
        double dx = (double)gs->targets[i].x - (double)gs->drone.x;
        double dy = (double)gs->targets[i].y - (double)gs->drone.y;
        double pow_d = pow_distance(dx, dy);
        if(pow_d < best_pow_d){
            best_pow_d = pow_d;
            near_target = i;
        }
    } 
    
    //F_attraction = zeta * (q_target - q)
    double dx = (double)gs->targets[near_target].x - (double)gs->drone.x;
    double dy = (double)gs->targets[near_target].y - (double)gs->drone.y;

    double zeta = gs->zeta;
    F.fx += zeta * dx;
    F.fy += zeta * dy;

    return F;
} */


// DRONE - physics
void add_drone_dynamics(GameState *gs){

    Force F_input = { gs->fx_cmd, gs->fy_cmd }; //set the command forces
    Force F_repulsion = add_obstacles_repulsion(gs); //compute the repiulsive force from obstacles
    Force F_fence = add_fence_repulsion(gs); //compute the repulsive force from the fence
    //Force F_attraction = add_targets_attraction(gs);

    // F_tot = F_input + F_repulsion + F_fence + F_attraction
    double fx = F_input.fx + F_repulsion.fx + F_fence.fx /*+ F_attraction.fx*/; //total force along x
    double fy = F_input.fy + F_repulsion.fy + F_fence.fy /*+ F_attraction.fy*/; //total force along y

    //save the total force in the GameState struct
    gs->fx_tot = fx;
    gs->fy_tot = fy;


    // a = (F - k*v)/M
    double ax = (fx - gs->k *gs->drone.vx) / gs->mass;
    double ay = (fy - gs->k * gs->drone.vy) / gs->mass;

    // v = v_old + a*dt 
    gs->drone.vx += ax * gs->dt;
    gs->drone.vy += ay * gs->dt;

    /*//Check on the max velocity - no tunnelling effect
    double max_vel = gs->max_force / gs->k; 
    double current_vel_sq = gs->drone.vx * gs->drone.vx + gs->drone.vy * gs->drone.vy;
    if (current_vel_sq > max_vel * max_vel) {
        double current_vel = sqrt(current_vel_sq);
        gs->drone.vx *= max_vel / current_vel;
        gs->drone.vy *= max_vel / current_vel;
    }*/

    //Euler - save the new position
    double new_x = gs->drone.x + gs->drone.vx * gs->dt;
    double new_y = gs->drone.y + gs->drone.vy * gs->dt;

    //threshold
    double r_coll = 1.2;  
    double r2 = r_coll*r_coll;

    for (int i = 0; i < gs->num_obstacles; ++i) {
        //save the coordinates in for the i-th obstacle
        double ox = (double)gs->obstacles[i].x;
        double oy = (double)gs->obstacles[i].y;

        //compute the new distance
        double dx = new_x - ox;
        double dy = new_y - oy;
        double d2 = dx*dx + dy*dy;

        //collision check
        if (d2 < r2) { 
            if(d2 < 1e-9){ //prevent division by zero
                d2 = 1e-9;
            }
            double d = sqrt(d2);

            //threshold around the obstacle
            double scale = r_coll / d;
            new_x = ox + dx * scale;
            new_y = oy + dy * scale;
            break; 
        }
    }


    //border clamp 
    if (new_x < 0.0) {
        new_x = 0.0;
        gs->drone.vx = 0.0;  
    }
    if (new_x >= gs->world_width) {
        new_x = gs->world_width - 0.001; 
        gs->drone.vx = 0.0;
    }
    if (new_y < 0.0) {
        new_y = 0.0;
        gs->drone.vy = 0.0;
    }
    if (new_y >= gs->world_height) {
        new_y = gs->world_height - 0.001;
        gs->drone.vy = 0.0;
    }
    
    //save position
    gs->drone.x = new_x;
    gs->drone.y =  new_y;
}