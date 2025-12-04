#include <math.h>

#include "world_physics.h"   
#include "map.h" 

typedef struct{
    double fx;
    double fy;
} Force;

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


// OBSTACLES - repulsion
static inline double pow_distance(double dx, double dy){
    double pow_d = dx*dx + dy*dy;
    return pow_d;
}

static Force add_obstacles_repulsion(GameState *gs){
    Force F = {0,0};
    double rho = gs->rho;
    double eta = gs->eta;
    double beta = gs->tangent_gain;

    for(int i=0; i<gs->num_obstacles; i++){
        double dx = (double)gs->drone.x - (double)gs->obstacles[i].x;
        double dy = (double)gs->drone.y - (double)gs->obstacles[i].y;

        double pow_d = pow_distance(dx, dy);
        if (pow_d < 1e-6) continue;
        double d = sqrt(pow_d);
        
        if(d<rho){
            // F_repulsion = eta * (1/d - 1/rho) * (1/d^2) * ((q - q_obs)/d), d=ρ(q)
            double F_repulsion = eta * (1/d - 1/rho) * (1/ pow_d);

            //radiant component (versor: (q - q_obs)/d)
            double nx = dx/d;
            double ny = dy/d;
            double Fr_x = F_repulsion*nx;
            double Fr_y = F_repulsion*ny;

            //tangent component (default: swirl to left)
            double tx = -ny;
            double ty = nx;
            double Ft_mag = beta * fabs(F_repulsion);
            double Ft_x = Ft_mag * tx;
            double Ft_y = Ft_mag * ty;

            //repulsive force
            F.fx += Fr_x + Ft_x;
            F.fy += Fr_y  +Ft_y;
        }
        gs->fx_obst = F.fx;
        gs->fy_obst = F.fy;
    }
    return F;
}


// FENCE - repulsion
static Force add_fence_repulsion(GameState *gs){
    Force F = {0,0};

    //if an obstacle is near the wall
    double min_d2 = 1e9;
    for (int i = 0; i < gs->num_obstacles; ++i) {
        double dx = gs->drone.x - gs->obstacles[i].x;
        double dy = gs->drone.y - gs->obstacles[i].y;
        double d2 = dx*dx + dy*dy;
        if (d2 < min_d2) min_d2 = d2;
    }

    double scale = 1.0;
    double rho_obs = gs->rho;
    if (min_d2 < rho_obs * rho_obs) {
        scale = 0.9;
    }

    double rho = gs->rho*0.5; //distance of wall's influence
    double eta = gs->eta*0.2*scale; //gain of wall's repulsion

    //border distance
    double bl = (double)gs->drone.x;
    double br = (double)(gs->world_width - 1 - gs->drone.x);
    double bt = (double)gs->drone.y;
    double bb = (double)(gs->world_height - 1 - gs->drone.y);

    if (bl < rho) {
        double d = bl;
        if (d < 1e-3) d = 1e-3;
        // F_fence = eta * (1/d - 1/rho) * (1/d) * (+1.0)
        double F_fence  = eta * (1.0/d - 1.0/rho) * (1.0/(d*d));
        F.fx += F_fence * (+1.0); //to right (+x)
    }
    if (br < rho) {
        double d = br;
        if (d < 1e-3) d = 1e-3;
        double F_fence  = eta * (1.0/d - 1.0/rho) * (1.0/(d*d));
        F.fx += F_fence * (-1.0); //to left (-x)
    }
    if (bt < rho) {
        double d = bt;
        if (d < 1e-3) d = 1e-3;
        double F_fence  = eta * (1.0/d - 1.0/rho) * (1.0/(d*d));
        F.fy += F_fence * (+1.0); //to down (+y)
    }
    if (bb < rho) {
        double d = bb;
        if (d < 1e-3) d = 1e-3;
        double F_fence  = eta * (1.0/d - 1.0/rho) * (1.0/(d*d));
        F.fy += F_fence * (-1.0); //to up (-y)
    }

    //control max value of the fence force
    double max_fance = gs->max_force * 2.0; 
    double magnitude  = sqrt(F.fx*F.fx + F.fy*F.fy);
    if (magnitude > max_fance && magnitude > 1e-9){
        F.fx *= max_fance / magnitude;
        F.fy *= max_fance / magnitude;
    }

    gs->fx_fence = F.fx;
    gs->fy_fence = F.fy;

    return F;
}


// TARGET - attraction
/*static Force add_targets_attraction(GameState *gs){
    
    Force F = {0,0};
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

    Force F_input = { gs->fx_cmd, gs->fy_cmd };
    Force F_repulsion = add_obstacles_repulsion(gs);
    Force F_fence = add_fence_repulsion(gs);
    //Force F_attraction = add_targets_attraction(gs);

    // F_tot = F_input + F_repulsion + F_fence + F_attraction
    double fx = F_input.fx + F_repulsion.fx + F_fence.fx /*+ F_attraction.fx*/;
    double fy = F_input.fy + F_repulsion.fy + F_fence.fy /*+ F_attraction.fy*/;

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

    //Euler - new position
    double new_x = gs->drone.x + gs->drone.vx * gs->dt;
    double new_y = gs->drone.y + gs->drone.vy * gs->dt;
    

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