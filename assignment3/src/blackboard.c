/* This file containts the world server:
    - initialize the struct fot the pipe messages
    - initialize the world variable with their value read from the config file
    - create the ncurses windows 
    - read the obstacle position from the msgObstacle and add the element on the map
    - read the target position from the msgTarget and add the element on the map
    - management the physics of the world

    - utility for the watchdog
        - create the shared memory for the watchdog (used as a heartbeat table) and the semaphore for its safety
        - initializes the heartbeat table to zero
        - reflesh the slot to segnalize its activity
 */

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>  
#include <sys/select.h>
#include <string.h>
#include <ncurses.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdint.h>
#include <sys/wait.h>
#include <signal.h>
#include <pthread.h>  

#include "heartbeat.h"
#include "map.h"
#include "world.h"
#include "drone_physics.h"
#include "logger.h"
#include "network.h" 

#define LOG_PATH "logs/"

NetworkState g_net_state; //common state with the thread of network
pthread_mutex_t g_net_mutex = PTHREAD_MUTEX_INITIALIZER;  //mutex to protect the g_net_state
// --------------------------------------------------------------- STRUCT
typedef struct { //use for the input messages, x and y to divide the input force in its directions
    char type;
    int dx, dy;
} msgInput;

typedef struct  { //use for the drone message, x and y to check the drone position
    char type;
    int x, y;
} msgDrone;

typedef struct { //use for the target messages, define the number of the targets
    char type;
    int num;
    Target targets[MAX_TARGETS];
} msgTargets;

typedef struct  { //use use for the obstacles messages, define the number of the obstacles
    char type;
    int num;
    Obstacle obstacles[MAX_OBSTACLES];
} msgObstacles;


// read Config file
static void load_config(const char *path, Config *cfg) {

    memset(cfg, 0, sizeof(Config)); //initialize the byte of the message
    
//-------------------------------------------------------------- READ CONFIG

    FILE *f = fopen(path, "r"); //read config file
    if (!f) { //debug for the reading of the file
        fprintf(stderr, "Error in reading parameters.config %s\n", path); 
        fprintf(stderr, "Use default values.\n");
        return;
    }

    //parsig: iniitalize all the variables with the value read from the config
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char key[128], value[128];

        if (sscanf(line, "%127[^=]=%127s", key, value) == 2) {
            //size
            if (!strcmp(key, "WORLD_WIDTH"))  cfg->world_width  = atoi(value);
            else if (!strcmp(key, "WORLD_HEIGHT")) cfg->world_height = atoi(value);

            //physics
            else if (!strcmp(key, "MASS")) cfg->mass = atof(value);
            else if (!strcmp(key, "K")) cfg->k = atof(value);
            else if (!strcmp(key, "DT")) cfg->dt = atof(value);
            else if (!strcmp(key, "COMMAND_FORCE")) cfg->command_force = atof(value);
            else if (!strcmp(key, "MAX_FORCE")) cfg->max_force = atof(value);
            else if (!strcmp(key, "RHO")) cfg->rho = atof(value);
            else if (!strcmp(key, "ETA")) cfg->eta = atof(value);
            else if (!strcmp(key, "ZETA")) cfg->zeta = atof(value);
            else if (!strcmp(key, "TANGENT_GAIN")) cfg->tangent_gain = atof(value);            

            //drone
            else if (!strcmp(key, "DRONE_START_X")) cfg->drone_start_x = atoi(value);
            else if (!strcmp(key, "DRONE_START_Y")) cfg->drone_start_y = atoi(value);

            //target
            else if (!strcmp(key, "NUM_TARGETS")) cfg->num_targets = atoi(value);

            //obstacles
            else if (!strcmp(key, "NUM_OBSTACLES")) cfg->num_obstacles = atoi(value);

            //network
            else if (!strcmp(key, "NETWORK_ENABLED")) cfg->network_enabled = atoi(value);
            else if (!strcmp(key, "NETWORK_PORT")) cfg->network_port = atoi(value);
            else if (!strcmp(key, "NETWORK_MAX_EXTERNAL_OBSTACLES")) cfg->network_max_external_obstacles = atoi(value);
            else if (!strcmp(key, "NETWORK_CLIENT_TARGET")) strncpy(cfg->network_client_target, value, sizeof(cfg->network_client_target) - 1);
        }
    }

    fclose(f);
}

//use the new parameters in the gamestate variables
void apply_new_parameters(GameState *gs, Config *cfg) {
    gs->mass = cfg->mass;
    gs->k = cfg->k;
    gs->dt = cfg->dt;
    gs->command_force = cfg->command_force;
    gs->max_force = cfg->max_force;
    gs->rho = cfg->rho;
    gs->eta = cfg->eta;
    gs->zeta = cfg->zeta;
    gs->tangent_gain = cfg->tangent_gain;
    gs->world_width = cfg->world_width;
    gs->world_height = cfg->world_height;
}

//for the WATCHDOG
static volatile sig_atomic_t g_stop = 0; //used for the shutdown request
static volatile sig_atomic_t g_sighup = 0; //for sighup handler -> clean shutdown

static void on_watchdog_stop(int sig) {
    (void)sig;
    g_stop = 1; //flag if blackboard and watchdog can not comunicate
}

static void on_sighup(int sig) { //to avoid abrupt closure of the blackboard -> watchdog need to track this closure
    (void)sig;
    log_message("BLACKBOARD", "window closed: received SIGHUP");
    g_stop = 1;
}

static void on_sigint(int sig) { //handler for ctrl+c
    (void)sig;
    log_message("BLACKBOARD", "received SIGINT (Ctrl+C), shutting down");
    g_stop = 1;
}



//helper function
void print_help() {
    printf("Drone Simulator\n");
    printf("Options:\n");
    printf("  -h, --help        Show this help message\n\n");

    printf("Controls:\n");
    printf("  w/e/r/s/d/f/x/c/v   Movement keys\n");
    printf("  d                   Brake\n");
    printf("  q                   Quit\n\n");

    printf("Signals:\n");
    printf("  kill -SIGUSR2 <pid>   Reload configuration\n");
}

//used to correctly terminate child processes 
static void wait_and_log(pid_t pid, const char *name) {
    int status;
    waitpid(pid, &status, 0);

    if (WIFEXITED(status)) {
        log_message("BLACKBOARD", "%s exited with code %d",
                    name, WEXITSTATUS(status));
    } else if (WIFSIGNALED(status)) {
        log_message("BLACKBOARD", "%s killed by signal %d",
                    name, WTERMSIG(status));
    }
}

//used to avoid zombie processes
static void wait_pid(pid_t pid, const char *name) {
    int status;
    if (waitpid(pid, &status, 0) > 0) {
        if (!WIFEXITED(status)) { //if the process did not terminate correctly
            kill(pid, SIGKILL);  
        }
    }
    log_message("BLACKBOARD", "%s terminated", name);
}


//used to print final statistics of END-GAME
int print_final_win(GameState *gs, pid_t pid_watchdog) {
    //kills watchtdog
    if (pid_watchdog > 0) { 
        kill(pid_watchdog, SIGTERM);
    }

    //clear the screen
    clear(); 
    refresh();

    //open final statistics window
    WINDOW *final_win = newwin(10, 50, LINES/2 - 5, COLS/2 - 25); 
    box(final_win, 0, 0);

    wattron(final_win, COLOR_PAIR(4) | A_BOLD);
    mvwprintw(final_win, 1, 2, "You have collected ALL THE TARGETS!");
    wattroff(final_win, COLOR_PAIR(4) | A_BOLD);      
    mvwprintw(final_win, 3, 2, "Final Statistics:");
    mvwprintw(final_win, 4, 4, "Score: %d", gs->score);
    mvwprintw(final_win, 5, 4, "Obstacles hit: %d", gs->obstacles_hit_tot);
    mvwprintw(final_win, 6, 4, "Fence collisions: %d", gs->fence_collision_tot);
    mvwprintw(final_win, 8, 2, "Press 'e' to exit");
    wrefresh(final_win);

    //exit game
    int ch;
    do {
    ch = getch(); 
    } while (ch != 'e' && ch != 'E');
    delwin(final_win);

    return 1; //signal to stop the main loop
}

// ----------------------------------------------------------- MAIN

int main(int argc, char *argv[])
{  
    if (argc == 1) {
        printf("--help to see the help commands\n");
    }

    static int bb_log_counter = 0; //to avoid the child log write on the initial log

    //log and setup watchdog
    log_message("BLACKBOARD", "[BOOT] Blackboard awakes", bb_log_counter++); //start log
    unlink("logs/processes.pid"); //addictional control to remove olds pid file
    register_process("BLACKBOARD"); //register blackboard pid in the pid file
    //shm_unlink(HB_SHM_NAME); // ignore errors before the processes 'wake up'

    //parameters -------------------------------------------------------------------
    Config cfg;
    load_config("bin/parameters.config", &cfg);
    log_message("BLACKBOARD", "[BOOT] Config loaded: %dx%d world, %d obstacles, %d targets",
                cfg.world_width, cfg.world_height, cfg.num_obstacles, cfg.num_targets, bb_log_counter++);
    
    //helper
    if (argc > 1 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) {
        print_help();
        return 0;
    }
    
    //ncurses -----------------------------------------------------------------
    initscr(); //initialize
    int old_lines = LINES;
    int old_cols  = COLS;

    start_color();
    //yellow target
    init_pair(1, COLOR_YELLOW, COLOR_BLACK);
    //magenta obstacles
    init_pair(2, COLOR_MAGENTA, COLOR_BLACK);
    //green drone
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    //cyan color
    init_pair(4, COLOR_CYAN, COLOR_BLACK); 

    noecho();
    curs_set(0);
    srand(time(NULL));

    //create windows ----------------------------------------------------------------
    Screen screen; //initialize the screen 
    init_screen(&screen);

    //create the inspection window
    WINDOW *info_win = newwin(8, 40, 0, 2); //info window
    box(info_win, 0, 0);
    mvwprintw(info_win, 0, 2, "[ Info ]");

    WINDOW *processes_win = newwin(6, 40, 0, 60); //processes pid window
    box(processes_win, 0, 0);
    mvwprintw(processes_win, 0, 2, "[ Processes ]");

    WINDOW *collision_win = newwin(5, 40, 8, 2); //collision window
    box(collision_win, 0, 0);
    mvwprintw(collision_win, 0, 2, "[ Collisions ]");

    WINDOW *help_win = newwin(5, 40, 8, 60); //help window
    box(help_win, 0, 0);
    mvwprintw(help_win, 0, 2, "[ Help  ]");


    //initialize the variables of the gamestate struct --------------------------------
    GameState gs; 
    init_game(&gs, &cfg);

    //network initialization --------------------------------------------------------------
    if (cfg.network_enabled) {
        int ret = network_state_init(
            &g_net_state,
            cfg.world_width,
            cfg.world_height,
            cfg.network_max_external_obstacles  
        );
        
        if (ret != 0) {
            log_message("BLACKBOARD", "ERROR: Failed to initialize network state");
            cfg.network_enabled = 0;
        } else {
            log_message("BLACKBOARD", "Network state initialized (max external obstacles: %d)", 
                        cfg.network_max_external_obstacles);
        }
    }

    // SHM ----------------------------------------------------------------------------------------------------------------------
    //create shared memory
    int hb_fd = shm_open(HB_SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (hb_fd < 0) { 
        perror("shm_open"); 
        goto cleanup;
    }

    //resize shm to fit the struct
    if (ftruncate(hb_fd, sizeof(HeartbeatTable)) < 0) {
        perror("ftruncate"); 
        goto cleanup;
    }

    //mapping shm to the memory address
    HeartbeatTable *hb = mmap(NULL, sizeof(HeartbeatTable), PROT_READ | PROT_WRITE, MAP_SHARED, hb_fd, 0);
    if (hb == MAP_FAILED) { 
        perror("mmap"); 
        goto cleanup;
    }

    //initialize the heartbeat table to zero
    memset(hb, 0, sizeof(*hb));

    //initialize the semaphore
    if (sem_init(&hb->mutex, 1, 1) == -1) {  //(mutex, shared between processes, initial value)
        perror("sem_init");
        goto cleanup;
    }
    log_message("BLACKBOARD", "[BOOT] Heartbeat semaphore initialized", bb_log_counter++);

    struct timespec ts = {0, 200 * 1000 * 1000};  //delay for wait the log to write in the system.log (200ms)
    nanosleep(&ts, NULL);

    //save in the blackboard info in its heartbeat slot ---------------------------------
    sem_wait(&hb->mutex); //lock the heartbeat table
    hb->entries[HB_SLOT_BLACKBOARD].pid = getpid();
    hb->entries[HB_SLOT_BLACKBOARD].last_seen_ms = now_ms(); //tells the watchdog it is stil active
    sem_post(&hb->mutex); //unlock the heartbeat table

    //signal handlers ----------------------------------------------------------------
    //handler to sigusr1 (watchdog request to stop)
    struct sigaction sa; 
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_watchdog_stop;
    sigaction(SIGUSR1, &sa, NULL);

    //handler to sigint (ctrl+c)
    struct sigaction sa_int; 
    memset(&sa_int, 0, sizeof(sa_int));
    sa_int.sa_handler = on_sigint;
    sigaction(SIGINT, &sa_int, NULL);

    //handler to SIGHUP (window close)
    struct sigaction sa_hup;   //handler to sighup (window close)
    memset(&sa_hup, 0, sizeof(sa_hup));
    sa_hup.sa_handler = on_sighup;
    sigaction(SIGHUP, &sa_hup, NULL);

    // FORK ------------------------------------------------------------------------------------------------------------------------

    //implementedsignal mask to avoid wrong sighup and sigusr1 in child processes
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGHUP);
    sigaddset(&mask, SIGUSR1);

    // define the pipes
    int pipe_input[2];
    int pipe_drone[2];
    int pipe_obstacles[2];
    int pipe_targets[2];
    
    //check pipes creation 
    if (pipe(pipe_input) == -1) {
        perror("pipe creation failed");
        log_message("BLACKBOARD", "cannot create pipes");
        exit(EXIT_FAILURE);
    }
    if (pipe(pipe_drone) == -1) {
        perror("pipe creation failed");
        log_message("BLACKBOARD", "cannot create pipes");
        exit(EXIT_FAILURE);
    }
    if (pipe(pipe_obstacles) == -1) {
        perror("pipe creation failed");
        log_message("BLACKBOARD", "cannot create pipes");
        exit(EXIT_FAILURE);
    }
    if (pipe(pipe_targets) == -1) {
        perror("pipe creation failed");
        log_message("BLACKBOARD", "cannot create pipes");
        exit(EXIT_FAILURE);
    }

    //initialize pid (process not alive yet)
    pid_t pid_input = -1;
    pid_t pid_drone = -1;
    pid_t pid_targets = -1;
    pid_t pid_obstacles = -1;
    pid_t pid_watchdog = -1;

    if (sigprocmask(SIG_BLOCK, &mask, NULL) == -1) { //active mask to avoid wrong signals
        perror("sigprocmask");
        exit(1);
    }
    log_message("BLACKBOARD", "[BOOT] Signal mask active", bb_log_counter++);

    //input
    pid_input = fork();
    if(pid_input < 0) { // error
        perror("fork failed for process_input");
        log_message("BLACKBOARD", "ERROR: fork failed for INPUT");
        g_stop = 1;
        exit(1);
    } else if (pid_input == 0){ //child process of the input process
        //pipes not used
        close(pipe_input[0]); 
        close(pipe_drone[0]);
        close(pipe_drone[1]);
        close(pipe_targets[0]);
        close(pipe_targets[1]);
        close(pipe_obstacles[0]);
        close(pipe_obstacles[1]);

        char fd_str[16];
        snprintf(fd_str, sizeof(fd_str), "%d", pipe_input[1]);

        char slot_str[8]; //used for the watchdog processes
        snprintf(slot_str, sizeof(slot_str), "%d", HB_SLOT_INPUT);
        execlp("konsole", "konsole", "-e", "./build/bin/process_input",
            fd_str, HB_SHM_NAME, slot_str, (char *)NULL);
        perror("execlp process_input failed");
        _exit(1);
    } else {
        log_message("BLACKBOARD", "Forked INPUT process with PID=%d", pid_input);
    }

    // drone
    pid_drone = fork();
    if(pid_drone < 0){ //error
        perror("fork failed for process_drone");
        log_message("BLACKBOARD", "ERROR: fork failed for DRONE");
        g_stop = 1;
        exit(1);
    } else if(pid_drone == 0){ //child process of the drone process
        //pipes not used
        close(pipe_drone[0]);
        close(pipe_input[0]);
        close(pipe_input[1]);
        close(pipe_targets[0]);
        close(pipe_targets[1]);
        close(pipe_obstacles[0]);
        close(pipe_obstacles[1]);

        char fd_str[16];
        snprintf(fd_str, sizeof(fd_str), "%d", pipe_drone[1]);
        
        char slot_str[8]; //used for the watchdog processes
        snprintf(slot_str, sizeof(slot_str), "%d", HB_SLOT_DRONE);
        execlp("./build/bin/process_drone", "./build/bin/process_drone",
            fd_str, HB_SHM_NAME, slot_str, (char *)NULL);
        perror("execlp process_drone failed");
        _exit(1);
    } else {
        log_message("BLACKBOARD", "Forked DRONE process with PID=%d", pid_drone);
    }

    // target
    pid_targets = fork();
    if(pid_targets < 0){ //error
        perror("fork failed for process_targets");
        log_message("BLACKBOARD", "ERROR: fork failed for TARGETS");
        g_stop = 1;
        exit(1);
    } else if (pid_targets == 0) { //child process of the targets process
        //pipes not used
        close(pipe_targets[0]); //use only pipe_targets[1] for the tick
        close(pipe_input[0]);
        close(pipe_input[1]);
        close(pipe_drone[0]);
        close(pipe_drone[1]);
        close(pipe_obstacles[0]);
        close(pipe_obstacles[1]);

        char fd_str[16];
        snprintf(fd_str, sizeof(fd_str), "%d", pipe_targets[1]);
        
        char slot_str[8];
        snprintf(slot_str, sizeof(slot_str), "%d", HB_SLOT_TARGETS);

        execlp("./build/bin/process_targets", "./build/bin/process_targets",
            fd_str, HB_SHM_NAME, slot_str, (char*)NULL);

        perror("execlp process_targets failed");
        _exit(1);
    } else {
        log_message("BLACKBOARD", "Forked TARGETS process with PID=%d", pid_targets);
    }
    
    // obstacles
    pid_obstacles = fork();
    if(pid_obstacles < 0){ //error
        perror("fork failed for process_obstacles");
        log_message("BLACKBOARD", "ERROR: fork failed for OBSTACLES");
        g_stop = 1;
        exit(1);
    } else if(pid_obstacles == 0){ //child process of the obstacles process
        //pipes not used
        close(pipe_obstacles[0]); //use only pipe_obstacles[1] for the tick
        close(pipe_input[0]);
        close(pipe_input[1]);
        close(pipe_drone[0]);
        close(pipe_drone[1]);
        close(pipe_targets[0]);
        close(pipe_targets[1]);
        char fd_str[16];
        snprintf(fd_str, sizeof(fd_str), "%d", pipe_obstacles[1]);
        
        char slot_str[8]; //used for the watchdog processes
        snprintf(slot_str, sizeof(slot_str), "%d", HB_SLOT_OBSTACLES);
        execlp("./build/bin/process_obstacles", "./build/bin/process_obstacles",
            fd_str, HB_SHM_NAME, slot_str, (char *)NULL);

        perror("execlp process_obstacles failed");
        _exit(1);
    } else {
        log_message("BLACKBOARD", "Forked OBSTACLES process with PID=%d", pid_obstacles);
    }

    if (g_stop) {
        log_message("BLACKBOARD", "Fork failed, skipping main loop");
        goto cleanup;
    }

    //debug
    log_message("BLACKBOARD", "All processes forked successfully");

    //watchdog
    pid_watchdog = fork();
    if (pid_watchdog < 0) {
        perror("fork failed for watchdog");
        log_message("BLACKBOARD", "ERROR: fork failed for WATCHDOG");
        g_stop = 1;
        exit(1);
    } else if (pid_watchdog == 0) {
        char timeout_str[16];
        snprintf(timeout_str, sizeof(timeout_str), "%d", 2000); // 2s timeout

        execlp("./build/bin/watchdog",  "./build/bin/watchdog",
            HB_SHM_NAME, timeout_str, (char *)NULL);

        perror("execlp watchdog failed");
        _exit(1);
    }

    //network threads - if abilited
    pthread_t server_thread = 0; 
    pthread_t client_thread = 0;
    int have_server = 0; 
    int have_client = 0;

    if (cfg.network_enabled) {
        log_message("BLACKBOARD", "[NETWORK] Starting network threads");
        
        //active server thread
        have_server = (pthread_create( &server_thread, NULL, network_server_thread, &cfg.network_port ) == 0);
        
        if (have_server) {
            log_message("BLACKBOARD", "[NETWORK] Server thread started (port: %d)", cfg.network_port);
        } else {
            log_message("BLACKBOARD", "[NETWORK] ERROR: Failed to create server thread");
        }
        
        //active client thread 
        if (cfg.network_client_target[0] != '\0') {  //check if the target is not empty
            have_client = (pthread_create( &client_thread, NULL, network_client_thread, cfg.network_client_target ) == 0);
            
            if (have_client) {
                log_message("BLACKBOARD", "[NETWORK] Client thread started (target: %s)", cfg.network_client_target);
            } else {
                log_message("BLACKBOARD", "[NETWORK] ERROR: Failed to create client thread");
            }
        }
    }

    sigprocmask(SIG_UNBLOCK, &mask, NULL); //deactive signalmask 
    log_message("BLACKBOARD", "[BOOT] Signal mask deactive", bb_log_counter++);

    //close write 
    close(pipe_input[1]);
    close(pipe_drone[1]);
    close(pipe_obstacles[1]);
    close(pipe_targets[1]);

    // READ MESSAGES -------------------------------------------------------------

    // messagge by process_obstacles
    msgObstacles msg_obstacles;
    ssize_t n = read(pipe_obstacles[0], &msg_obstacles, sizeof(msg_obstacles));
    if (n == sizeof(msg_obstacles) && msg_obstacles.type == 'O') { //define the obstacle as 'O'
        gs.num_obstacles = msg_obstacles.num;
        if (gs.num_obstacles > MAX_OBSTACLES) gs.num_obstacles = MAX_OBSTACLES;

        for (int i = 0; i < gs.num_obstacles; i++) { //loop to associate every obstacle to its (x,y) coordinates
            gs.obstacles[i].x = msg_obstacles.obstacles[i].x;
            gs.obstacles[i].y = msg_obstacles.obstacles[i].y;
        }
    } else {
        gs.num_obstacles = 0;
    }
    // position check
    for (int i = 0; i < gs.num_obstacles; i++) {
        if (gs.obstacles[i].x == gs.drone.x && gs.obstacles[i].y == gs.drone.y) {
            respawn_obstacle(&gs, i); //find a new coordinates for the i-th obstacles
        }
    }
    
    // massage by process_targets 
    msgTargets msg_t;
    ssize_t nt = read(pipe_targets[0], &msg_t, sizeof(msg_t));

    if (nt == sizeof(msg_t) && msg_t.type == 'T') { //define the target as 'T'
        gs.num_targets = msg_t.num;
        if (gs.num_targets > MAX_TARGETS) gs.num_targets = MAX_TARGETS;

        for (int i = 0; i < gs.num_targets; i++) { //loop to associate every target to its (x,y) coordinates
            gs.targets[i].x = msg_t.targets[i].x;
            gs.targets[i].y = msg_t.targets[i].y;
        }
    } else {
        gs.num_targets = 0;
    }
    //position check
    for (int i = 0; i < gs.num_targets; i++) {
        int tx = gs.targets[i].x;
        int ty = gs.targets[i].y;
        for (int j = 0; j < gs.num_obstacles; j++) { //check if an obstacle already exists 
            if (gs.obstacles[j].x == tx && gs.obstacles[j].y == ty) {
                respawn_target(&gs, i); //find a new coordinates for the i-th target
                break;
            }
        }
    }

    // SELECT---------------------------------------------------------------

    fd_set set; //define set of the file to 'listen'
    //select the number of descriptor
    int maxfd = pipe_input[0];
    if (pipe_drone[0] > maxfd) maxfd = pipe_drone[0];
    if (pipe_targets[0] > maxfd) maxfd = pipe_targets[0];
    if (pipe_obstacles[0] > maxfd) maxfd = pipe_obstacles[0];  
    maxfd += 1;

    while (1){

        if (g_stop) { // watchdog requested shutdown
            log_message("BLACKBOARD", "Watchdog requested shutdown: send SIGUSR1 to blackboard");            
            break;   
        }
        if (g_sighup) {
            log_message("BLACKBOARD", "Terminal closed (SIGHUP), shutting down");
            break;
        }

        sem_wait(&hb->mutex); //lock the heartbeat table
        hb->entries[HB_SLOT_BLACKBOARD].last_seen_ms = now_ms();  //reflesh the slot (for the wathcdog) every iteration - it is indipendent from pipes
        sem_post(&hb->mutex); //unlock the heartbeat table

        FD_ZERO(&set);
        FD_SET(pipe_input[0], &set);
        FD_SET(pipe_drone[0], &set);
        FD_SET(pipe_targets[0], &set);
        FD_SET(pipe_obstacles[0], &set); 

        //timer to update the heartbeat - small timeout for a periodic refresh
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 100 ms

        int rc = select(maxfd, &set, NULL, NULL, &tv); //listen to all the set of processes
        if (rc < 0) {
            if (errno == EINTR) continue;   // resize
                perror("select");
                break;
        }

        // INPUT 
        if (FD_ISSET(pipe_input[0], &set)) {
            msgInput m;
            ssize_t ri = read(pipe_input[0], &m, sizeof(m));         
            if (ri != sizeof(m)) continue;  //error of reading
              
            if (m.type == 'Q') {
                log_message("BLACKBOARD", "Quit: shutting down");
                
                //kills exist processes
                kill(pid_input, SIGTERM);
                kill(pid_drone, SIGTERM);
                kill(pid_targets, SIGTERM);
                kill(pid_obstacles, SIGTERM);
                kill(pid_watchdog, SIGTERM);
                break; //quit
            } else if (m.type == 'I') {  //direction: separation in x and y
                int mx = m.dx;
                int my = m.dy;
                add_direction(&gs, mx, my); //use to compute the input force in the pshysics process
            }
            else if (m.type == 'B') {  //brake
                use_brake(&gs);
            }/* else if (m.type == 'P'){ //read parameters
                load_config("bin/parameters.config", &cfg);
                apply_new_parameters(&gs, &cfg);
            }*/
        }

        // DRONE - drone dynamics
        if(FD_ISSET(pipe_drone[0], &set)){
            msgDrone m;
            ssize_t rd= read(pipe_drone[0], &m, sizeof(m));         //timer callout: update the drone dynamics
            if (rd != sizeof(m)) continue;  //error of reading
            
            add_drone_dynamics(&gs); 
        }

        // TARGET - respawn
        if (FD_ISSET(pipe_targets[0], &set)) {
            msgTargets mt;
            ssize_t nr = read(pipe_targets[0], &mt, sizeof(mt)); //timer callout: change targets position
            if (nr != sizeof(mt)) continue; //error of reading

            if (mt.type == 'R') {
                int n = mt.num;
                if (n > gs.num_targets) {
                    n = gs.num_targets; // relocation of the remains targets. They should be the same for architecture choices
                }
                for (int i = 0; i < n; i++) {
                    gs.targets[i] = mt.targets[i]; //new vector for the remains targets 
                }
                log_message("BLACKBOARD", "Target respawned");
                
                //check overlap with obstacles
                for (int i = 0; i < n; i++) {
                    for (int j = 0; j < gs.num_obstacles; j++) {
                        if (gs.targets[i].x == gs.obstacles[j].x && gs.targets[i].y == gs.obstacles[j].y) {
                            respawn_target(&gs, i);
                            break;
                        }
                    }
                }
            }
        }

        // OBSTACLES - respawn
        if (FD_ISSET(pipe_obstacles[0], &set)) {
            msgObstacles mo;
            ssize_t no = read(pipe_obstacles[0], &mo, sizeof(mo)); //timer callout: change obstacles position
            if (no != sizeof(mo)) continue; //error of reading

            if (mo.type == 'R') {                
                int n = mo.num;
                if (n > gs.num_obstacles) {
                    n = gs.num_obstacles; // relocation of the obstacles
                }
                
                for (int i = 0; i < n; i++) { //new vector of obstacles used for the respawn
                    gs.obstacles[i] = mo.obstacles[i]; 
                }

                //check position
                for (int i = 0; i < n; i++) {
                    //no overlap with drone
                    if (gs.obstacles[i].x == (int)gs.drone.x && gs.obstacles[i].y == (int)gs.drone.y) {
                        respawn_obstacle(&gs, i);
                    }
                }
            }
        }

        //check position - respect of the map border 
        if (gs.drone.x < 0) gs.drone.x = 0;
        if (gs.drone.y < 0) gs.drone.y = 0;
        if (gs.drone.x >= gs.world_width) gs.drone.x = gs.world_width - 1;
        if (gs.drone.y >= gs.world_height) gs.drone.y = gs.world_height - 1;

        //resize
        if (LINES != old_lines || COLS != old_cols) {
        old_lines = LINES;
        old_cols  = COLS;
        refresh_screen(&screen);
        }

        drone_target_collide(&gs); //manages the collision

        //update NetworkState with the new drone position
        if (cfg.network_enabled) {
            pthread_mutex_lock(&g_net_mutex);
            g_net_state.drone_x = gs.drone.x;
            g_net_state.drone_y = gs.drone.y;
            pthread_mutex_unlock(&g_net_mutex);
        }

        //uploard external obstacles from NetworkState to GameState
        if (cfg.network_enabled) {
            pthread_mutex_lock(&g_net_mutex);
            
            //copy external obstacles in GameState
            for (int i = 0; i < g_net_state.num_external_obstacles; i++) {
                if (g_net_state.external_obstacles[i].active && i < MAX_OBSTACLES) {
                    gs.external_obstacles[i].x = g_net_state.external_obstacles[i].x;
                    gs.external_obstacles[i].y = g_net_state.external_obstacles[i].y;
                }
            }
            gs.num_external_obstacles = g_net_state.num_external_obstacles;
            
            pthread_mutex_unlock(&g_net_mutex);
        }

        calculate_final_score(&gs); //update score
        
        //END-GAME
        if (gs.current_target_index >= gs.total_targets) {
            pthread_mutex_lock(&g_net_mutex);
            g_net_state.server_running = 0;  
            pthread_mutex_unlock(&g_net_mutex);

            if (gs.current_target_index >= gs.total_targets) {
                log_message("BLACKBOARD", "All targets collected");

                //kills exist processes
                kill(pid_watchdog, SIGTERM); //first kill watchdog to avoid wrong sigusr1
                kill(pid_input, SIGTERM); 
                kill(pid_drone, SIGTERM); 
                kill(pid_targets, SIGTERM); 
                kill(pid_obstacles, SIGTERM); 

                log_message("BLACKBOARD", "Open the final statistics");
                g_stop = print_final_win(&gs, pid_watchdog); //call function to print final statistics
                log_message("BLACKBOARD", "Final statistics, shutting down");

                break; //exit the main loop
            }
        }

        render(&screen, &gs);
        
        //debug - print inspection windows
        werase(info_win);
        box(info_win, 0, 0);
        mvwprintw(info_win, 0, 2, "[ Info ]");
        mvwprintw(info_win, 1, 2, "Cmd: fx=%.2f fy=%.2f", gs.fx_cmd, gs.fy_cmd);
        mvwprintw(info_win, 2, 2, "Obst: fx=%.2f fy=%.2f", gs.fx_obst, gs.fy_obst);
        mvwprintw(info_win, 3, 2, "Fence: fx=%.2f fy=%.2f", gs.fx_fence, gs.fy_fence);
        mvwprintw(info_win, 4, 2, "Vel: vx=%.2f vy=%.2f", gs.drone.vx, gs.drone.vy);
        mvwprintw(info_win, 5, 2, "Pos: x=%.2f y=%.2f", gs.drone.x, gs.drone.y);
        mvwprintw(info_win, 6, 2, "Targets: %d/%d", gs.total_target_collected, gs.total_targets);
        wrefresh(info_win);

        werase(processes_win);
        box(processes_win, 0, 0);
        mvwprintw(processes_win, 0, 2, "[ Processes ]");
        mvwprintw(processes_win, 1, 2, "Input PID: %d", hb->entries[HB_SLOT_INPUT].pid); 
        mvwprintw(processes_win, 2, 2, "Drone PID: %d", hb->entries[HB_SLOT_DRONE].pid); 
        mvwprintw(processes_win, 3, 2, "Targets PID: %d", hb->entries[HB_SLOT_TARGETS].pid); 
        mvwprintw(processes_win, 4, 2, "Obstacles PID: %d", hb->entries[HB_SLOT_OBSTACLES].pid); 
        wrefresh(processes_win);

        werase(collision_win);
        box(collision_win, 0, 0);
        mvwprintw(collision_win, 0, 2, "[ Collisions ]");
        mvwprintw(collision_win, 1, 2, "Obstacles hit: %d", gs.obstacles_hit_tot); 
        mvwprintw(collision_win, 2, 2, "Fence hit: %d", gs.fence_collision_tot); 
        mvwprintw(collision_win, 3, 2, "Score: %d", gs.score);
        wrefresh(collision_win);

        werase(help_win);
        box(help_win, 0, 0);
        mvwprintw(help_win, 0, 2, "[ Help ]");
        mvwprintw(help_win, 1,2, "Run 'make help' in the terminal");
        mvwprintw(help_win, 2,2, "to see all available commands."); 
        wrefresh(help_win);
    }

    if (g_stop == 1 || g_sighup){ //normal shutdown
        //kills exist processes
        kill(pid_input, SIGTERM);
        kill(pid_drone, SIGTERM);
        kill(pid_targets, SIGTERM);
        kill(pid_obstacles, SIGTERM);
        kill(pid_watchdog, SIGTERM);
    }

    //wait for child processes to terminate
    wait_and_log(pid_input, "INPUT");
    wait_and_log(pid_drone, "DRONE");
    wait_and_log(pid_targets, "TARGETS");
    wait_and_log(pid_obstacles, "OBSTACLES");
    wait_and_log(pid_watchdog, "WATCHDOG");

    endwin();

    //clanup SHM ----------------------------------------------------------------------------------------------------------------
    sem_destroy(&hb->mutex); //destroy the semaphore    
    munmap(hb, sizeof(*hb));
    close(hb_fd);
    shm_unlink(HB_SHM_NAME);

    log_message("BLACKBOARD", "Blackboard shutdown");

    return 0;


//CLEANUP 
cleanup:
    log_message("BLACKBOARD", "Entering cleanup phase");

     //network cleanup
    if (cfg.network_enabled) {
        log_message("BLACKBOARD", "Stopping network threads...");
        
        pthread_mutex_lock(&g_net_mutex);
        g_net_state.server_running = 0;
        pthread_mutex_unlock(&g_net_mutex);
        
        //wait thread (with timeout)
        if (have_server) pthread_join(server_thread, NULL);
        if (have_client) pthread_join(client_thread, NULL);
        
        network_state_cleanup(&g_net_state);
        log_message("BLACKBOARD", "Network cleanup completed");
    }
    
    nanosleep(&ts, NULL); //delay for killing all processes (200ms)
    
    //check if processes killes
    if (pid_input > 0) kill(pid_input, SIGKILL);
    if (pid_drone > 0) kill(pid_drone, SIGKILL);
    if (pid_targets > 0) kill(pid_targets, SIGKILL);
    if (pid_obstacles > 0) kill(pid_obstacles, SIGKILL);
    if (pid_watchdog > 0) kill(pid_watchdog, SIGKILL);
    
    //avoid zombie child
    wait_pid(pid_input, "INPUT");
    wait_pid(pid_drone, "DRONE");
    wait_pid(pid_targets, "TARGETS");
    wait_pid(pid_obstacles, "OBSTACLES");
    wait_pid(pid_watchdog, "WATCHDOG");

    //close all pipes
    close(pipe_input[0]);
    close(pipe_input[1]);
    close(pipe_drone[0]);
    close(pipe_drone[1]);
    close(pipe_targets[0]);
    close(pipe_targets[1]);
    close(pipe_obstacles[0]);
    close(pipe_obstacles[1]);
    
    //cleanup shm
    if (hb != MAP_FAILED && hb != NULL) { //if cleaunp before mmap
        sem_destroy(&hb->mutex);
        munmap(hb, sizeof(*hb));
    }
    if (hb_fd >= 0) { //if cleaunp before hb
        close(hb_fd);
    }
    shm_unlink(HB_SHM_NAME);

    log_message("BLACKBOARD", "Blackboard shutdown");
    
    if (g_stop && pid_input < 0) {
        log_message("BLACKBOARD", "Shutdown due to fork failure");
        return EXIT_FAILURE;
    }

    log_message("BLACKBOARD", "Blackboard terminated normally");
    return EXIT_SUCCESS;
}