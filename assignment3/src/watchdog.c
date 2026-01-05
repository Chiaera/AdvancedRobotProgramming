/* this file contains the function for the watchdog process
    - monitored all processes by the shared memory heartbeat table
    
    - Every CHECK_INTERVAL_MS:
        read last_seen_ms for each slot
        compare with now_ms()

    - IF not receive heartbeat updates for longer than TIMEOUT_MS:
        1. notifies the blackboard (SIGUSR1) then it call endwin()
        2. kills all processes in the table (SIGKILL)
        3. exits
        
    - IF blackboard process terminates (window closed):
        1. logs the event
        2. kills all remaining processes
        3. exits
 */

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>
#include <string.h>

#include "heartbeat.h"
#include "logger.h"

#define CHECK_INTERVAL_MS 10

//macro to print the heartbeat table (debug)
#define LOG_PATH "logs/"
#ifdef DEBUG
static FILE *g_log_file = NULL;

static void log_open(const char *name) {
    if (!g_log_file) {
        g_log_file = fopen(name, "a");
        if (!g_log_file) {
            perror("fopen log failed");
            g_log_file = stderr;   // fallback: no NULL
        }
    }
}

#define LOGF(name, ...) do { \
    log_open(name); \
    fprintf(g_log_file, "[WATCHDOG DBG] " __VA_ARGS__); \
    fflush(g_log_file); \
} while(0)
#else
#define LOGF(name, ...) do {} while(0)
#endif


//SIGKILL to kill all processes after timeout
static void kill_all(const HeartbeatTable *hb) {
    for (int i = 0; i < HB_SLOTS; i++) {
        pid_t p = hb->entries[i].pid;
        if (p > 0 && p != getpid()) {
            kill(p, SIGKILL);
        }
    }
}

//used for the global SIGHUP
static volatile sig_atomic_t g_sighup_received = 0;

static void on_sighup(int sig) {
    (void)sig;
    g_sighup_received = 1;
}


int main(int argc, char **argv) {
    
    if (argc <3) {
        /* args:
            1. argv[1] = shm_name  ('/heartbeat')
            2. argv[2] = timeout_ms ('2000')
        */
        fprintf(stderr, "Usage: %s <shm_name> <timeout_ms>\n", argv[0]);
        return 1;
    }

    //read the argvs
    const char *shm_name = argv[1];
    uint64_t timeout_ms = (uint64_t)strtoull(argv[2], NULL, 10);

    log_message("WATCHDOG", "Watchdog awakes (timeout: %llums)", (unsigned long long)timeout_ms);

    register_process("WATCHDOG"); //register watchdog pid in the pid file

    //save handler to SIGHUP
    struct sigaction sa_hup;
    memset(&sa_hup, 0, sizeof(sa_hup));
    sa_hup.sa_handler = on_sighup;
    sigaction(SIGHUP, &sa_hup, NULL);

    //open shared memory created by blackboard
    int hb_fd = shm_open(shm_name, O_RDWR, 0666);
    if (hb_fd < 0) { 
        log_message("WATCHDOG", "ERROR: shared memory open failed");
        return 1; 
    }

    //map heartbeat table
    HeartbeatTable *hb = mmap(NULL, sizeof(HeartbeatTable), PROT_READ | PROT_WRITE, MAP_SHARED, hb_fd, 0);
    if (hb == MAP_FAILED) { 
        log_message("WATCHDOG", "ERROR: mmap failed");        
        close(hb_fd); 
        return 1; 
    }
    log_message("WATCHDOG", "Heartbeat table mapped successfully");

    //waiting for all process to wake up
    struct timespec grace_ts;
    grace_ts.tv_sec = 0;
    grace_ts.tv_nsec = 300 * 1000 * 1000; // 300 ms
    nanosleep(&grace_ts, NULL);
    log_message("WATCHDOG", "All processes should be awake");

    //use to pass the correct parameters to the timeout
    int detected_slot = -1; 
    pid_t detected_pid = -1;

    #ifdef DEBUG //to print the heartbeat table in the watchdog.log
    static uint64_t last_log_ms = 0;
    #endif

    while (1) { //monitoring loop
        uint64_t now = now_ms();

        #ifdef DEBUG //to print the 'slot set'
            int do_log = (now - last_log_ms >= 1000);
            if (do_log) last_log_ms = now;
        #endif

        //check if the processes still exist
        for (int i = 0; i < HB_SLOTS; i++) {
            sem_wait(&hb->mutex); //lock the heartbeat table
            pid_t p = hb->entries[i].pid;
            sem_post(&hb->mutex); //unlock the heartbeat table

            if (p <= 0) continue; //not registered processes

            //check if the i-th process is still active
            if (kill(p, 0) == -1 && errno == ESRCH) { //process does not exist 
                const char *proc_name = "UNKNOWN";
                if (i == HB_SLOT_BLACKBOARD) proc_name = "BLACKBOARD";
                else if (i == HB_SLOT_INPUT) proc_name = "INPUT";
                else if (i == HB_SLOT_DRONE) proc_name = "DRONE";
                else if (i == HB_SLOT_TARGETS) proc_name = "TARGETS";
                else if (i == HB_SLOT_OBSTACLES) proc_name = "OBSTACLES";
                
                log_message("WATCHDOG", "%s process (slot=%d, PID=%d) no longer exists, killing all processes", 
                            proc_name, i, (int)p);
                LOGF(LOG_PATH "watchdog.log", "%s PID %d not found (ESRCH), killing all\n", proc_name, (int)p);
                
                //notify blackboard before killing all
                pid_t bb = hb->entries[HB_SLOT_BLACKBOARD].pid;
                if (bb > 0 && i != HB_SLOT_BLACKBOARD) {
                    LOGF(LOG_PATH "watchdog.log", "Sending SIGUSR1 to blackboard (PID %d)\n", (int)bb);
                    kill(bb, SIGUSR1);
                    
                    //delay for endwin()
                    struct timespec ts_edwin;
                    ts_edwin.tv_sec = 0;
                    ts_edwin.tv_nsec = 200 * 1000 * 1000; // 200 ms
                    nanosleep(&ts_edwin, NULL);
                }
                
                kill_all(hb);
                munmap(hb, sizeof(*hb));
                close(hb_fd);
                log_message("WATCHDOG", "Watchdog shutdown (reason: process %s terminated)", proc_name);
                return 3; //exit code (against the '2' of timeout)
            }
        }

        //close widow after the SIGHUP is received
        if (g_sighup_received) {
            pid_t bb_pid = hb->entries[HB_SLOT_BLACKBOARD].pid; //check if blackboard still active
            
            //if SIGHUP follow blackboard kill
            if (bb_pid > 0 && kill(bb_pid, 0) == -1 && errno == ESRCH) { //SIGHUP follows the blackboard kill
                log_message("WATCHDOG", "BLACKBOARD process terminated (caused SIGHUP), killing all processes");
                LOGF(LOG_PATH "watchdog.log", "BLACKBOARD terminated, SIGHUP received as consequence\n");
            } else { //close the window with 'x'
                log_message("WATCHDOG", "Received SIGHUP: blackboard window closed, killing all processes");
                LOGF(LOG_PATH "watchdog.log", "SIGHUP received: window closed\n");
            }
            
            kill_all(hb);
            munmap(hb, sizeof(*hb));
            close(hb_fd);
            log_message("WATCHDOG", "Watchdog shutdown (reason: SIGHUP)");
            return 3; 
        }

        for (int i = 0; i < HB_SLOTS; i++) { //create the heartbeat table
            sem_wait(&hb->mutex); //lock the heartbeat table
            pid_t p = hb->entries[i].pid;
            uint64_t last = hb->entries[i].last_seen_ms;
            sem_post(&hb->mutex); //unlock the heartbeat table

            //if the PID is not register continue -> possibility: process not started yet
            if (p <= 0) continue;

            //process registered but has not heartbeated yet -> wait
            if (last == 0) continue;

            //no overflow: last>now
            if (last > now) {
                LOGF(LOG_PATH "watchdog.log", 
                     "WARNING: slot=%d has future timestamp (last=%llu > now=%llu), resetting\n",
                     i, (unsigned long long)last, (unsigned long long)now);
                
                sem_wait(&hb->mutex); //lock the heartbeat table
                hb->entries[i].last_seen_ms = now; // Reset timestamp
                sem_post(&hb->mutex); //unlock the heartbeat table
                
                continue;
            }

            //check the time before the last heartbeat
            uint64_t elapsed = now - last;
            if (elapsed > timeout_ms) {
                detected_slot = i;
                detected_pid = p;

                log_message("WATCHDOG", "TIMEOUT: slot=%d pid=%d (last seen %llums ago)",
                            detected_slot, (int)detected_pid, (unsigned long long)elapsed);
                LOGF(LOG_PATH "watchdog.log", "watchdog msg: TIMEOUT slot=%d pid=%d last=%llums ago\n",
                    i, (int)p, (unsigned long long)elapsed); //print in the watchdog.log (to learn more about the watchdog activity)
                
                goto timeout;
            }

            //DEBUG - print table    
            #ifdef DEBUG
            if (do_log) {
                LOGF(LOG_PATH "watchdog.log",
                    "slot=%d pid=%d last=%llu now=%llu diff=%llu\n",
                    i, (int)p,
                    (unsigned long long)last,
                    (unsigned long long)now,
                    (unsigned long long)(now - last));
            }
            #endif

        }

        //used for the 'nanosleep' function
        struct timespec ts_check;
        ts_check.tv_sec = 0;
        ts_check.tv_nsec = CHECK_INTERVAL_MS * 1000 * 1000; //ms -> ns
        nanosleep(&ts_check, NULL);
        continue;


timeout:
        //cleanup ncurses window (blackboard) before killing the process
        pid_t bb = hb->entries[HB_SLOT_BLACKBOARD].pid;
        if (bb > 0) { //registered processes
            LOGF(LOG_PATH "watchdog.log", "watchdog sending SIGUSR1 to blackboard (PID %d)\n", (int)bb); //to learn more about the watchdog (check if bb_pid == detected_pid)
            kill(bb, SIGUSR1);
        }

        //delay for the edwin() - used for the 'nanosleep' function
        struct timespec ts_edwin;
        ts_edwin.tv_sec = 0;
        ts_edwin.tv_nsec = 200 * 1000 * 1000; // 200 ms
        nanosleep(&ts_edwin, NULL);

        //kill all registered processe  
        kill_all(hb);

        munmap(hb, sizeof(*hb));
        close(hb_fd);

        log_message("WATCHDOG", "Killes all processes, watchdog shutdown (reason: timeout)");

        return 2;
    }
}
