/* this file contains the function for the watchdog process
    - monitored all processes by the shared memory heartbeat table
    
    - Every CHECK_INTERVAL_MS:
        read last_seen_ms for each slot
        compare with now_ms()

    - IF not receive heartbeat updates for longer than TIMEOUT_MS:
        1. notifies the blackboard (SIGUSR1) then it call endwin()
        2. kills all processes in the table (SIGKILL)
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

#include "heartbeat.h"
#include "logger.h"

#define CHECK_INTERVAL_MS 20

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

    // Open existing shared memory created by blackboard
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

        for (int i = 0; i < HB_SLOTS; i++) { //create the heartbeat table
            pid_t p = hb->entries[i].pid;
            uint64_t last = hb->entries[i].last_seen_ms;

            //if the PID is not register continue -> possibility: process not started yet
            if (p <= 0) continue;
            // process registered but has not heartbeated yet -> wait
            if (last == 0) continue;

            //to check the time before the last heartbeat
            if ((now - last) > timeout_ms) {
                detected_slot = i;
                detected_pid = p;
                log_message("WATCHDOG", "TIMEOUT: slot=%d pid=%d (last seen %llums ago)",
                            detected_slot, (int)detected_pid, (unsigned long long)(now - last));
                LOGF(LOG_PATH "watchdog.log", "watchdog msg: TIMEOUT slot=%d pid=%d last=%llums ago\n",
                    i, (int)p,
                    (unsigned long long)(now - last)); //to print in the watchdog.log (to learn more about the watchdog activity)
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
            log_message("WATCHDOG", "TIMEOUT detected on slot %d (PID %d) sending SIGUSR1 to blackboard", 
                detected_slot, (int)detected_pid);
            LOGF(LOG_PATH "watchdog.log", "PID %d sending SIGUSR1 to blackboard\n", (int)bb); //to learn more about the watchdog (check if bb_pid == detected_pid)
            kill(bb, SIGUSR1);
        }

        //delay for the edwin() - used for the 'nanosleep' function
        struct timespec ts_edwin;
        ts_edwin.tv_sec = 0;
        ts_edwin.tv_nsec = 200 * 1000 * 1000; // 200 ms
        nanosleep(&ts_edwin, NULL);

        //kill all registered processe
        log_message("WATCHDOG", "Killing all processes");        
        kill_all(hb);

        munmap(hb, sizeof(*hb));
        close(hb_fd);

        log_message("WATCHDOG", "Watchdog shutdown");

        return 2;
    }
}
