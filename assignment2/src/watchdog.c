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

#include "heartbeat.h"

#define LOG_PATH "logs/"

//macro to print the heartbeat table (debug)
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

    LOGF(LOG_PATH "watchdog.log", "watchdog awakes\n");
    LOGF(LOG_PATH "watchdog.log", "Timeout: %llums | Check interval: 20ms\n", 
         (unsigned long long)timeout_ms);

    // Open existing shared memory created by blackboard
    int hb_fd = shm_open(shm_name, O_RDWR, 0666);
    if (hb_fd < 0) { perror("watchdog shm_open"); return 1; }

    //map heartbeat table
    HeartbeatTable *hb = mmap(NULL, sizeof(HeartbeatTable), PROT_READ | PROT_WRITE, MAP_SHARED, hb_fd, 0);
    if (hb == MAP_FAILED) { 
        perror("watchdog mmap"); 
        close(hb_fd); 
        return 1; 
    }
    LOGF(LOG_PATH "watchdog.log", "Heartbeat table mapped successfully\n");

    while (1) { //monitoring loop
        uint64_t now = now_ms();

        for (int i = 0; i < HB_SLOTS; i++) { //create the heartbeat table
            pid_t p = hb->entries[i].pid;
            uint64_t last = hb->entries[i].last_seen_ms;

            //if the PID is not register continue -> possibility: process not started yet
            if (p <= 0) continue;

            // process registered but has not heartbeated yet -> wait
            if (last == 0) {
                continue;
            }

            //to check the time before the last heartbeat
            if (now > last && (now - last) > timeout_ms) {
                LOGF(LOG_PATH "watchdog.log", "watchdog msg: TIMEOUT slot=%d pid=%d last=%llums ago\n",
                    i, (int)p,
                    (unsigned long long)(now - last));
                goto timeout;
            }

            //DEBUG - print table    
            LOGF(LOG_PATH "watchdog.log", "watchdog msg: slot=%d pid=%d last=%llu now=%llu diff=%llu\n",                
                i, (int)p, 
                (unsigned long long)last,
                (unsigned long long)now,
                (unsigned long long)(now-last));
        }

        //used for the 'nanosleep' function
        struct timespec ts_check;
        ts_check.tv_sec = 0;
        ts_check.tv_nsec = 20 * 1000 * 1000;   // 20ms
        nanosleep(&ts_check, NULL);
        continue;


timeout:
        //cleanup ncurses window (blackboard) before killing the process
        pid_t bb = hb->entries[HB_SLOT_BLACKBOARD].pid;
        if (bb > 0) { //registered processes
            LOGF(LOG_PATH "watchdog.log", "PID %d sending SIGUSR1 to blackboard\n", (int)bb);
            kill(bb, SIGUSR1);
        }

        //delay for the edwin() - used for the 'nanosleep' function
        struct timespec ts_edwin;
        ts_edwin.tv_sec = 0;
        ts_edwin.tv_nsec = 200 * 1000 * 1000; // 200 ms
        nanosleep(&ts_edwin, NULL);

        //kill all registered processe
        LOGF(LOG_PATH "watchdog.log", "Killing all registered processes\n");
        kill_all(hb);

        munmap(hb, sizeof(*hb));
        close(hb_fd);
        return 2;
    }
}
