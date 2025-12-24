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

//SIGKILL to use after timeout
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
                fprintf(stderr, "[WATCHDOG] TIMEOUT slot=%d pid=%d last=%llu now=%llu\n", i, (int)p, (unsigned long long)last, (unsigned long long)now);
                goto timeout;
            }
        }

        usleep(100000); //100
        continue;

timeout:
        //cleanup ncurses window (blackboard) before killing the process
        pid_t bb = hb->entries[HB_SLOT_BLACKBOARD].pid;
        if (bb > 0) {
            kill(bb, SIGUSR1);
        }

        //delay for the edwin()
        usleep(200000); // 200ms

        //kill all registered processe
        kill_all(hb);

        munmap(hb, sizeof(*hb));
        close(hb_fd);
        return 2;
    }
}
