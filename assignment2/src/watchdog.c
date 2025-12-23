/* this file contains the function for the watchdog process
    - monitored all processes by the shared memory heartbeat table
    - IF not receive heartbeat updates -> detects deadlock
    - timeout -> clean shoutdown of Blackboard and kills all processes
    - Every CHECK_INTERVAL_MS:
        read last_seen_ms for each slot
        compare with now_ms()
        if any delta > timeout => emergency shutdown
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "heartbeat.h"

#define CHECK_INTERVAL_MS 100

static void msleep(int ms) {
    usleep(ms * 1000);
}

int main(int argc, char **argv) {
    /* Expected args example:
     * watchdog <timeout_ms> <pid_blackboard> <pid_input> <pid_drone> <pid_targets> <pid_obstacles>
     */
    if (argc != 7) {
        fprintf(stderr, "Usage: watchdog <timeout_ms> <bb> <input> <drone> <targets> <obstacles>\n");
        return 1;
    }

    int timeout_ms = atoi(argv[1]);
    pid_t pid_bb = (pid_t)atoi(argv[2]);
    pid_t pid_in = (pid_t)atoi(argv[3]);
    pid_t pid_dr = (pid_t)atoi(argv[4]);
    pid_t pid_tg = (pid_t)atoi(argv[5]);
    pid_t pid_ob = (pid_t)atoi(argv[6]);

    // Open existing shared memory object created by blackboard
    int hb_fd = shm_open(HB_SHM_NAME, O_RDWR, 0666);
    if (hb_fd < 0) { perror("watchdog shm_open"); return 1; }

    // Map shared memory into watchdog address space
    HeartbeatTable *hb = mmap(NULL, sizeof(HeartbeatTable),
                              PROT_READ | PROT_WRITE, MAP_SHARED, hb_fd, 0);
    if (hb == MAP_FAILED) { perror("watchdog mmap"); return 1; }

    // Main monitoring loop
    while (1) {
        uint64_t now = now_ms();

        // For each monitored slot, compute how long ago the heartbeat was refreshed
        uint64_t dt_bb = now - hb->entries[HB_SLOT_BLACKBOARD].last_seen_ms;
        uint64_t dt_in = now - hb->entries[HB_SLOT_INPUT].last_seen_ms;
        uint64_t dt_dr = now - hb->entries[HB_SLOT_DRONE].last_seen_ms;
        uint64_t dt_tg = now - hb->entries[HB_SLOT_TARGETS].last_seen_ms;
        uint64_t dt_ob = now - hb->entries[HB_SLOT_OBSTACLES].last_seen_ms;

        /* If any process has not updated its heartbeat for too long,
         * we consider it blocked and terminate the simulation.
         */
        if (dt_bb > (uint64_t)timeout_ms ||
            dt_in > (uint64_t)timeout_ms ||
            dt_dr > (uint64_t)timeout_ms ||
            dt_tg > (uint64_t)timeout_ms ||
            dt_ob > (uint64_t)timeout_ms) {

            // 1) Ask blackboard to close ncurses cleanly (endwin) via a signal handler
            kill(pid_bb, SIGUSR1);

            // Small delay to let blackboard restore terminal
            msleep(200);

            // 2) Force kill all processes (as required by specs)
            kill(pid_in, SIGKILL);
            kill(pid_dr, SIGKILL);
            kill(pid_tg, SIGKILL);
            kill(pid_ob, SIGKILL);
            kill(pid_bb, SIGKILL);

            break;
        }

        // Sleep before next check to reduce CPU usage
        msleep(CHECK_INTERVAL_MS);
    }

    // Cleanup mapping (optional, process is terminating)
    munmap(hb, sizeof(*hb));
    close(hb_fd);

    return 0;
}
