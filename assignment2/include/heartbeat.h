/* heartbeat.h
    shared memory object (shm) heartbeat table used by the Watchdog
    verified if the process is alive and active (not blocked / deadlocked)

    - POSIX shm -> map for each process
    - each process updates its slot with:
      - PID
      - monotonic timestamp (last_seen_ms) - monotonic for a more robust and deterministic timeout
    - watchdog checks the timestamps (IF not exist an answar: process is stuck)
*/

#pragma once

#include <stdint.h>     
#include <sys/types.h>  
#include <time.h>       

// POSIX shared memory name - used by all processes
#define HB_SHM_NAME "/heartbeat"

//slot for each process (each slot is private to the considered process)
enum {
  HB_SLOT_BLACKBOARD = 0,
  HB_SLOT_INPUT      = 1,
  HB_SLOT_DRONE      = 2,
  HB_SLOT_TARGETS    = 3,
  HB_SLOT_OBSTACLES  = 4,
  HB_SLOTS           = 5
};

//heartbeat struct
typedef struct {
  uint64_t last_seen_ms; //last_seen to confirm the process is running
  pid_t    pid; //pid to  identifier the process
} HbEntry;

//heartbeat table stuct - one entry(slot) for each process
typedef struct {
  HbEntry entries[HB_SLOTS];
} HeartbeatTable;


//monotonic clock - current time in milliseconds (all processes refresh their slot in the heartbeat table)
static inline uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}
