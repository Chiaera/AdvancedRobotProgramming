#path
SRC_DIR := src
INC_DIR := include
BUILD_DIR := build
BIN_DIR := $(BUILD_DIR)/bin

#compiler
CC := gcc
CFLAGS := -Wall -Wextra -g -I$(INC_DIR) -DDEBUG

LDFLAGS_NCURSES := -lncurses
LDFLAGS_MATH := -lm
LDFLAGS_PTHREAD := -lpthread 

#include (bin)
BLACKBOARD := $(BIN_DIR)/blackboard
INPUT_PROCESS := $(BIN_DIR)/process_input
DRONE_PROCESS := $(BIN_DIR)/process_drone
OBSTACLES_PROCESS := $(BIN_DIR)/process_obstacles
TARGET_PROCESS := $(BIN_DIR)/process_targets
WATCHDOG_PROCESS := $(BIN_DIR)/watchdog

#logs
LOG_DIR := logs
SYSTEM_LOG := $(LOG_DIR)/system.log 
WATCHDOG_LOG := $(LOG_DIR)/watchdog.log
PROCESS_PID := $(LOG_DIR)/processes.pid 

#src
BLACKBOARD_SRC := $(SRC_DIR)/blackboard.c \
                  $(SRC_DIR)/map.c \
                  $(SRC_DIR)/world.c \
                  $(SRC_DIR)/drone_physics.c
INPUT_SRC := $(SRC_DIR)/process_input.c
DRONE_SRC := $(SRC_DIR)/process_drone.c
OBSTACLES_SRC := $(SRC_DIR)/process_obstacles.c
TARGET_SRC := $(SRC_DIR)/process_targets.c
WATCHDOG_SRC := $(SRC_DIR)/watchdog.c
NETWORK_SRC := $(SRC_DIR)/network.c


#default rule
all: | $(BIN_DIR) $(LOG_DIR)
all: $(BLACKBOARD) $(INPUT_PROCESS) $(DRONE_PROCESS) $(OBSTACLES_PROCESS) $(TARGET_PROCESS) $(WATCHDOG_PROCESS)

#ensure dirs exists
$(BIN_DIR):
	mkdir -p $(BIN_DIR)
$(LOG_DIR):
	mkdir -p $(LOG_DIR)
	touch $(SYSTEM_LOG) $(WATCHDOG_LOG)

#blackboard
$(BLACKBOARD): $(BLACKBOARD_SRC) $(NETWORK_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(BLACKBOARD_SRC) $(NETWORK_SRC) -o $@ $(LDFLAGS_NCURSES) $(LDFLAGS_MATH) $(LDFLAGS_PTHREAD)
#input
$(INPUT_PROCESS): $(INPUT_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(INPUT_SRC) -o $@ $(LDFLAGS_NCURSES) $(LDFLAGS_PTHREAD)
#drone
$(DRONE_PROCESS): $(DRONE_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(DRONE_SRC) -o $@ $(LDFLAGS_PTHREAD)
#targets
$(TARGET_PROCESS): $(TARGET_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(TARGET_SRC) -o $@ $(LDFLAGS_PTHREAD)
#obstacles
$(OBSTACLES_PROCESS): $(OBSTACLES_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(OBSTACLES_SRC) -o $@ $(LDFLAGS_PTHREAD)
#watchdog
$(WATCHDOG_PROCESS): $(WATCHDOG_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(WATCHDOG_SRC) -o $@ $(LDFLAGS_PTHREAD)

#help function
help:
	./build/bin/blackboard --help

#run everything (append to existing logs)
run: all | $(LOG_DIR)
	konsole -e ./build/bin/blackboard &

#run with clean logs
run-clean: all | $(LOG_DIR)
	@echo "Cleaning old logs"
	@rm -f $(SYSTEM_LOG) $(WATCHDOG_LOG) $(PROCESS_PID)
	@echo "Starting blackboard"
	konsole -e ./build/bin/blackboard &

#if there are problems with konsole, run in the same terminal
run-generic: all | $(LOG_DIR)
	@echo "Starting blackboard (use Ctrl+C to stop)"
	@./build/bin/blackboard || true

#shortcut
r: run-clean

#open live logs in new terminals
tail-logs: | $(LOG_DIR)
	konsole -e bash -lc "tail -n 20 -f $(SYSTEM_LOG)" &
	konsole -e bash -lc "tail -f $(WATCHDOG_LOG)" &

#kill all processes
kill:
	-pkill -9 blackboard process_input process_drone process_targets process_obstacles watchdog || true

#clean dirs
clean:
	rm -rf $(BUILD_DIR) $(LOG_DIR)
clean-logs:
	rm -rf $(LOG_DIR)
clean-build:
	rm -rf $(BUILD_DIR)

.PHONY: all clean run kill clean-logs tail-logs run-clean r
