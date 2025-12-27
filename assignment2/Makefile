#path
SRC_DIR := src
INC_DIR := include
BUILD_DIR := build
BIN_DIR := $(BUILD_DIR)/bin

#compiler
CC := gcc
CFLAGS := -Wall -Wextra -g -I$(INC_DIR)

#debug (0=no debug, 1=debug)
DEBUG ?= 0
ifeq ($(DEBUG),1)
CFLAGS += -DDEBUG
endif

LDFLAGS_NCURSES := -lncurses
LDFLAGS_MATH := -lm

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


#default rule
all: | $(BIN_DIR) $(LOG_DIR)
all: $(BLACKBOARD) $(INPUT_PROCESS) $(DRONE_PROCESS) $(OBSTACLES_PROCESS) $(TARGET_PROCESS) $(WATCHDOG_PROCESS)

#ensure dirs exists
$(BIN_DIR):
	mkdir -p $(BIN_DIR)
$(LOG_DIR):
	mkdir -p $(LOG_DIR)

#blackboard
$(BLACKBOARD): $(BLACKBOARD_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(BLACKBOARD_SRC) -o $@ $(LDFLAGS_NCURSES) $(LDFLAGS_MATH)
#input
$(INPUT_PROCESS): $(INPUT_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(INPUT_SRC) -o $@ $(LDFLAGS_NCURSES)
#drone
$(DRONE_PROCESS): $(DRONE_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(DRONE_SRC) -o $@
#targets
$(TARGET_PROCESS): $(TARGET_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(TARGET_SRC) -o $@
#obstacles
$(OBSTACLES_PROCESS): $(OBSTACLES_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(OBSTACLES_SRC) -o $@
#watchdog
$(WATCHDOG_PROCESS): $(WATCHDOG_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(WATCHDOG_SRC) -o $@

#run everything
run: all | $(LOG_DIR)
	touch $(SYSTEM_LOG) $(WATCHDOG_LOG)
	konsole -e ./build/bin/blackboard &

#open live logs in new terminals
tail-logs: | $(LOG_DIR)
	konsole -e bash -lc "tail -f $(SYSTEM_LOG)" &
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

.PHONY: all clean run kill clean-logs tail-logs
