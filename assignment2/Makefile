# path
SRC_DIR := src
INC_DIR := include
BUILD_DIR := build
BIN_DIR := $(BUILD_DIR)/bin
# compiler
CC := gcc
CFLAGS := -Wall -Wextra -g -I$(INC_DIR)
LDFLAGS_NCURSES := -lncurses
LDFLAGS_MATH := -lm
# include
BLACKBOARD := $(BIN_DIR)/blackboard
INPUT_PROCESS := $(BIN_DIR)/process_input
DRONE_PROCESS := $(BIN_DIR)/process_drone
OBSTACLES_PROCESS := $(BIN_DIR)/process_obstacles
TARGET_PROCESS := $(BIN_DIR)/process_targets
# src
BLACKBOARD_SRC := $(SRC_DIR)/blackboard.c \
                  $(SRC_DIR)/map.c \
                  $(SRC_DIR)/world.c \
                  $(SRC_DIR)/world_physics.c
INPUT_SRC := $(SRC_DIR)/process_input.c
DRONE_SRC := $(SRC_DIR)/process_drone.c
OBSTACLES_SRC := $(SRC_DIR)/process_obstacles.c
TARGET_SRC := $(SRC_DIR)/process_targets.c

# default
all: $(BLACKBOARD) $(INPUT_PROCESS) $(DRONE_PROCESS) $(OBSTACLES_PROCESS) $(TARGET_PROCESS)

# ensure bin dir exists
$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# blackboard
$(BLACKBOARD): $(BLACKBOARD_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(BLACKBOARD_SRC) -o $@ $(LDFLAGS_NCURSES) $(LDFLAGS_MATH)
# input
$(INPUT_PROCESS): $(INPUT_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(INPUT_SRC) -o $@ $(LDFLAGS_NCURSES)
# drone
$(DRONE_PROCESS): $(DRONE_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(DRONE_SRC) -o $@
# targets
$(TARGET_PROCESS): $(TARGET_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(TARGET_SRC) -o $@
# obstacles
$(OBSTACLES_PROCESS): $(OBSTACLES_SRC) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(OBSTACLES_SRC) -o $@

# run everything
run: all
	konsole -e ./build/bin/blackboard &

clean:
	rm -rf $(BUILD_DIR)


.PHONY: all clean run
