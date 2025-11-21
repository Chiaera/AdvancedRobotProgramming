CC      = gcc
CFLAGS  = -Wall -Wextra -g -Iinclude
NCURSES = -lncurses

SRC_DIR = src
OBJ_DIR = build/obj
BIN_DIR = build

SRCS  = $(wildcard $(SRC_DIR)/*.c)
OBJS  = $(SRCS:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

TARGET = $(BIN_DIR)/server

all: $(TARGET)

# build
$(BIN_DIR) $(OBJ_DIR):
	mkdir -p $@
$(TARGET): $(OBJS) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(NCURSES)

# compute .c
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf build

.PHONY: all clean
