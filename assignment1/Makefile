
CC = gcc
CFLAGS = -Wall -Wextra -g


NCURSES = -lncurses
SRC = src
BUILD = build
PROGRAMS = ncurses_test server_test pipes_test watchdog_test

all: $(PROGRAMS)
%: $(SRC)/%.c
	$(CC) $(CFLAGS) $< -o $(BUILD)/$@ $(NCURSES)

clean:
	rm -f $(BUILD)/*
