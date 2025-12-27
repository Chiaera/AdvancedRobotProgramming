/* this file contains the logger process 
    - shared logging utility for all processes
    - file lock to prevent write corruption
*/

#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include <time.h>
#include <sys/file.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <sys/stat.h>

#define SYSTEM_LOG_PATH "logs/system.log"
#define PROCESS_FILE_PATH "logs/processes.pid"

//---------------------------------------------------------------------------------------------------------LOG
//write the messages with the lock
static inline void log_message(const char *process_name, const char *format, ...) {
    if (mkdir("logs", 0775) == -1 && errno != EEXIST) { //to be sure the makefile created the correct directory
        perror("Failed to create logs directory");
    }

    //open the log_file
    FILE *log_file = fopen(SYSTEM_LOG_PATH, "a"); 
    if (!log_file) {
        perror("Failed to open system log");
        return;
    }

    //exclusive lock -> just one process at a time can write
    if (flock(fileno(log_file), LOCK_EX) == -1) {
        perror("Failed to lock log file");
        fclose(log_file);
        return;
    }

    //timestamp
    time_t now = time(NULL);
    struct tm tm_info;
    localtime_r(&now, &tm_info);

    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_info); //formatting time

    pid_t pid = getpid(); 

    //message: <timestamp> <process_name> <message> <pid>
    fprintf(log_file, "%s [%s pid=%d]", time_str, process_name, (int)pid);

    //variable message
    va_list args;
    va_start(args, format);
    vfprintf(log_file, format, args);
    va_end(args);

    fprintf(log_file, "\n"); //print variable message

    fflush(log_file); //to 'download' the message without wait to 'complete' the buffer 

    flock(fileno(log_file), LOCK_UN); //unlock file -> another process can write

    fclose(log_file);
}

//---------------------------------------------------------------------------------------------------------PROCESS FILE
//save provess in the filePID
static inline void register_process(const char *process_name) {
    if (mkdir("logs", 0775) == -1 && errno != EEXIST) { //to be sure the makefile created the correct directory
        perror("Failed to create logs directory");
    }

    //open the pid file
    FILE *pf = fopen(PROCESS_FILE_PATH, "a");
    if (!pf) {
        perror("Failed to open process file");
        return;
    }

    //exclusive lock -> just one process at a time can write
    if (flock(fileno(pf), LOCK_EX) == -1) {
        perror("Failed to lock process file");
        fclose(pf);
        return;
    }

    //message: <process_name> <PID>
    fprintf(pf, "%s %d\n", process_name, getpid());
    fflush(pf);

    //unlock
    flock(fileno(pf), LOCK_UN);
    fclose(pf);
}

#endif