/* this file contains the function for the client process
    - manage connesion to the server
    - receive window dimensions
    - receive drone (server) position and send its drone position as an obstacle

    loop:
        rcv ok
        snd ook

        rcv size l h
        snd sok

        loop:
            rcv x
            if x == q:
                snd qok
                exit

            switch x:
                case drone:
                    rcv x y
                    snd dok

                case obst:
                    snd x y
                    rcv pok
*/

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <string.h>

#include "network.h"
#include "logger.h"


//manage socket
int network_client_init(NetworkContext *ctx){
    struct sockaddr_in serv_addr;

    //socket creation
    ctx->sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (ctx->sockfd < 0) {
        log_message("NETWORK", "[CLIENT] ERROR: cannot create socket");
        return -1;
    }
    log_message("NETWORK", "[CLIENT] Socket created");

    //server address setup
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(ctx->port);

    if (inet_pton(AF_INET, ctx->server_ip, &serv_addr.sin_addr) <= 0) {
        log_message("NETWORK", "[CLIENT] ERROR: invalid IP address %s", ctx->server_ip);
        return -1;
    }

    //connect to server
    if (connect(ctx->sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        log_message("NETWORK", "[CLIENT] ERROR: connection failed");
        return -1;
    }

    log_message("NETWORK", "[CLIENT] Connected to server");

    ctx->connfd = ctx->sockfd;
    return 0;
}

//-------------------------------------------------------------------------COMUNICATION

//handshake
int client_handshake(NetworkContext *ctx){
    char buffer[BUFFER_SIZE];

    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) { //receive handshake
        log_message("NETWORK", "[CLIENT] ERROR: failed to receive handshake");
        return -1;
    }
    if (strcmp(buffer, "ok") != 0) { //check 'ok'
        log_message("NETWORK", "[CLIENT] ERROR: invalid handshake ack");
        return -1;
    }
    log_message("NETWORK", "[CLIENT] Received handshake: %s", buffer);

    if (send_msg(ctx->connfd, "ook") < 0) { //send 'ook'
        log_message("NETWORK", "[CLIENT] ERROR: failed to send handshake ack");
        return -1;
    }

    log_message("NETWORK", "[CLIENT] Handshake completed");
    return 0;
}


//receive window dimensions (size W H -> sok)
int receive_window_size(NetworkContext *ctx, int *W, int *H) {
    char buffer[BUFFER_SIZE];

    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) { //receive H and W
        log_message("NETWORK", "[CLIENT] ERROR: failed to receive window size");
        return -1;
    }

    int w, h;
    if (sscanf(buffer, "size %d %d", &w, &h) != 2) { //check message format
        log_message("NETWORK", "[CLIENT] ERROR: invalid size message '%s'", buffer);
        return -1;
    }

    *W = w;
    *H = h;
    log_message("NETWORK", "[CLIENT] Received window size %d x %d", w, h);

    //send ack 'sok'
    if (send_msg(ctx->connfd, "sok") < 0) {
        log_message("NETWORK", "[CLIENT] ERROR: failed to send size ack");
        return -1;
    }

    return 0;
}


//receive drone position
int receive_drone_position(NetworkContext *ctx, int *x, int *y){
    char buffer[BUFFER_SIZE];

    //receive message 'x y'
    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) {
        log_message("NETWORK", "[CLIENT] ERROR: failed to receive drone position");
        return -1;
    }

    //parse x and y
    if (sscanf(buffer, "%d %d", x, y) != 2){
        log_message("NETWORK", "[CLIENT] ERROR: invalid drone position coordinates");
        return -1;
    }

    //send ack 'dok'
    if (send_msg(ctx->connfd, "dok") < 0) {
        log_message("NETWORK", "[CLIENT] ERROR: failed to send drone position ack");
        return -1;
    }

    log_message("NETWORK", "[CLIENT] Received drone position: (%d, %d)", *x, *y);
    return 0;
}


//send obstacle position
int send_obstacle_position(NetworkContext *ctx, int x, int y){
    char buffer[BUFFER_SIZE];

    //send message obstacle position 'x y' 
    snprintf(buffer, BUFFER_SIZE, "%d %d", x, y);
    if (send_msg(ctx->connfd, buffer) < 0) {
        log_message("NETWORK", "[CLIENT] ERROR: failed to send obstacle position");
        return -1;
    }
    //receive ack 'pok'
    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) {
        log_message("NETWORK", "[CLIENT] ERROR: failed to receive obstacle position ack");
        return -1;
    }
    if (strcmp(buffer, "pok") != 0) { //check 'pok'
        log_message("NETWORK", "[CLIENT] ERROR: invalid obstacle position ack");
        return -1;
    }

    log_message("NETWORK", "[CLIENT] Obstacle position sent successfully");
    return 0;
}



//send quit
int receive_quit(NetworkContext *ctx) {
    char buffer[BUFFER_SIZE];

    //send ack 'qok'
    if (send_msg(ctx->connfd, "qok") < 0) {
        log_message("NETWORK", "[CLIENT] ERROR: failed to send quit ack");
        return -1;
    }

    log_message("NETWORK", "[CLIENT] Received quit message");
    return 0;
}