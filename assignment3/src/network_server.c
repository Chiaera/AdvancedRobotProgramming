/* this file contains the function for the server process
    - manage the socket
    - initial handshake
    - send window dimensions
    - send drone position and receive obstacle (client drone) position

    loop:
        snd ok
        rcv ook

        snd size l h
        rcv sok

        loop:
            if quit:
                snd q
                rcv qok
                exit

            snd drone
            snd x y
            rcv dok

            snd obst
            rcv x y
            snd pok
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
int network_server_init(NetworkContext *ctx){
    //setup socket
    int newsockfd;
    struct sockaddr_in serv_addr;
    socklen_t clilen;
    struct sockaddr_in cli_addr;

    //create socket
    ctx->sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (ctx->sockfd < 0) {
        perror("[SERVER] ERROR opening socket");
        log_message("NETWORK", "E[SERVER] ERROR: socket creation failed");
        exit(EXIT_FAILURE);
    }
    log_message("NETWORK", "[SERVER] Socket created");
     
    //bind to correct port
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(ctx->port);
    if (bind(ctx->sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        perror("[SERVER] ERROR on binding");
        log_message("NETWORK", "[SERVER] ERROR: binding failed");
        exit(EXIT_FAILURE);
    }
    log_message("NETWORK", "[SERVER] Bind OK on port %d", ctx->port);

    //listen and wait for client
    listen(ctx->sockfd,5);
    log_message("NETWORK", "[SERVER] Listening for client...");
    clilen = sizeof(cli_addr);

    //accept connection
    newsockfd = accept(ctx->sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if (newsockfd < 0) {
        perror("[SERVER] ERROR accepting connection");
        log_message("NETWORK", "[SERVER] ERROR: connection to client failed");
        exit(EXIT_FAILURE);
    }
    log_message("NETWORK", "[SERVER] Client connected (fd=%d)", newsockfd);
    
    ctx->connfd = newsockfd; //store connected socket
    return 0;
}

//-------------------------------------------------------------------------COMUNICATION

//initial handshake (ok -> ook)
int  server_handshake(NetworkContext *ctx){
    char buffer[BUFFER_SIZE];
    //send handshake message 'ok'
    send_msg(ctx->connfd, "ok"); 

    //receive ack 'ook'
    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) { //receive ack
        perror("[SERVER] ERROR receiving handshake");
        log_message("NETWORK", "[SERVER] ERROR: handshake receive failed");
        return -1;
    }
    if (strcmp(buffer, "ook") != 0) { //check ack 'ook'
        log_message("NETWORK", "[SERVER] ERROR: invalid handshake ack");
        return -1;
    }

    log_message("NETWORK", "[SERVER] Handshake completed");
    return 0;
}

//send window dimensions (size l h -> sok)
int send_window_size(NetworkContext *ctx, int width, int height){
    char buffer[BUFFER_SIZE];

    //write message
    snprintf(buffer, BUFFER_SIZE, "size %d %d", width, height); 

    //send message 'size l h'
    if (send_msg(ctx->connfd, buffer) < 0) { 
        perror("[SERVER] ERROR sending window size");
        log_message("NETWORK", "[SERVER] ERROR: window size send failed");
        return -1;
    }

     //receive ack 'sok'
    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) { //receive ack
        perror("[SERVER] ERROR receiving window size ack");
        log_message("NETWORK", "[SERVER] ERROR: window size ack receive failed");
        return -1;
    }
    if (strncmp(buffer, "sok", 3) != 0) { //check 'sok' ack
        log_message("NETWORK", "[SERVER] ERROR: invalid window size ack");
        return -1;
    }

    log_message("NETWORK", "[SERVER] Sent window size %d x %d", width, height);
    return 0;
}

//send drone position (drone -> x, y -> dok <drone>)
int send_drone_position(NetworkContext *ctx, int x, int y){
    char buffer[BUFFER_SIZE];

    //type message 'drone'
    if (send_msg(ctx->connfd, "drone") < 0) return -1;

    //write drone message
    snprintf(buffer, BUFFER_SIZE, "%d %d", x, y); 
    if (send_msg(ctx->connfd, buffer) < 0) { 
        perror("[SERVER] ERROR sending drone position");
        log_message("NETWORK", "[SERVER] ERROR: drone position send failed");
        return -1;
    }

    //receive ack 'dok'
    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) { //receive ack
        perror("[SERVER] ERROR receiving drone position ack");
        log_message("NETWORK", "[SERVER] ERROR: drone position ack receive failed");
        return -1;
    }
    if (strncmp(buffer, "dok", 3) != 0) { //check 'dok' ack
        log_message("NETWORK", "[SERVER] ERROR: invalid drone position ack");
        return -1;
    }

    log_message("NETWORK", "[SERVER] Sending drone position: %d %d", x, y);
    return 0;
}

//receive obstacle position (obst -> x, y -> pok <obstacle>)
int receive_obstacle_position(NetworkContext *ctx, int *x, int *y){
    char buffer[BUFFER_SIZE];

    //type message 'obst'
    if (send_msg(ctx->connfd, "obst") < 0) return -1;

    //receive message
    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) { 
        perror("[SERVER] ERROR receiving obstacle position");
        log_message("NETWORK", "[SERVER] ERROR: obstacle position receive failed");
        return -1;
    }

    //parse x and y
    if (sscanf(buffer, "%d %d", x, y) != 2) {
        log_message("NETWORK", "[SERVER] ERROR: failed to parse obstacle position");
        return -1;
    } else {
        if (send_ack(ctx->connfd, "pok") < 0) { //send ack 'pok'
            perror("[SERVER] ERROR sending obstacle position ack");
            log_message("NETWORK", "[SERVER] ERROR: obstacle position ack send failed");
            return -1;
        }
    }

    log_message("NETWORK", "[SERVER] Received obstacle: %d %d", *x, *y);
    return 0;
}

//send quit message (q -> qok)
int send_quit(NetworkContext *ctx){
    char buffer[BUFFER_SIZE];

    //write quit message
    snprintf(buffer, BUFFER_SIZE, "q"); 
    if (send_msg(ctx->connfd, buffer) < 0) { 
        perror("[SERVER] ERROR sending quit message");
        log_message("NETWORK", "[SERVER] ERROR: quit message send failed");
        return -1;
    }

    //receive ack 'qok'
    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) { //receive ack
        perror("[SERVER] ERROR receiving quit message ack");
        log_message("NETWORK", "[SERVER] ERROR: quit message ack receive failed");
        return -1;
    }
    if (strncmp(buffer, "qok", 3) != 0) { //check 'qok' ack
        log_message("NETWORK", "[SERVER] ERROR: invalid quit message ack");
        return -1;
    }

    log_message("NETWORK", "[SERVER] Quit message sent successfully");
    return 0;
}
