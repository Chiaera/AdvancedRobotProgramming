/* this file contains the function for the server process
    - manage the socket
    - initial handshake
    - send window dimensions
    - send drone position and receive obstacle (client drone) position
*/

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>

#include "network.h"
#include "logger.h"


int network_server_init(NetworkContext *ctx){
    //setup socket
    int newsockfd;
    struct sockaddr_in serv_addr;
    socklen_t clilen;
    struct sockaddr_in cli_addr;

    //create socket
    ctx->sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (ctx->sockfd < 0) {
        perror("ERROR opening socket");
        log_message("NETWORK", "ERROR: socket creation failed");
        exit(EXIT_FAILURE);
    }
     
    //bind to correct port
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(ctx->port);
    if (bind(ctx->sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        perror("ERROR on binding");
        log_message("NETWORK", "ERROR: binding failed");
        exit(EXIT_FAILURE);
    }

    //listen and wait for client
    listen(ctx->sockfd,5);
    clilen = sizeof(cli_addr);

    //accept connection
    newsockfd = accept(ctx->sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if (newsockfd < 0) {
        perror("ERROR accepting connection");
        log_message("NETWORK", "ERROR: connection to client failed");
        exit(EXIT_FAILURE);
    }
    
    ctx->connfd = newsockfd; //store connected socket
    return 0;
}

//-------------------------------------------------------------------------COMUNICATION

//initial handshake (ok -> ook)
int handshake(NetworkContext *ctx){
    char buffer[BUFFER_SIZE];
    send_msg(ctx->connfd, "ok"); //send handshake message

    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) { //receive ack
        perror("ERROR receiving handshake");
        log_message("NETWORK", "ERROR: handshake receive failed");
        return -1;
    }
    if (strcmp(buffer, "ook") != 0) { //check ack 'ook'
        log_message("NETWORK", "ERROR: unexpected handshake ack message: %s", buffer);
        return -1;
    }

    log_message("NETWORK", "Handshake successful with client");
    return 0;
}

//send window dimensions (size l h -> sok)
int send_window_size(NetworkContext *ctx, int width, int height){
    char buffer[BUFFER_SIZE];

    snprintf(buffer, BUFFER_SIZE, "size %d %d", width, height); //write message

    if (send_msg(ctx->connfd, buffer) < 0) { //send message
        perror("ERROR sending window size");
        log_message("NETWORK", "ERROR: window size send failed");
        return -1;
    }

    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) { //receive ack
        perror("ERROR receiving window size ack");
        log_message("NETWORK", "ERROR: window size ack receive failed");
        return -1;
    }
    if (strncmp(buffer, "sok", 3) != 0) { //check 'sok' ack
        log_message("NETWORK", "ERROR: invalid window");
        return -1;
    }

    log_message("NETWORK", "Window size sent successfully to client");
    return 0;
}

//send drone position (drone -> x, y -> dok <drone>)
int send_drone_position(NetworkContext *ctx, int x, int y){
    char buffer[BUFFER_SIZE];

    snprintf(buffer, BUFFER_SIZE, "drone %d %d", x, y); //write position

    if (send_msg(ctx->connfd, buffer) < 0) { //send message
        perror("ERROR sending drone position");
        log_message("NETWORK", "ERROR: drone position send failed");
        return -1;
    }

    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) { //receive ack
        perror("ERROR receiving drone position ack");
        log_message("NETWORK", "ERROR: drone position ack receive failed");
        return -1;
    }
    if (strncmp(buffer, "dok", 3) != 0) { //check 'dok' ack
        log_message("NETWORK", "ERROR: invalid drone position ack");
        return -1;
    }

    log_message("NETWORK", "Drone position sent successfully to client");
    return 0;
}

//receive obstacle position (obst -> x, y -> pok <obstacle>)
int receive_obstacle_position(NetworkContext *ctx, int *x, int *y){
    char buffer[BUFFER_SIZE];

    strcpy(buffer, "obst"); //write message

    if (send_msg(ctx->connfd, buffer) < 0) { //send message
        perror("ERROR asking obstacle position");
        log_message("NETWORK", "ERROR: obstacle position ask failed");
        return -1;
    }

    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) { //receive ack
        perror("ERROR receiving obstacle position");
        log_message("NETWORK", "ERROR: obstacle position receive failed");
        return -1;
    }
    if (sscanf(buffer, "%d %d", x, y) != 2) {
        log_message("NETWORK", "ERROR: failed to parse obstacle position");
        return -1;
    } else {
        if (send_ack(ctx->connfd, "pok") < 0) { //send ack
            perror("ERROR sending obstacle position ack");
            log_message("NETWORK", "ERROR: obstacle position ack send failed");
            return -1;
        }
    }

    log_message("NETWORK", "Obstacle position received successfully");
    return 0;
}
