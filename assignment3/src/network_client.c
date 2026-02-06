/* this file contains the function for the client process
    - manage connesion to the server
    - receive window dimensions
    - receive drone (server) position and send its drone position as an obstacle
*/

/*
//initial handshake (ok -> ook)
int handshake(NetworkContext *ctx){
    char buffer[BUFFER_SIZE];
    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) {
        perror("ERROR receiving handshake");
        log_message("NETWORK", "ERROR: handshake receive failed");
        return -1;
    }
    if (strcmp(buffer, "ok") != 0) {
        log_message("NETWORK", "ERROR: unexpected handshake message: %s", buffer);
        return -1;
    }
    if (send_ack(ctx->connfd, "ook") < 0) {
        perror("ERROR sending handshake ack");
        log_message("NETWORK", "ERROR: handshake ack send failed");
        return -1;
    }
    log_message("NETWORK", "Handshake successful with client");
    return 0;
}

int network_quit(NetworkContext *ctx){
    char buffer[BUFFER_SIZE];

    snprintf(buffer, BUFFER_SIZE, "quit"); //write message

    if (send_msg(ctx->connfd, buffer) < 0) { //send message
        perror("ERROR sending quit message");
        log_message("NETWORK", "ERROR: quit message send failed");
        return -1;
    }

    if (recv_msg(ctx->connfd, buffer, BUFFER_SIZE) < 0) { //receive ack
        perror("ERROR receiving quit ack");
        log_message("NETWORK", "ERROR: quit ack receive failed");
        return -1;
    }
    if (strncmp(buffer, "qok", 3) != 0) { //check ack
        log_message("NETWORK", "ERROR: invalid quit ack");
        return -1;
    }

    log_message("NETWORK", "Quit message sent successfully");
    return 0;
} */