/* network.c
    - initialize and cleanup the network status
    - comunication functions (send/receive
    - thread server (protocol server-side)
    - thread client (protocol client-side)
 */

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "network.h"
#include "logger.h"


extern NetworkState g_net_state;
extern pthread_mutex_t g_net_mutex;


//initialization and clean up
int network_state_init(NetworkState* ns, int world_w, int world_h, int max_obstacles) {
    if (!ns) {
        log_message("NETWORK", "ERROR: NULL NetworkState pointer");
        return -1;
    }
    
    memset(ns, 0, sizeof(NetworkState)); //clean the structure
    
    //dynamic array to allocate the external obstacles
    ns->external_obstacles = (ClientObstacle*)malloc(max_obstacles * sizeof(ClientObstacle));
    if (!ns->external_obstacles) {
        log_message("NETWORK", "ERROR: Failed to allocate memory for %d obstacles", max_obstacles);
        return -1;
    }
    
    //all the slot to 'inactive'
    for (int i = 0; i < max_obstacles; i++) {
        ns->external_obstacles[i].x = 0;
        ns->external_obstacles[i].y = 0;
        ns->external_obstacles[i].active = 0;
    }
    
    //default paramethers
    ns->max_external_obstacles = max_obstacles;
    ns->num_external_obstacles = 0;
    ns->world_width = world_w;
    ns->world_height = world_h;
    ns->drone_x = 0.0;
    ns->drone_y = 0.0;
    ns->server_running = 0;
    ns->client_connected = 0;
    
    log_message("NETWORK", "Initialized: world=%dx%d, max_ext_obstacles=%d",
                world_w, world_h, max_obstacles);
    
    return 0;
}

void network_state_cleanup(NetworkState* ns) {
    if (!ns) return;
    
    if (ns->external_obstacles) {
        free(ns->external_obstacles);
        ns->external_obstacles = NULL;
    }
    
    ns->max_external_obstacles = 0;
    ns->num_external_obstacles = 0;
    
    log_message("NETWORK", "Cleanup completed");
}


//comunication
int send_message(int sockfd, const char* msg) {
    if (sockfd < 0 || !msg) {
        log_message("NETWORK", "ERROR: Invalid send parameters");
        return -1;
    }
    
    //newline for the new message
    char buffer[BUFFER_SIZE];
    int len = snprintf(buffer, sizeof(buffer), "%s\n", msg);
    
    if (len < 0 || len >= (int)sizeof(buffer)) {
        log_message("NETWORK", "ERROR: Message too long");
        return -1;
    }
    
    // send messages
    ssize_t sent = send(sockfd, buffer, len, 0);
    if (sent < 0) {
        log_message("NETWORK", "ERROR: send() failed: %s", strerror(errno));
        return -1;
    }
    
    if (sent != len) {
        log_message("NETWORK", "WARNING: Partial send (%zd/%d bytes)", sent, len);
    }
    
    log_message("NETWORK", "SENT: %s", msg); //to debug
    
    return 0;
}

int receive_message(int sockfd, char* buffer, int buffer_size) {
    if (sockfd < 0 || !buffer || buffer_size <= 0) {
        log_message("NETWORK", "ERROR: Invalid receive parameters");
        return -1;
    }
    
    memset(buffer, 0, buffer_size);
    
    //receive external data
    ssize_t received = recv(sockfd, buffer, buffer_size - 1, 0);
    
    if (received < 0) {
        log_message("NETWORK", "ERROR: recv() failed: %s", strerror(errno));
        return -1;
    }
    
    if (received == 0) {
        log_message("NETWORK", "Connection closed by peer");
        return -1;
    }
    
    //remove newline - return
    char* newline = strchr(buffer, '\n');
    if (newline) *newline = '\0';
    
    char* carriage = strchr(buffer, '\r');
    if (carriage) *carriage = '\0';
    
    log_message("NETWORK", "RECV: %s", buffer); //to debug
    
    return 0;
}


//server thread
int network_server_init(int port) { //initialize thread server
    //create socket
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        log_message("NET_SERVER", "socket() failed: %s", strerror(errno));
        return -1;
    }
    
    //possible reuso of address
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        log_message("NET_SERVER", "setsockopt() failed: %s", strerror(errno));
        close(server_fd);
        return -1;
    }
    
    //prepare address
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;  //listening to all the interface
    addr.sin_port = htons(port);
    
    //bind
    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        log_message("NET_SERVER", "bind() failed on port %d: %s", port, strerror(errno));
        close(server_fd);
        return -1;
    }
    
    //listen
    if (listen(server_fd, MAX_CONNECTIONS) < 0) {
        log_message("NET_SERVER", "listen() failed: %s", strerror(errno));
        close(server_fd);
        return -1;
    }
    
    log_message("NET_SERVER", "Server listening on port %d", port);
    return server_fd;
}

//manage a single client (complete protocol)
void handle_client(int client_fd) {
    char buffer[BUFFER_SIZE];
    
    log_message("NET_SERVER", "Client connected, starting protocol");
    
    //STEP1 - handshake  (server "ok" / client "ook")
    if (send_message(client_fd, "ok") < 0) goto cleanup;
    if (receive_message(client_fd, buffer, BUFFER_SIZE) < 0) goto cleanup;
    
    if (strcmp(buffer, "ook") != 0) {
        log_message("NET_SERVER", "Protocol error: expected 'ook', got '%s'", buffer);
        goto cleanup;
    }
    
    //STEP2 - send world dimensions ("size W H" / "sok ")
    pthread_mutex_lock(&g_net_mutex);
    int w = g_net_state.world_width;
    int h = g_net_state.world_height;
    pthread_mutex_unlock(&g_net_mutex);
    
    char size_msg[BUFFER_SIZE];
    snprintf(size_msg, sizeof(size_msg), "size %d %d", w, h);
    if (send_message(client_fd, size_msg) < 0) goto cleanup;
    
    if (receive_message(client_fd, buffer, BUFFER_SIZE) < 0) goto cleanup;
    if (strncmp(buffer, "sok", 3) != 0) {
        log_message("NET_SERVER", "Protocol error: expected 'sok', got '%s'", buffer);
        goto cleanup;
    }
    
    log_message("NET_SERVER", "Handshake complete, entering main loop");
    
    //STEP3 - main loop
    int loop_count = 0;
    while (g_net_state.server_running) {
        
        //send drone info
        if (send_message(client_fd, "drone") < 0) break;
        
        //read drone position from NetworkState
        pthread_mutex_lock(&g_net_mutex);
        int dx = (int)g_net_state.drone_x;
        int dy = (int)g_net_state.drone_y;
        pthread_mutex_unlock(&g_net_mutex);
        
        //send position
        char drone_pos[BUFFER_SIZE];
        snprintf(drone_pos, sizeof(drone_pos), "%d %d", dx, dy);
        if (send_message(client_fd, drone_pos) < 0) break;
        
        //receive "dok 'drone'"
        if (receive_message(client_fd, buffer, BUFFER_SIZE) < 0) break;
        if (strncmp(buffer, "dok", 3) != 0) {
            log_message("NET_SERVER", "Expected 'dok', got '%s'", buffer);
            break;
        }
        
        //ask for the obstacle "obst"
        if (send_message(client_fd, "obst") < 0) break;
        
        //receive obst position
        if (receive_message(client_fd, buffer, BUFFER_SIZE) < 0) break;
        
        int ox, oy;
        if (sscanf(buffer, "%d %d", &ox, &oy) == 2) {
            //save obst external of NetworkState
            pthread_mutex_lock(&g_net_mutex);
            
            //find a free slot
            int slot = -1;
            for (int i = 0; i < g_net_state.max_external_obstacles; i++) {
                if (!g_net_state.external_obstacles[i].active) {
                    slot = i;
                    break;
                }
            }
            
            if (slot == -1) slot = 0; //if all slot full - overwrite the first
            
            g_net_state.external_obstacles[slot].x = ox;
            g_net_state.external_obstacles[slot].y = oy;
            g_net_state.external_obstacles[slot].active = 1;
            
            //update counter
            if (slot >= g_net_state.num_external_obstacles) {
                g_net_state.num_external_obstacles = slot + 1;
            }
            
            pthread_mutex_unlock(&g_net_mutex);
            
            if (loop_count % 10 == 0) {  //log every 10 interaction
                log_message("NET_SERVER", "External obstacle at (%d, %d)", ox, oy);
            }
        }
        
        //check the recive "pok 'obstacle'"
        char pok_msg[BUFFER_SIZE];
        snprintf(pok_msg, sizeof(pok_msg), "pok obstacle_%d", loop_count);
        if (send_message(client_fd, pok_msg) < 0) break;
        
        loop_count++;
        
        //slow the exchange (500ms)
        struct timespec ts = {0, 500 * 1000 * 1000};
        nanosleep(&ts, NULL);
    }
    
    //send quit
    send_message(client_fd, "q");
    receive_message(client_fd, buffer, BUFFER_SIZE);  //wait "qok"
    
    log_message("NET_SERVER", "Protocol completed after %d exchanges", loop_count);
    
cleanup:
    close(client_fd);
}

//thread server
void* network_server_thread(void* arg) {
    int port = *((int*)arg);
    
    log_message("NET_SERVER", "Server thread started on port %d", port);
    
    int server_fd = network_server_init(port);
    if (server_fd < 0) {
        log_message("NET_SERVER", "Failed to initialize server");
        return NULL;
    }
    
    g_net_state.server_running = 1;
    
    //connections loop 
    while (g_net_state.server_running) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        log_message("NET_SERVER", "Waiting for client connection...");
        
        int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) {
            if (g_net_state.server_running) {
                log_message("NET_SERVER", "accept() failed: %s", strerror(errno));
            }
            continue;
        }
        
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
        log_message("NET_SERVER", "Client connected from %s:%d", 
                    client_ip, ntohs(client_addr.sin_port));
        
        //managment the client (one at a time)
        handle_client(client_fd);
    }
    
    close(server_fd);
    log_message("NET_SERVER", "Server thread terminated");
    return NULL;
}


//thread client
//connection to remote server
int network_client_init(const char* host, int port) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        log_message("NET_CLIENT", "socket() failed: %s", strerror(errno));
        return -1;
    }
    
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    
    if (inet_pton(AF_INET, host, &server_addr.sin_addr) <= 0) {
        log_message("NET_CLIENT", "Invalid address: %s", host);
        close(sockfd);
        return -1;
    }
    
    log_message("NET_CLIENT", "Connecting to %s:%d...", host, port);
    
    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        log_message("NET_CLIENT", "connect() failed: %s", strerror(errno));
        close(sockfd);
        return -1;
    }
    
    log_message("NET_CLIENT", "Connected successfully");
    return sockfd;
}

//principal thread
void* network_client_thread(void* arg) {
    char* server_info = (char*)arg;
    char host[256];
    int port = DEFAULT_PORT;
    
    //parse "host:port" or only "host"
    if (sscanf(server_info, "%255[^:]:%d", host, &port) < 1) {
        log_message("NET_CLIENT", "Invalid server format: %s (use IP:PORT)", server_info);
        return NULL;
    }
    
    log_message("NET_CLIENT", "Client thread started, target: %s:%d", host, port);
    
    int sockfd = network_client_init(host, port);
    if (sockfd < 0) {
        log_message("NET_CLIENT", "Failed to connect to server");
        return NULL;
    }
    
    char buffer[BUFFER_SIZE];
    
    //STEP1 - receive "ok", send "ook"
    if (receive_message(sockfd, buffer, BUFFER_SIZE) < 0) goto cleanup;
    if (strcmp(buffer, "ok") != 0) {
        log_message("NET_CLIENT", "Expected 'ok', got '%s'", buffer);
        goto cleanup;
    }
    
    if (send_message(sockfd, "ook") < 0) goto cleanup;
    
    //STEP2  receive "size W H", send "sok ..."
    if (receive_message(sockfd, buffer, BUFFER_SIZE) < 0) goto cleanup;
    
    int remote_w, remote_h;
    if (sscanf(buffer, "size %d %d", &remote_w, &remote_h) != 2) {
        log_message("NET_CLIENT", "Invalid size format: %s", buffer);
        goto cleanup;
    }
    
    log_message("NET_CLIENT", "Remote world size: %dx%d", remote_w, remote_h);
    
    char sok_msg[BUFFER_SIZE];
    snprintf(sok_msg, sizeof(sok_msg), "sok %dx%d", remote_w, remote_h);
    if (send_message(sockfd, sok_msg) < 0) goto cleanup;
    
    log_message("NET_CLIENT", "Handshake complete, entering main loop");
    g_net_state.client_connected = 1;
    
    //STEP3 - main loop
    while (1) {
        //receive command
        if (receive_message(sockfd, buffer, BUFFER_SIZE) < 0) break;
        
        //check quit
        if (strcmp(buffer, "q") == 0) {
            send_message(sockfd, "qok");
            log_message("NET_CLIENT", "Received quit command");
            break;
        }
        
        //receive "drone"
        if (strcmp(buffer, "drone") != 0) {
            log_message("NET_CLIENT", "Expected 'drone', got '%s'", buffer);
            break;
        }
        
        //receive drone position
        if (receive_message(sockfd, buffer, BUFFER_SIZE) < 0) break;
        
        int remote_dx, remote_dy;
        if (sscanf(buffer, "%d %d", &remote_dx, &remote_dy) == 2) {
            log_message("NET_CLIENT", "receive drone position (%d,%d)", remote_dx, remote_dy);
        }
        
        //send "dok"
        if (send_message(sockfd, "dok drone") < 0) break;
        
        //receive "obst"
        if (receive_message(sockfd, buffer, BUFFER_SIZE) < 0) break;
        if (strcmp(buffer, "obst") != 0) {
            log_message("NET_CLIENT", "Expected 'obst', got '%s'", buffer);
            break;
        }
        
        //send one random obstacles
        pthread_mutex_lock(&g_net_mutex);
        int our_w = g_net_state.world_width;
        int our_h = g_net_state.world_height;
        pthread_mutex_unlock(&g_net_mutex);
        
        int ox = rand() % our_w;
        int oy = rand() % our_h;
        
        char obst_pos[BUFFER_SIZE];
        snprintf(obst_pos, sizeof(obst_pos), "%d %d", ox, oy);
        if (send_message(sockfd, obst_pos) < 0) break;
        
        //receive "pok"
        if (receive_message(sockfd, buffer, BUFFER_SIZE) < 0) break;
        if (strncmp(buffer, "pok", 3) != 0) {
            log_message("NET_CLIENT", "Expected 'pok', got '%s'", buffer);
            break;
        }
        
        //slow the exchange
        struct timespec ts = {0, 500 * 1000 * 1000};
        nanosleep(&ts, NULL);
    }
    
    g_net_state.client_connected = 0;
    log_message("NET_CLIENT", "Protocol completed");
    
cleanup:
    close(sockfd);
    log_message("NET_CLIENT", "Client thread terminated");
    return NULL;
}