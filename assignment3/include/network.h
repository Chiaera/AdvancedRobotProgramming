/* protocol.h
    - header file for network communication between simulators
    - protocol:
        server: sends drone position, receives obstacles
        client: receives drone position, sends obstacles
 */

#ifndef NETWORK_H
#define NETWORK_H

#include <stdint.h>
#include "map.h"  

#define DEFAULT_PORT 8888 //default port for network communication
#define MAX_CONNECTIONS 5 //max number of clients that can connect to the server
#define BUFFER_SIZE 256 //size of buffer for messages


typedef struct { //obstacles received from the network
    int x, y; //coordinates of the obstacle
    int active; //1 = active, 0 = inactive
} ClientObstacle;

//connection state shared between blackboard and network threads - access protected by pthread_mutex (g_net_mutex)
typedef struct {
    //data to send (my drone position)
    double drone_x;             
    double drone_y;           
    
    //data to receive (external obstacles)
    ClientObstacle* external_obstacles; //to be allocated dynamically
    int num_external_obstacles;         
    int max_external_obstacles;                  
    
    //info about the world size
    int world_width;            
    int world_height;          
    
    //connection status
    int server_running; // 1 = server running, 0 = stopped
    int client_connected; // 1 = client connected, 0 = disconnected
} NetworkState;


//--------------------------------------------------------------------------------- MEMORY MANAGEMENT

//initialize the network state (allocate memory for external obstacles)
int network_state_init(NetworkState* ns, int world_w, int world_h, int max_obstacles);

//free the network state memory
void network_state_cleanup(NetworkState* ns);


//--------------------------------------------------------------------------------- SERVER

//initialize the network socket for server
int network_server_init(int port);

//thread function for server to accept connections and handle protocol
void* network_server_thread(void* arg);

//--------------------------------------------------------------------------------- CLIENT
//connect to server at host:port
int network_client_init(const char* host, int port);

//thread function for client to handle protocol
void* network_client_thread(void* arg);
//--------------------------------------------------------------------------------- UTILITIES
//send a message (appends '\n')
int send_message(int sockfd, const char* msg);

//receive a message (reads until '\n')
int receive_message(int sockfd, char* buffer, int buffer_size);

#endif




