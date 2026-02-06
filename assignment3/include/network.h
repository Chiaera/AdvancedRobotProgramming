#ifndef NETWORK_H 
#define NETWORK_H 
#include <netinet/in.h> 
#include <arpa/inet.h> 
#include <sys/socket.h> 
#define DEFAULT_PORT 5555
#define BUFFER_SIZE 256

//------------------------------------------------------------------------STRUCTS
typedef enum { //network mode
    NET_SERVER,
    NET_CLIENT
} NetworkRole;

typedef enum {//rotation for virtual coordinate system
    ROT_0,
    ROT_90,
    ROT_180,
    ROT_270
} Rotation;

typedef struct { //network info
    NetworkRole role;
    int sockfd;
    int connfd; //server
    char server_ip[64]; //client
    int port;
    Rotation rotation;
} NetworkContext;

//------------------------------------------------------------------------FUNCTIONS
int send_msg(int sockfd, const char *msg);
int recv_msg(int sockfd, char *buffer, size_t buf_size);
int send_ack(int sockfd, const char *ack);
int recv_ack(int sockfd, const char *expected_ack);
int handshake(NetworkContext *ctx);

//socket setup
int network_server_init(NetworkContext *ctx);
int network_client_init(NetworkContext *ctx);

//coordinate conversion
//x1 = x0 + x cos(alfa) - y sin(alfa), y1 = y0 + x sin(alfa) + y cos(alfa) | alfa(0, ±0.5pi, ±pi)
void convert_to_virtual(int x, int y, int *vx, int *vy, int W, int H, Rotation rot);
void convert_from_virtual(int vx, int vy, int *x, int *y, int W, int H, Rotation rot);

//quit
int network_quit(NetworkContext *ctx);

#endif
