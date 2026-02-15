#ifndef SERVER_H
#define SERVER_H

#include "robot.h"
#include <netinet/in.h>

typedef enum { SERVER_DISCONNECTED, SERVER_CONNECTED } ServerState;

typedef struct {
  int server_fd;
  int client_fd;
  int port;
  ServerState state;
  struct sockaddr_in address;
} Server;

int server_init(Server *server, int port);
void server_poll(Server *server, Robot *robot);
void server_close(Server *server);
void server_broadcast(Server *server, const char *msg);

#endif
