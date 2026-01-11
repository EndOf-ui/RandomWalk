#ifndef SOCKET
#define SOCKET

#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

int passive_socket_init(const int port);
int passive_socket_wait_for_client(int passiveSocket);
void passive_socket_destroy(int socket);
int connect_to_server(const char * name, const int port);
void active_socket_destroy(int socket);

#endif // !SOCKET
