#ifndef _SERVER_H_
#define _SERVER_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <thread>
#include <iostream>
#include <string>
#include "sessionManager.h"
#include "log.h"

using namespace std;

typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define SOCKET int
#define closesocket close

class Server {
    
public:
    Server(int port);
    ~Server();

public:
    void serverStart();
    static void startWithListen(Server *server);
    
private:
    void beginListen();
    void nonblockAccecpt(SOCKET mScoket);
    //void sessionConnect(SOCKET mScoket,int lifeTime);
    void sessionConnect(SOCKET mScoket,int lifeTime, int mPort);
protected:
    int mPort;
    SOCKET mScoket;
    char mPong[32];
};

#endif