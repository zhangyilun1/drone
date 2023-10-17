#include "server.h"
#include <log4cplus/loggingmacros.h>
#include <sstream>
std::mutex mtx;

SessionManager *m_sessionManager = SessionManager::GetInstance();

Server :: Server(int port){
    this->mPort = port;
}

Server::~Server(){

}

void Server:: serverStart(){
    thread th1(startWithListen, this);
    th1.detach();
}


void Server::startWithListen(Server* server){

    //std::lock_guard<std::mutex> lock(mtx);
    pthread_t tid = pthread_self();
    std::stringstream ss;
    ss << tid;
    TRACE( "tid : " + ss.str());
    //TRACE("Before lock acquisition");
    server->beginListen();
    //TRACE("Lock released");
}

void Server::beginListen(){
    int mOne = 1;
    SOCKADDR_IN t;
    signal(SIGPIPE, SIG_IGN);
    mScoket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    setsockopt(mScoket, SOL_SOCKET, SO_REUSEADDR, (char *)&mOne, sizeof(int));
    if(mScoket == INVALID_SOCKET)
    {
        return;
    }
    t.sin_family = AF_INET;
    t.sin_port = htons(this->mPort);
    t.sin_addr.s_addr = htonl(INADDR_ANY);
    if(::bind(mScoket, (SOCKADDR *)&t, sizeof(t)) == SOCKET_ERROR)
    {
        return;
    }
    if(listen(mScoket, 4) == SOCKET_ERROR)
    {
        return;
    }
    TRACE( "Server started. Listening on port : " + to_string(this->mPort));
    while(1)
    {
        nonblockAccecpt(mScoket);
    }
}


void Server::nonblockAccecpt(SOCKET mSocket)
{   
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(mSocket, &rset);
    int nfds = mSocket + 1;
    struct timeval timeout;
    timeout.tv_sec = 1;  // 设置超时时间为1秒
    timeout.tv_usec = 0;
    ::memset(&timeout, 0, sizeof(timeout));
    if (::select(nfds, &rset, 0, 0, &timeout) > 0)
    {
        if (FD_ISSET(mSocket, &rset))
        {
            SOCKADDR_IN addr;
            socklen_t len = sizeof(addr);
            memset(&addr, 0, sizeof(addr));
            SOCKET socket = ::accept(mSocket, (SOCKADDR*) &addr, &len);
            if (socket == INVALID_SOCKET) 
            {
                return;
            }
            char clientIP[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(addr.sin_addr), clientIP, INET_ADDRSTRLEN);
            int clientPort = ntohs(addr.sin_port);
            string str(clientIP);
            TRACE( "Client IP: " + str);
            TRACE( "Client Port: " + to_string(clientPort));
            //sessionConnect(socket, 100000);
            //sessionConnect(socket, 100000, this->mPort);
            sessionConnect(socket, 140, this->mPort);
        }
    }
}
//void Server::sessionConnect(SOCKET socket, int lifeTime)
void Server::sessionConnect(SOCKET socket, int lifeTime, int port)
{
    SessionManager *m_sessionManager = SessionManager::GetInstance();
    //m_sessionManager->RegistSession(socket, lifeTime)
    m_sessionManager->RegistSession(socket, lifeTime, port);
}




