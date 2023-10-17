#ifndef _SESSION_H_
#define _SESSION_H_

#include "server.h"
#include "task.h"
#include <sys/time.h>

#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define SOCKET int
#define closesocket close

class Session
{
public:
    unsigned int mLastHeartbeat;

public:
   // Session(SOCKET aSocket, uint64_t tmo);
    Session(SOCKET aSocket, uint64_t tmo, int listenPort);
    ~Session();
    int write(const char *aString);
    int read(char *aBuffer, int aLen);
    void Update();
    bool Validate();

    void setTask(Tasks* task);
    Tasks* getTask();
    
    SOCKET socket() { 
        return mSocket; 
    }

    std::mutex& GetMutex() { 
        return mMutex; 
    } 
public: 
    int mlistenPort;
private:
    uint64_t mLastActivetime;
    SOCKET mSocket = INVALID_SOCKET;
    uint64_t mTimeout;
   
    std::mutex mMutex;
    Tasks* task; 
};
#endif