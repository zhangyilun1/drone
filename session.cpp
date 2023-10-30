#include "session.h"
#include <log4cplus/loggingmacros.h>

std::mutex mt;
// std::mutex sendMutex;
// std::mutex receiveMutex;

uint64_t getTimestamp()
{
  timeval curtime;
  gettimeofday(&curtime, 0);

  unsigned long ts = (unsigned long) curtime.tv_sec;
    TRACE("curtime : " + to_string(ts));
  // Allow to truncate
  ts *= 1000;
  ts += curtime.tv_usec / 1000;
  return ts;

}

// Session::Session(SOCKET aSocket, uint64_t tmo)
// {
//   mSocket = aSocket;
//   mTimeout = tmo;
//   Update();
//   fcntl(mSocket, F_SETFL, fcntl(mSocket, F_GETFL) | O_NONBLOCK);//Set to non-blocking
// }

Session::Session(SOCKET aSocket, uint64_t tmo, int listenPort)
{
  mSocket = aSocket;
  mTimeout = tmo;
  mlistenPort = listenPort;
  Update();
  fcntl(mSocket, F_SETFL, fcntl(mSocket, F_GETFL) | O_NONBLOCK);//Set to non-blocking
}

Session::~Session()
{
  shutdown(mSocket, SHUT_RDWR);
  closesocket(mSocket);
}


void Session::setTask(Tasks* aTask) {
    task = aTask;
}

Tasks* Session::getTask() {
    return task;
}

int Session::write(const char *aString)
{
  std::lock_guard<std::mutex> lock(mt);
  string sendInfor = aString;
  //TRACE( "send to client : " +  sendInfor);
  int res;
  try {
    res = send(mSocket, aString, (int) strlen(aString), MSG_DONTWAIT);
    }
  catch(...) {
    res = -1;
  }
  return res;
}

int Session::read(char *aBuffer, int aLen)
{
  std::lock_guard<std::mutex> lock(mt);
  int len = recv(mSocket, aBuffer, aLen, MSG_DONTWAIT);
  //TRACE("len : "  + to_string(len) + " aBuffer : " + string(aBuffer));
  if (len > 0) {
    aBuffer[len] = '\0'; // 添加字符串结束符
  } 
  // else 
  //   perror("recv error");
  
  return len;
}


void Session::Update()
{
    mLastActivetime = getTimestamp();
    TRACE( "mLastActivetime : " << mLastActivetime );
}

bool Session::Validate()
{
  // TRACE( "lastTime : " <<  mLastActivetime);
  // TRACE( "time : " << mLastActivetime +  7000 * 2 );
  // TRACE( "current time : " << getTimestamp() );
  // TRACE( "session is validate : " << to_string(mLastActivetime +  7000 * 2 <= getTimestamp()) );
  return mLastActivetime +  7000 * 2 < getTimestamp();
}

