#ifndef _SESSION_MANAGER_H_
#define _SESSION_MANAGER_H_

#include "session.h"
#include "dataProcessor.h"
#include <map>
using namespace std;

class Session;
class DataProcessor;
class SessionManager{
  public:
    static SessionManager* GetInstance();
    
  private:
    SessionManager();

  public:
    ~SessionManager();
    //void RegistSession(int64_t socket, uint64_t tmo);
    void RegistSession(int64_t socket, uint64_t tmo,int port);
    Session* DetachSession(int64_t sock);

    static void PfnHeartBeats(SessionManager* sessionManager);
    void HeartBeats();

    static void PfnCheckValidate(SessionManager* sessionManager);
    void CheckValidate();
    map<int64_t, Session*> ValidateSession();

    void SendFileInfo(string fileName, string pictureUrl);
    
    int CheckSessionMapSize();

    map<int64_t, Session*> getSessionMap(); 

    void insertToDroneSessionMap(const string& snCode, Session* session);
    
    Session* getSessionBySnCode(const string& snCode);


    void insertToHeartbeatInfoSessionMap(Session* session, HeartbeatInfo heartbeatInfo);
    
    //Session* getSessionBySnCode(const string& snCode);

  public:
    map<string, Session*> snCodeToSessionMap;

    map<Session*, HeartbeatInfo> dronesMap;
  private:
    map<int64_t, Session*> m_sessMap;
    mutex mt;
    volatile uint64_t m_tmo = 2000; 
    DataProcessor* dataProcessor;

};

#endif