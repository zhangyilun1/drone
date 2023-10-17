#include "sessionManager.h"
#include <sstream>

static SessionManager *sessionManager;

SessionManager *SessionManager::GetInstance()
{
    if (sessionManager == nullptr)
        sessionManager = new SessionManager();

    return sessionManager;
}

SessionManager::SessionManager()
{
    DataProcessor* dataProcessor = new DataProcessor();

    std::thread t(PfnHeartBeats, this);
    t.detach();

    std::thread t2(PfnCheckValidate, this);
    t2.detach();
}

SessionManager::~SessionManager()
{
    delete dataProcessor;  
    for (auto it = m_sessMap.begin(); it != m_sessMap.end(); it++)
    {
        delete it->second;
    }
}

// void SessionManager::RegistSession(int64_t socket, uint64_t tmo)
void SessionManager::RegistSession(int64_t socket, uint64_t tmo, int port)
{
    std::lock_guard<std::mutex> lock(mt);

    // if session unexisted insert to map
    if (m_sessMap.find(socket) != m_sessMap.end())
    {
        // update session status
        m_sessMap[socket]->Update();
        return;
    }

    if (tmo < m_tmo)
    {
        m_tmo = tmo;
    }

    //m_sessMap.insert(pair<int64_t, Session *>(socket, new Session(socket, tmo)));
    m_sessMap.insert(pair<int64_t, Session *>(socket, new Session(socket, tmo, port)));

    if(port == 2345){
        TRACE("dronesMap size : " + to_string(dronesMap.size()));
        TRACE("snCodeToSessionMap size : " + to_string(snCodeToSessionMap.size()));
        for(auto droneSession = snCodeToSessionMap.begin(); droneSession!=snCodeToSessionMap.end(); droneSession++){
            TRACE("drone snCode :" + droneSession->first);
            //HeartbeatInfo infor = dronesMap.find(droneSession->second);
            auto droneInfo = dronesMap.find(droneSession->second);
            if (droneInfo != dronesMap.end()) {
                HeartbeatInfo infor = droneInfo->second;
                TRACE("longitude :" + to_string(infor.longitude));
                TRACE("latitude :" + to_string(infor.latitude));
                TRACE("altitude :" + to_string(infor.altitude));
                TRACE("missionID :" + to_string(infor.missionID));
                TRACE("status :" + to_string(infor.status));
                TRACE("index :" + to_string(infor.index));
                TRACE("count :" + to_string(infor.count));       
                nlohmann::json jsonData;
                jsonData["snCode"] = droneSession->first;
                jsonData["longitude"] = to_string(infor.longitude);
                jsonData["latitude"] = to_string(infor.latitude);
                jsonData["altitude"] = to_string(infor.altitude);
                jsonData["missionID"] = to_string(infor.missionID);
                jsonData["status"] = to_string(infor.status);
                jsonData["index"] = to_string(infor.index);
                jsonData["count"] = to_string(infor.count);
                // jsonData["snCode"] = "AD00F";
                // jsonData["longitude"] = "123321";
                // jsonData["latitude"] = "321123";
                // jsonData["altitude"] = "787878";
                // jsonData["missionID"] = "-1";
                // jsonData["status"] = "3";
                // jsonData["index"] = "1";
                // jsonData["count"] = "-4";
                string jsonString = jsonData.dump();
                const char* buffer1 = jsonString.c_str();
                TRACE("buffer1 :" + to_string(strlen(buffer1)));
                ssize_t bytesSent = send(socket, buffer1, strlen(buffer1), 0);
                TRACE("SEND TO PHP SIZE :" + to_string(bytesSent));
            }
        }
    } 
}

map<int64_t, Session *> SessionManager::getSessionMap()
{
    return m_sessMap;
}

void SessionManager::PfnHeartBeats(SessionManager *sessionManager)
{
    sessionManager->HeartBeats();
}
void SessionManager::HeartBeats()
{
    while (true)
    {
        TRACE("check hearbeat");
        const char mPong[32] = {"* PONG \n"};
        uint8_t buffer[100000];  // Use uint8_t buffer for receiving data
        TRACE("m_sessMap size : " +  to_string(m_sessMap.size()));
        TRACE("snCodeToSessionMap size : " +  to_string(snCodeToSessionMap.size()));
        TRACE("dronesMap size : " +  to_string(dronesMap.size()));
        for (auto it = m_sessMap.begin(); it != m_sessMap.end(); it++)
        {
            // TRACE("SOCKET : " +  to_string(it->second->socket()));
            int ret = it->second->read(reinterpret_cast<char*>(buffer), sizeof(buffer));
            // TRACE("bytesRead : " + to_string(ret));
            // if(buffer[0]== 0x00){
            //     if(dataProcessor->getHeartbeatInfo(buffer, ret).isInitialized == true){
            //         it->second->Update();
            //     } 
            // }
            //else 
            if (ret > 0 && !(strncmp(reinterpret_cast<char*>(buffer), "* PING", 6) == 0)){
                TRACE(" cmd : " + to_string(buffer[0]));
                vector<vector<uint8_t>> packets;
                if(it->second->mlistenPort == 32330){
                    TRACE("come from update port");
                    dataProcessor->processUpgrade(buffer, ret, it->second);  
                }else if(it->second->mlistenPort == 32321) {              
                    for (size_t i = 0; i < ret;) {
                        uint16_t cmd = buffer[i];
                        TRACE(" cmd : " + to_string(cmd));
                        uint16_t dataLength = (static_cast<uint16_t>(buffer[i + 1]) << 8) | buffer[i + 2];
                        int intValue = static_cast<int>(dataLength);
                        TRACE(" intValue : " + to_string(intValue));
                        if (i + 3 + intValue <= ret) {
                            TRACE(" value : " + to_string(i + 3 + intValue));
                            vector<uint8_t> packet(buffer + i, buffer + i + 3 + dataLength);
                            packets.push_back(packet);
                            i += 3 + dataLength;
                        }
                        else
                            break;
                    }
             
                    for (auto & packet : packets) {
                        const uint8_t *packetData = packet.data();
                        size_t packetSize = packet.size();
                        Tasks* task = it->second->getTask();
                        if(task && packetData[0]== 0x06){
                            TRACE("getTaskInfo cmd : " + to_string(packetData[0]));
                            dataProcessor->getTaskInfo(packetData, packetSize, it->second);
                        } else if(task && packetData[0]== 0x08){
                            TRACE("getRecord cmd : " + to_string(packetData[0]));
                            dataProcessor->getRecord(packetData, packetSize, it->second);
                        } 
                        else 
                            dataProcessor->processDronesData(packetData, packetSize, it->second);
                    }
                    it->second->Update();
                }else if(it->second->mlistenPort == 2345)
                {
                    dataProcessor->processReceivedData(reinterpret_cast<char*>(buffer), ret, it->second);
                }   
            } 
            
            // else if(it->second->mlistenPort == 2345){
            //     TRACE("dronesMap size : " + to_string(dronesMap.size()));
            //     TRACE("snCodeToSessionMap size : " + to_string(snCodeToSessionMap.size()));
            //     nlohmann::json jsonData;
            //     jsonData["snCode"] = "drone1";
            //     jsonData["longitude"] = "123321";
            //     jsonData["latitude"] = "321123";
            //     jsonData["altitude"] = "787878";
            //     jsonData["missionID"] = "-1";
            //     jsonData["status"] = "-2";
            //     jsonData["index"] = "-3";
            //     jsonData["count"] = "-4";
            //     string jsonString = jsonData.dump();
            //     const char* buffer1 = jsonString.c_str();
            //     TRACE("buffer1 :" + to_string(strlen(buffer1)));
            //     ssize_t bytesSent = send(it->second->socket(), buffer1, strlen(buffer1), 0);
            //     TRACE("SEND TO PHP SIZE :" + to_string(bytesSent));
            //     close(it->second->socket()); 
                // for(auto droneSession = snCodeToSessionMap.begin(); droneSession!=snCodeToSessionMap.end(); droneSession++){
                //     TRACE("drone snCode :" + droneSession->first);
                //     //HeartbeatInfo infor = dronesMap.find(droneSession->second);
                //     auto droneInfo = dronesMap.find(droneSession->second);
                //     if (droneInfo != dronesMap.end()) {
                //         HeartbeatInfo infor = droneInfo->second;
                //         TRACE("longitude :" + to_string(infor.longitude));
                //         TRACE("latitude :" + to_string(infor.latitude));
                //         TRACE("altitude :" + to_string(infor.altitude));
                //         TRACE("missionID :" + to_string(infor.missionID));
                //         TRACE("status :" + to_string(infor.status));
                //         TRACE("index :" + to_string(infor.index));
                //         TRACE("count :" + to_string(infor.count));
                //         nlohmann::json jsonData;
                //         jsonData["snCode"] = droneSession->first;
                //         jsonData["longitude"] = to_string(infor.longitude);
                //         jsonData["latitude"] = to_string(infor.latitude);
                //         jsonData["altitude"] = to_string(infor.altitude);
                //         jsonData["missionID"] = to_string(infor.missionID);
                //         jsonData["status"] = to_string(infor.status);
                //         jsonData["index"] = to_string(infor.index);
                //         jsonData["count"] = to_string(infor.count);
                //         string jsonString = jsonData.dump();
                //         const char* buffer = jsonString.c_str();
                //         TRACE("buffer :" + to_string(strlen(buffer)));
                //         ssize_t bytesSent = send(it->second->socket(), buffer, strlen(buffer), 0);
                //         TRACE("SEND TO PHP SIZE :" + to_string(bytesSent));
                //     } else {
                //        TRACE("HeartbeatInfo not find ");
                //     }

                

            //}
          
            memset(buffer, 0, sizeof(buffer));
        }
        sleep(2);
    }
}

// void SessionManager::HeartBeats()
// {
//     while (true)
//     {
//         TRACE("check hearbeat");
//         // std::lock_guard<std::mutex> lock(mt);
//         const char mPong[32] = {"* PONG \n"};
//         char buffer[1024];
//         // uint8_t buffer[1024];
//         for (auto it = m_sessMap.begin(); it != m_sessMap.end(); it++)
//         {
//             int ret = it->second->read(buffer, 1024);
//             uint8_t* uint8_buffer = reinterpret_cast<uint8_t*>(buffer);
//             // int ret = it->second->read(buffer, 1024);
//             if (6 == ret && strncmp(buffer, "* PING", 6) == 0){
//                 string str = buffer;
//                 TRACE("check hearbeat : " + to_string(ret) + " info : " + str);
//                 it->second->write(mPong);
//                 it->second->Update();
//             }
//             else if (ret > 0 && !(strncmp(buffer, "* PING", 6) == 0)){
//                 TRACE("bytesRead : " + to_string(ret));
//                 // TRACE("Received data from client : " + std::string(buffer));
//                 // DataProcessor* dataProcessor = new DataProcessor();
//                 dataProcessor->processDronesData(buffer, ret);

//                 // delete it->second;
//                 // processReceivedData(buff, ret);
//                 //it->second->write("success !!!");
//             }
//             *(uint64_t *)buffer = 0;
//         }
//         // usleep(m_tmo/10*1000);//Hang for a while
//         sleep(2);
//     }
// }

void SessionManager::PfnCheckValidate(SessionManager *sessionManager)
{
    sessionManager->CheckValidate();
}

void SessionManager::CheckValidate()
{
    // std::lock_guard<std::mutex> lock(mt);
    while (true)
    {
        //usleep(150 * 100000);
        usleep(50 * 100000);
        // 14000000
        ValidateSession();
    }
}


map<int64_t, Session*> SessionManager::ValidateSession()
{
    TRACE("ValidateSession");
    std::vector<Session*> delVec;
    auto iter = m_sessMap.begin();
    
    // 遍历 m_sessMap，找到无效的 Session
    while (iter != m_sessMap.end())
    {
        if (iter->second->Validate())
        {
            delVec.push_back(iter->second);
        }
        iter++;
    }

    for (size_t i = 0; i < delVec.size(); i++)
    {
        Session* sessionToDelete = delVec[i];
        // 删除 m_sessMap 中的 Session
        m_sessMap.erase(sessionToDelete->socket());

        auto snCodeIter = snCodeToSessionMap.begin();
        while (snCodeIter != snCodeToSessionMap.end())
        {
            if (snCodeIter->second == sessionToDelete)
            {
                snCodeToSessionMap.erase(snCodeIter++);
            }
            else
            {
                ++snCodeIter;
            }
        }

        // 删除 dronesMap 中的 Session
        auto dronesIter = dronesMap.find(sessionToDelete);
        if (dronesIter != dronesMap.end())
        {
            dronesMap.erase(dronesIter);
        }
        // 删除 Session 对象
        delete sessionToDelete;
    }

    return m_sessMap;
}


// map<int64_t, Session *> SessionManager::ValidateSession()
// {
//     // std::lock_guard<std::mutex> lock(mt);
//     TRACE("ValidateSession");
//     std::vector<int64_t> delVec;
//     Session* session;
//     auto iter = m_sessMap.begin();
//     for (; iter != m_sessMap.end(); iter++)
//     {
//         if (iter->second->Validate())
//         {
//             delVec.push_back(iter->first);
//             session = iter->second;
//         }
           
//     }   
//     for (size_t i = 0; i < delVec.size(); i++)
//     {
//         m_sessMap.erase(delVec[i]);
//         // string snCodeToDelete = delVec[i]->getSnCode();
//         // snCodeToSessionMap.erase(delVec[i]);
//         //dronesMap.erase(delVec[i]);
//     }
//     return m_sessMap;
// }

void SessionManager::SendFileInfo(string key, string value)
{
}

int SessionManager::CheckSessionMapSize()
{
    return m_sessMap.size();
}

void SessionManager::insertToDroneSessionMap(const string& snCode, Session* session) {
    lock_guard<mutex> lock(mt);
    snCodeToSessionMap.insert(std::make_pair(snCode, session));
}

Session* SessionManager::getSessionBySnCode(const std::string& snCode) {
    auto it = snCodeToSessionMap.find(snCode);
    if (it != snCodeToSessionMap.end()) {
        return it->second;
    }
    return nullptr;
}



void SessionManager::insertToHeartbeatInfoSessionMap(Session* session, HeartbeatInfo heartbeatInfo) {
    lock_guard<mutex> lock(mt);
    dronesMap.insert(std::make_pair(session,heartbeatInfo));
}

// Session* SessionManager::getSessionBySnCode(const std::string& snCode) {
//     auto it = snCodeToSessionMap.find(snCode);
//     if (it != snCodeToSessionMap.end()) {
//         return it->second;
//     }
//     return nullptr;
// }