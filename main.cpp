#include "server.h"
#include "sqliteConnection.h"
#include "config.h"
#include "log.h"

//#include "/opt/homebrew/Cellar/nlohmann-json/3.11.2/include/nlohmann/json.hpp" 

 

#define DRONE_SERVER_PORT "DRONE_SERVER_PORT"
#define PHP_SERVER_PORT "PHP_SERVER_PORT"
#define UPGRADE_SERVER_PORT "UPGRADE_SERVER_PORT"
#define SQLITE_PATH "SQLITE_PATH"

// std::mutex sessionMutex;



// void processReceivedData(const char* data, int length) {
//     // Convert the received data to a string for processing
//     std::string receivedData(data, length);
//     // Print the received data
//     std::cout << "Received data: " << receivedData << std::endl;
//     try {
//         nlohmann::json jsonData = nlohmann::json::parse(receivedData);
        
//         // Now you can access JSON data and perform operations
//         if (jsonData.contains("controller")) {
//             std::string value = jsonData["controller"];
//             std::cout << "Value of key: " << value << std::endl;
//             if(!value.empty())
//             {
//                 if(value == "get_drone")
//                 {
//                     TRACE("value : " + value);
//                     // SQLiteConnector.executeQuery("select * from drone ;");
//                 }
//             }

//         }
//     } catch (const std::exception& e) {
//         // JSON parsing failed
//         std::cerr << "Error parsing JSON: " << e.what() << std::endl;
//     }
// }

// void start(Session* session) {

//     //std::unique_lock<std::mutex> lock(sessionMutex);
//     TRACE("start");
//     char buff[1024];
//     int bytesRead = session->read(buff, sizeof(buff));
//     pthread_t tid = pthread_self();
//     std::stringstream ss;
//     ss << tid;
//     TRACE("bytesRead : " + to_string(bytesRead));
//     if (bytesRead > 0 && !(strncmp(buff, "* PING", 6) == 0)) {
//         TRACE("Received data from client : " + std::string(buff) + " tid = " + ss.str());
//         processReceivedData(buff, bytesRead);
//         session->write("success !!!");

//     }
//     else 
//     {
//         session->write("fail !!!");
//         TRACE("fail !! ");  
//     }
//     memset(buff, 0, sizeof(buff));

// }


//vector<thread>& sessionThreads
void handleClient(SessionManager* m_sessionManager) {
    // 接收和发送数据的缓冲区
    //std::unique_lock<std::mutex> lock(sessionMutex);
    TRACE( "sission map size in handleClient: " + to_string(m_sessionManager->CheckSessionMapSize()));
    if(m_sessionManager->CheckSessionMapSize() == 0)
    {
        TRACE( "sission map is null ");
    }
    else if(m_sessionManager->CheckSessionMapSize() > 0)
    {
        TRACE( "sission map size : " + to_string(m_sessionManager->CheckSessionMapSize()));
        // for(auto sessionPair : m_sessionManager->getSessionMap())
        // {
        //     Session* session = sessionPair.second;
        //     sessionThreads.emplace_back(start, session);
        // }
    }
}   







int main(){

    SessionManager *m_sessionManager = SessionManager::GetInstance();
    //read config content  
    //ConfigFile* mConfig = new ConfigFile("./config.txt");
    int dronePort = 32321;
    int phpServerPort = 2345;
    int upgradeServerPort =32330;
    string SQLitePath = "/Users/zhangyilun/Desktop/socket_connect/feixing.db";
    //mConfig->Read(SQLITE_PATH, SQLitePath);
    // int dronePort = mConfig->Read(DRONE_SERVER_PORT, dronePort);
    // int phpServerPort = mConfig->Read(PHP_SERVER_PORT, phpServerPort);
    // int upgradeServerPort = mConfig->Read(UPGRADE_SERVER_PORT, upgradeServerPort);
    // string SQLitePath = mConfig->Read(SQLITE_PATH, SQLitePath);

    TRACE( " drone port : " + to_string(dronePort) );
    TRACE( " phpServer port : " + to_string(phpServerPort));
    TRACE( " upgradeServer port : " + to_string(upgradeServerPort));
    TRACE( " SQLite Path : " +  SQLitePath );

    // create sqlite connect
    SQLiteConnection& SQLiteConnector = SQLiteConnection::getInstance();
    bool connection = SQLiteConnector.open(SQLitePath);
    TRACE( " connection is  : " +  to_string(connection));

    // port: 32321 begin listen
    Server* serverForDrone  = new Server(dronePort);
    thread th1(serverForDrone->startWithListen, serverForDrone); 
    th1.detach();

    //port: 2345 begin listen
    Server* serverForPHP = new Server(phpServerPort);
    thread th2(serverForPHP->startWithListen, serverForPHP);    
    th2.detach();

    Server* serverForUpgrade = new Server(upgradeServerPort);
    thread th3(serverForUpgrade->startWithListen, serverForUpgrade); 
    th3.detach();


    thread clientThread(handleClient, m_sessionManager);
    clientThread.detach();


    vector<thread> threads; 

    while(true)
    {    
        // for (auto it = threads.begin(); it != threads.end(); ) {
        //     if (it->joinable()) {
        //         it->join();  // 等待线程完成
        //         it = threads.erase(it);  // 删除已完成的线程
        //     } else {
        //         ++it;
        //     }
        // }

        // threads.clear();

        // handleClient(m_sessionManager, threads);
  
        sleep(10);
    }
    
    //delete mConfig;
    delete serverForDrone;
    delete serverForPHP;
}

