#include "dataProcessor.h"
#include <list>
#include <chrono>
#include <filesystem>
#include <sys/stat.h>
// #include "./algorithm/PointToLine.h"
// #include "./algorithm/PointToLineV2.h"
// #include "./algorithm/PointToLineV3.h"
using namespace std;

#define BUFFER_SIZE 10240

static SQLiteConnection* SQLiteConnector;

template<class T>
void Serialization(std::stringstream& ss, const T& t){
 cereal::BinaryOutputArchive oarchive(ss); 
 oarchive(t);
}

template<class T>
void UnSerialization(std::stringstream& ss, T& t){
 cereal::BinaryInputArchive iarchive(ss); 
 iarchive(t);
}


template <class Archive>
void save(Archive &ar, const std::list<WaypointInformation> &list) {
    ar(list.size()); // Serialize the size of the list
    for (const auto &item : list) {
        ar(item); // Serialize each element of the list
    }
}

template <class Archive>
void load(Archive &ar, std::list<WaypointInformation> &list) {
    size_t size;
    ar(size); // Deserialize the size of the list
    list.clear();
    for (size_t i = 0; i < size; ++i) {
        WaypointInformation item;
        ar(item); // Deserialize each element of the list
        list.push_back(item);
    }
}

uint8_t* beforeSerialization(const uint8_t* data, int length)
{
    int newLength = length - 2;
    // Create a new array to store the data
    uint8_t* newData = new uint8_t[newLength];
    // Copy data from the original array to the new array
    for (int i = 3; i < length; i++) {
        newData[i - 3] = data[i];
    }
    return newData;
}

DataProcessor::DataProcessor()
{
   
}

DataProcessor::~DataProcessor()
{
    
}

void DataProcessor::processReceivedData(const char* data, int length, Session* session) {
    // Convert the received data to a string for processing
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
    std::string receivedData(data, length);
    // Print the received data
    // std::cout << "Received data: " << receivedData << std::endl;
    try {
        nlohmann::json jsonData = nlohmann::json::parse(receivedData);
        if (jsonData.contains("controller")) {
            std::string controller = jsonData["controller"];
            TRACE("controller : " + controller );
            // std::cout << "Value of key: " << controller << std::endl;
            if(!controller.empty())
            {
                if(controller == "get_drone")
                {
                    TRACE("controller : " + controller);
                    uint8_t cmd = 0x0b;
                    TRACE("cmd : " + to_string(cmd));
                    DroneINFO droneInfo;
                    droneInfo.snCode = jsonData["data"]["droneSncode"];
                    TRACE("droneInfo.snCode  : " + droneInfo.snCode);
                    droneInfo.droneType = jsonData["data"]["droneType"];
                    TRACE(" droneInfo.droneType   : " +  droneInfo.droneType);
                    droneInfo.lensType = jsonData["data"]["lensType"];
                    TRACE("droneInfo.lensType  : " + droneInfo.lensType);
                    droneInfo.maxSpeed = jsonData["data"]["maxSpeed"];
                    TRACE("droneInfo.maxSpeed  : " + droneInfo.maxSpeed);
                    droneInfo.systemVersion = jsonData["data"]["systemVersion"];
                    TRACE("droneInfo.systemVersion  : " + droneInfo.systemVersion);

                    stringstream ss;
                    {
                        cereal::BinaryOutputArchive oarchive(ss);
                        oarchive(droneInfo);
                    }

                    TRACE("begin insert to info");
                    std::vector<uint8_t> info;
                    info.push_back(cmd);
                    auto serializedData = ss.str();
                    size_t dataLength = serializedData.size();
                    TRACE("serialData: " + to_string(serializedData.size()));
                    uint8_t highByte = (dataLength >> 8) & 0xFF;
                    uint8_t lowByte = dataLength & 0xFF;
                    info.push_back(highByte);
                    info.push_back(lowByte);
                    TRACE("finished insert to info");
                    // Append the serialized data to the info vector
                    //auto serializedData = ss.str();
                    info.insert(info.end(), serializedData.begin(), serializedData.end());

                    SessionManager *m_sessionManager = SessionManager::GetInstance(); 
                    Session* targetSession = m_sessionManager->getSessionBySnCode(droneInfo.snCode);
                    TRACE("snCodeToSessionMap : " + to_string(m_sessionManager->snCodeToSessionMap.size()));
                    if (targetSession != nullptr) {
                        // Send the info data using the session's write method
                        // targetSession->write(reinterpret_cast<const char*>(info.data()));
                        send(targetSession->socket(), info.data(), info.size(), 0);
                    } else {
                        TRACE("session unexisted");
                    }
                    for (const uint8_t byte : info) {
                        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                    }
                    std::cout << std::dec << std::endl;
                    ss.str("");
                    ss.clear();
                }
                else if(controller == "index"){
                    TRACE("controller : " + controller);
                    uint8_t cmd = 0x06;
                    size_t dataLength = 0;
                    uint8_t highByte = (dataLength >> 8) & 0xFF;
                    uint8_t lowByte = dataLength & 0xFF;
                    string snCode = jsonData["data"]["snCode"];
                    string submissionID = jsonData["data"]["submissionID"];
                    // TRACE("snCode: " + snCode);
                    // TRACE("submissionID: " + submissionID);
                    // TaskInfo taskInfo;
                    // taskInfo.rtk = jsonData["data"]["rtk"];

                    Tasks* task = new Tasks();
                    task->setSubmissionID(jsonData["data"]["submissionID"]);
                    TRACE(" taskInfo.submissionID   : " +  task->getSubmissionID());
                    task->setMissionType(jsonData["data"]["missionType"]);
                    TRACE(" taskInfo.missionType   : " +  task->getMissionType());
                    task->setRtk(jsonData["data"]["rtk"]);
                    TRACE(" taskInfo.RTK   : " +  to_string(task->getRtk()));
                    task->setFlightType(jsonData["data"]["flight"]);
                    TRACE(" taskInfo.flight  : " + to_string(task->getFlightType()));
                    task->setTimestamp(jsonData["data"]["timestamp"]);
                    TRACE(" taskInfo.timestamp  : " + to_string(task->getTimestamp()));
                    task->setWhetherRecord(jsonData["data"]["record"]);
                    TRACE("taskInfo.record  : " + to_string(task->getWhetherRecord()));
                    task->setImgwidth(jsonData["data"]["imgwidth"]);
                    TRACE("taskInfo.imgwidth  : " + to_string(task->getImgwidth()));
                    task->setImgheigth( jsonData["data"]["imgheight"]);
                    TRACE("taskInfo.imgheight  : " +  to_string(task->getImgheigth()));
                    
                    if(jsonData["data"]["return"] == 254){
                        double landingLongtitude = jsonData["data"]["landingLongtitude"];
                        TRACE("taskInfo.landingLongtitude  : " +  to_string(jsonData["data"]["landingLongtitude"]));
                        double landingLatitude = jsonData["data"]["landingLatitude"];
                        TRACE("taskInfo.landingLatitude  : " +  to_string(jsonData["data"]["landingLatitude"]));
                        double landingAltitude = jsonData["data"]["landingAltitude"];
                        TRACE("taskInfo.landingAltitude  : " +  to_string(jsonData["data"]["landingAltitude"]));
                        if(!isnan(landingLongtitude) && !isnan(landingLatitude) && !isnan(landingAltitude))
                        {   
                            Gps returnGps;
                            returnGps.stc_alt = landingAltitude;
                            returnGps.stc_lat = landingLatitude;
                            returnGps.stc_lon = landingLongtitude;
                            task->setReturnGPS(returnGps);
                        }
                    }
                    task->setReturnCode(jsonData["data"]["return"]);
                    TRACE("taskInfo.returnCode  : " +  to_string(task->getReturnCode()));

                    // TRACE("task rtk : " + task->getRtk());
                    // session->setTask(task);
                    // taskInfo.flight = jsonData["data"]["flight"];
                    // TRACE(" taskInfo.flight   : " +  to_string(taskInfo.flight));
                    // taskInfo.timestamp = jsonData["dasta"]["timestamp"];
                    // TRACE("taskInfo.timestamp  : " + to_string(taskInfo.timestamp));
                    // taskInfo.record = jsonData["data"]["record"];
                    // TRACE("taskInfo.record  : " + to_string(taskInfo.record));
                    // taskInfo.imgwidth = jsonData["data"]["imgwidth"];
                    // TRACE("taskInfo.imgwidth  : " + to_string(taskInfo.imgwidth));
                    // taskInfo.imgheight = jsonData["data"]["imgheight"];
                    // TRACE("taskInfo.imgheight  : " + to_string(taskInfo.imgheight));
                    // Tasks* task = new Tasks();
                    // task->setImgheigth

                    //WaypointInformation* waypoint = new WaypointInformation;

                    SessionManager *m_sessionManager = SessionManager::GetInstance(); 
                    Session* targetSession = m_sessionManager->getSessionBySnCode(snCode);
                    vector<uint8_t> info;
                    info.push_back(cmd); 
                    info.push_back(highByte); 
                    info.push_back(lowByte);
                    if (targetSession != nullptr) {
                        targetSession->setTask(task);
                        ssize_t bytesSent = send(targetSession->socket(), info.data(), info.size(), 0);
                        if (bytesSent == static_cast<ssize_t>(info.size())) {
                            TRACE("Data sent successfully.");
                        } else if (bytesSent == -1) {
                            TRACE("Failed to send data. Error code: " + std::to_string(errno));
                        } else {
                            TRACE("Partial data sent. Sent bytes: " + std::to_string(bytesSent));
                        }
                    } else {
                        TRACE("session unexisted");
                    }

                    // string  = jsonData["data"]["snCode"];
                    // WaypointInformation* waypoint = new WaypointInformation;
                    // SessionManager *m_sessionManager = SessionManager::GetInstance(); 
                    // Session* targetSession = m_sessionManager->getSessionBySnCode(snCode);
                    
                    // if(targetSession!= nullptr)
                    // {
                    //     getTaskUploadTimeStamp();
                        
                    //     TRACE("drone  online ");
                    // }
                    // else 

                    // WaypointInformation* waypoint1 = getWaypointInfor();

                    // std::vector<uint8_t> info;
                    // info.push_back(cmd); 

                    // stringstream INFO;
                    // Serialization(INFO,taskInfo);
                    // stringstream Waypoint;
                    // Serialization(Waypoint,waypoint);

                    // stringstream ss;
                    // {
                    //     cereal::BinaryOutputArchive oarchive(ss);
                    //     oarchive(taskInfo);
                    // }
                    // // stringstream ssss;
                    // // {
                    // //     cereal::BinaryOutputArchive oarchive(ssss);
                    // //     oarchive(waypoint);
                    // // }


                    // auto serializedData = ss.str();
                    // TRACE("serializedData : " + to_string(serializedData.size()));
                    // auto serializedWaypointData = Waypoint.str();
                    // TRACE("serializedWaypointData : " + to_string(serializedWaypointData.size()));
                    // size_t dataLength = serializedData.size() + serializedWaypointData.size();
                    // uint8_t highByte = (dataLength >> 8) & 0xFF;
                    // uint8_t lowByte = dataLength & 0xFF;
                    // info.push_back(highByte);
                    // info.push_back(lowByte);

                    // info.insert(info.end(), serializedData.begin(), serializedData.end());
                    // info.insert(info.end(), serializedWaypointData.begin(), serializedWaypointData.end());


                    for (const uint8_t byte : info) {
                        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                    }
                }else if(controller == "takeoffLink"){
                    TRACE("takeoffLink controller");
                    uint8_t cmd = 0x01;
                    size_t dataLength = 0;
                    uint8_t highByte = (dataLength >> 8) & 0xFF;
                    uint8_t lowByte = dataLength & 0xFF;
                    string snCode = jsonData["data"]["snCode"];
                    TRACE("snCode : " + snCode );
                    SessionManager *m_sessionManager = SessionManager::GetInstance(); 
                    Session* targetSession = m_sessionManager->getSessionBySnCode(snCode);
                    TRACE("snCodeToSessionMap : " + to_string(m_sessionManager->snCodeToSessionMap.size()));
                   // TRACE("targetSession socket: " + to_string(targetSession->socket()) );
                    vector<uint8_t> info;
                    info.push_back(cmd); 
                    info.push_back(highByte); 
                    info.push_back(lowByte);
                    //TRACE("targetSession socket: " + to_string(targetSession->socket()) );
                    if (targetSession != nullptr) {
                        // targetSession->setTask(task);
                        ssize_t bytesSent = send(targetSession->socket(), info.data(), info.size(), 0);
                        if (bytesSent == static_cast<ssize_t>(info.size())) {
                            TRACE("Data sent successfully.");
                        } else if (bytesSent == -1) {
                            TRACE("Failed to send data. Error code: " + std::to_string(errno));
                        } else {
                            TRACE("Partial data sent. Sent bytes: " + std::to_string(bytesSent));
                        }
                    } else {
                        TRACE("session unexisted");
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        // JSON parsing failed
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
    }
}

void DataProcessor::processDronesData(const uint8_t* data, int length, Session* session) {
    TRACE("=== processDronesData === ");
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
    if(length == 0)
    {
        TRACE("socket error in process data");
        return;
    }
    int cmd = data[0];
    //TRACE("cmd :" + to_string(cmd));
    Drones* drone = new Drones();
    uint8_t* serializationData = beforeSerialization(data,length);
    if(cmd == 0x0a){
       processSNCode(serializationData, length, SQLiteConnector, drone, session);
    } else if(cmd == 0x00){
        TRACE("getHeartbeatInfo : " + drone->getSnCode());
        getHeartbeatInfo(serializationData, length, drone, session);
    }
    // } else if(length == 36){
    //     TRACE("=== upgrade ==");
    //     processUpgrade(data, length, SQLiteConnector, drone, session);
    // }
    // else if (cmd == 0x06){
    //     processTask(serializationData, length, SQLiteConnector, drone, session);
    // }
    delete[] serializationData;
    delete drone;
}

void DataProcessor::processUpgrade(const uint8_t* data, int length, Session* session){
    char snCode[24] = {0};
    memcpy(snCode, data, 24);
    char versionNumber[12] = {0};
    memcpy(versionNumber, data + 24, 12);
    TRACE("snCode : " + (string)snCode);
    TRACE("versionNumber : " + (string)versionNumber);
    char confirm = 0;
    vector<vector<string>> versionAndFilePath = querySystemVersionAndFilePath();
    if(!versionAndFilePath.empty()){
        string dbVersionNumber = versionAndFilePath[0][0];
        string dbFilePath = versionAndFilePath[0][1];
        TRACE("Queried dbVersionNumber: " + dbVersionNumber + " " + to_string(dbVersionNumber.size()));
        TRACE("Queried dbFilePath: " + dbFilePath);
        int result = strncmp(versionNumber, dbVersionNumber.c_str(), dbVersionNumber.size());
        TRACE("result: " + to_string(result));
        int compareResult = versionNumberCompareWithDB(versionNumber, dbVersionNumber);
        TRACE("compare : " +  to_string(compareResult));
        ConfigFile* mConfig = new ConfigFile("./config.txt");
        int rollBack = whetherRoolBack(mConfig);
        //int rollBack = 3;
        if(rollBack == 1){
            TRACE("need rollback");
            confirm = 2;
            send(session->socket(), &confirm, 1, 0);
        } else if(result == 0 && compareResult == 1){
            TRACE("versionNumber equal to db ");
            send(session->socket(), &confirm, 1, 0);
        } else {
            TRACE("difference to db");
            // fileProcess(dbFilePath);  
            string md5Hash = calculateMD5byOpenSSL(dbFilePath);
            // string md5Hash = "1111";
            TRACE("md5Hash " + md5Hash + " size : " +to_string(md5Hash.size()));
            sendFile(session->socket(), dbFilePath, md5Hash);
        }
        delete mConfig;
    }
   
}

int DataProcessor::whetherRoolBack(ConfigFile* mConfig){
    int rollBack = mConfig->Read("ROLLBACK", rollBack);
    TRACE("rollBack : " + to_string(rollBack));
    return rollBack;
}

vector<vector<string>> DataProcessor::querySystemVersionAndFilePath(){
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance(); 
    const char* query = "SELECT Version,FilePath FROM UpgradeManager;";
    vector<vector<string>> results;
    if (SQLiteConnector->executeQueryWithPlaceholder(query, {}, results) && results.size() > 0) {
        return results;
    } else {
        TRACE("Error executing query or no results.");
        return results;
    }
}

bool DataProcessor::versionNumberCompareWithDB(const char* versionNumber, const string dbVersionNumber){
    int dbLength = dbVersionNumber.size() + 1;
    TRACE("dbLength :" + to_string(dbLength));
    int versionLength = strlen(versionNumber);
    TRACE("versionLength :" + to_string(versionLength) );
    for(int i = dbLength; i < versionLength; i++){
        if(versionNumber[i] != '0')
        {
            return false;
        }
    }
    return true;
}

void DataProcessor::sendFile(int clientSocket, const std::string& filePath, string md5Hash) {

    ifstream file(filePath, std::ios::binary | std::ios::ate);
    //ifstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        TRACE( "Error opening file : " + filePath);
        return;
    }
    const char * path = filePath.c_str();
    struct stat fileInfo; 
    if (stat(path, &fileInfo) != 0) { 
        TRACE( "Error opening file : " ); 
    } 
    size_t fileSize = fileInfo.st_size;
    TRACE("fileSize: " + std::to_string(fileSize));
    TRACE("good1: " + std::to_string(file.good()));

    /*
        comfirm :
            UpgradePacket ：
                char filePath[256];
                uint32_t fileSize;
                char md5[32];
    */
    char comfirm = 1;
    int a = send(clientSocket, &comfirm, 1, 0);


    UpgradePacket packet;
    packet.fileSize = file.tellg();



    file.seekg(0, std::ios::beg);
    TRACE("fileSize: " + std::to_string(packet.fileSize));

    char* buffer = new char[packet.fileSize];
    file.read(buffer, packet.fileSize);

    // buffer[packet.fileSize] = '\0';



    if (file.good()) {
        TRACE("file is good !!!");
        strcpy(packet.filePath, filePath.c_str());
        //packet.fileSize = fileSize;
        strcpy(packet.md5, md5Hash.c_str());
        int upgradePacketSize = sizeof(UpgradePacket);
        TRACE("filePath: " + std::string(packet.filePath));
        TRACE("fileSize: " + std::to_string(packet.fileSize));
        TRACE("md5: " + std::string(packet.md5));
        TRACE("struct size: " + std::to_string(upgradePacketSize));
        send(clientSocket, &packet, upgradePacketSize, 0);
        TRACE("good3: " + std::to_string(file.good()));

        // int bytesRead = file.gcount();
        // TRACE("gcount: " + std::to_string(bytesRead));
        // int len = send(clientSocket, buffer, packet.fileSize, 0);
        // TRACE("len : " + to_string(len));
       

      
    } else {
        TRACE("file is bad !!!");
    }


    sendFileBy10KB(buffer, packet.fileSize, clientSocket);
    // delete[] buffer;
    file.close();
}

void DataProcessor::sendFileBy10KB(char* buffer, int bufferSize, int clientSocket) {

    int totalBytesSent = 0;
    int i = 1;
    TRACE("buffer Size :" + to_string(bufferSize)); 
    size_t bytesToSend = std::min(1024U, static_cast<unsigned int>(bufferSize));
    TRACE("bytes To Send 1:" + to_string(bytesToSend)); 
    while (totalBytesSent < bufferSize) {
        int bytesSent = send(clientSocket, buffer + totalBytesSent, bytesToSend, MSG_DONTWAIT);
        TRACE("i :" + to_string(i) +  " bytes Sent : " + to_string(bytesSent));
        if (bytesSent == -1) {
            TRACE("Error sending data: " + std::to_string(bytesSent) + " (errno: " + std::to_string(errno) + ")");
            break;
        } else {
            // TRACE("Bytes sent: " + std::to_string(bytesSent));
            totalBytesSent += bytesSent;
            TRACE("totalBytesSent " + std::to_string(totalBytesSent));
        }
        bytesToSend = std::min(1024U, static_cast<unsigned int>(bufferSize - totalBytesSent));
        i++;
        usleep(1000);
    }





    // TRACE("===  send file by 10 kb === ");
    // char buffer[BUFFER_SIZE];
    // TRACE("buffer: " + to_string(sizeof(buffer)));
    // TRACE("file good : " + std::to_string(file.good()));
    // TRACE("eof : " + to_string(file.eof()));
    // if (!file) {
    //     TRACE("Error opening or reading the file.");
    //     return;
    // }
    // file.seekg(0, std::ios::beg);
    // file.read(buffer, BUFFER_SIZE);
    // if (file.peek() == ifstream::traits_type::eof()) {
    //     TRACE("File is empty.");
    //     return;
    // }   
    // TRACE("eof : " + to_string(file.eof()));
    // TRACE("gcount 1: " + to_string(file.gcount()));
    // while (file.read(buffer, BUFFER_SIZE)) {
    //     int bytesRead = file.gcount();
    //     TRACE("gcount: " + std::to_string(bytesRead));
    //     if (bytesRead > 0) {
    //         int bytesSent = send(clientSocket, buffer, bytesRead, 0);
    //         if (bytesSent == -1) {
    //             TRACE(" Error sending data:" + to_string(bytesSent));
    //             break;
    //         }
    //         else 
    //             TRACE("bytesSent :" + to_string(bytesSent));
    //         for (int i = 0; i < 10; i++) {
    //             std::cout << std::hex << (uint)(buffer[i]) << " ";
    //         }
    //         std::cout << std::endl;
    //     }
    // }

}

void DataProcessor::getTaskInfo(const uint8_t* data, int length, Session* session)
{
   TRACE("=== get TaskInfo === ");
   Gps gps;
   //Tasks* task= new Tasks();
    try{
        if(length > 3){
            uint8_t* data1 =  beforeSerialization(data,length);
            Tasks* task = session->getTask();
            vector<WaypointInformation> waypointVec;
            int submissionID = stoi(task->getSubmissionID());
            TRACE("submissionID in getTaskInfo: " + to_string(submissionID));

            // std::stringstream gpsInfo;
            // gpsInfo.write((const char*)data1, length);
            // UnSerialization(gpsInfo,gps);
            // TRACE("gps.stc_lon: " + to_string(gps.stc_lon));
            // TRACE("gps.stc_lat: " + to_string(gps.stc_lat));
            // TRACE("gps.stc_alt: " + to_string(gps.stc_alt));

            gps.stc_lon = 105.8937155;
            gps.stc_lat = 29.267501;
            gps.stc_alt = 289.872;

            WaypointInformation m_rtkhome_point;
        
            m_rtkhome_point.stc_x = gps.stc_lon;
	        m_rtkhome_point.stc_y = gps.stc_lat;
	        m_rtkhome_point.stc_z = gps.stc_alt;


            TowerList firstTower = getFirstTower(submissionID);
            TRACE("towerID : " + to_string(firstTower.towerID));
            TRACE("towerName : " + firstTower.towerName);
            //TRACE("dz : " + to_string(dz));

            LongitudeAndLatitude current_drone_position;
	        current_drone_position.stc_altitude = m_rtkhome_point.stc_z;
	        current_drone_position.stc_longitude = m_rtkhome_point.stc_x;
	        current_drone_position.stc_latitude = m_rtkhome_point.stc_y;

            CDatumPoint datumPoint;
	        datumPoint.setOrigin(firstTower.init_longitude, firstTower.init_latitude, firstTower.init_altitude, true);

            double dx = datumPoint.longitudeAndLatitudeToDatumPoint(current_drone_position).stc_x;
	        double dy = datumPoint.longitudeAndLatitudeToDatumPoint(current_drone_position).stc_y;
	        double dz = datumPoint.longitudeAndLatitudeToDatumPoint(current_drone_position).stc_z;

            TRACE("dx : " + to_string(dx));
            TRACE("dy : " + to_string(dy));
            TRACE("dz : " + to_string(dz));

            int missionType = stoi(task->getMissionType());
	        switch (missionType)
            {
                //踩点任务
                // case TASKTYPENOTUPLOADED:
                // case TASKTYPEDISTRIBUTIONDOT:
                // //获取航线数据接口
                // break;
                // //精细航线生成任务
                // case TASKTYPEROUTESGENERATE:
                //     m_data::Getm_data()->Get_Generate_fine_route_task_UploadToAircraft(m_vecDrone[index].submissionID, m_rtkhome_point);
                //     break;
                case TASKTYPEDISTRIBUTIONROUTE:
                //获取航线数据接口
                    waypointVec = generateMinorAirline(submissionID, m_rtkhome_point, task->getFlightType());
                    break;
                //复飞任务
                // case TASKTYPEROUTESPATROL:
                // case TASKTYPEIMPORTROUTE:
                //     m_data::Getm_data()->Get_Read_data_from_file(m_vecDrone[index].submissionID);
                //     break;
                default:
                    break;
            }
       
            if (waypointVec.size() < 2)
            {
                TRACE("data error ");
                return;
            }
            TRACE("finished generateMinorAirline");

            vector<WaypointInformation> commute_task = waypointVec;

            for(auto item : waypointVec){
            
              TRACE(to_string(item.stc_x) + " , " + to_string(item.stc_y) + " , " +to_string(item.stc_z) + " , " +
              to_string(item.POI_X)  + " , " + to_string(item.POI_Y) +" , " + to_string(item.POI_Z)+
              + " , " + to_string(item.CONFIG_NO)+ " , " +to_string(item.maxFlightSpeed) + " , " + to_string(item.gimbal_roll) );


              TRACE("FROM_PC_TXZF_WP_POI_X:  " + to_string(item.POI_X) + " FROM_PC_TXZF_WP_POI_Y = " + to_string(item.POI_Y) + " FROM_PC_TXZF_WP_POI_Z = " 
                +  to_string(item.POI_Z) + " FROM_PC_TXZF_WP_tower_id = " + to_string(item.tower_id)  + ", FROM_PC_pitch = " + to_string(item.stc_gimbalPitch) 
                + " FROM_PC_yaw = " + to_string(item.stc_yaw) + " FROM_PC_shotphoto = " +  to_string(item.stc_shootPhoto)  + ", FROM_PC_PHOTOTIME = " + item.Photo_time
                + " FROM_PC_target_class_id = " + to_string(item.target_class_id[0]) + " FROM_PC_TXZF_WP_necessary = " +  to_string(item.necessary)
                + " FROM_PC_ZoomFocalLength = " + to_string(item.zoom_Magnification) + ", FROM_PC_TXZF_WP_CONFIG_NO = " + to_string(item.CONFIG_NO)
                + " FROM_PC_taskid = " + to_string(item.task_id));

               TRACE( " FROM_PC_Hightprecision_latitude = " +  to_string(item.stc_y) + " FROM_PC_Hightprecision_longitude = " 
               + to_string(item.stc_x) +" FROM_PC_Hightprecision_relativeHeight = " + to_string(item.stc_z));

            }
            TRACE("===================");
            double d_longitude = 0;
	        double d_latitude = 0;
	        float d_height = 0;
            int doublephoto = GetDoublePhotoBySubmissionID(submissionID);
            for (int j = 0; j < commute_task.size(); j++) {
                /*if (basicType == 1)
                {
                    commute_task[j].stc_x -= d_longitude;
                    commute_task[j].stc_y -= d_latitude;
                }*/
                commute_task[j].stc_z -= d_height;
                if (doublephoto == 1)
                {
                    commute_task[j].Photo_time = "1";//复用Photo_time 此处代表两端拍照
                }
                else if (doublephoto == 0)
                {
                    commute_task[j].Photo_time = "0";
                }
	        }
            //复飞操作 
            /*
                code not add 
            */
            list<WaypointInformation> commute_task_list;
            TRACE("commute_task size : " + to_string(commute_task.size()));
	        for (int i = 0; i < commute_task.size(); i++)
	        {
		        commute_task_list.push_back(commute_task[i]);
	        }
            TRACE("commute_task_list size : " + to_string(commute_task_list.size()));
            vector<uint8_t> info;

            string realData;

            uint8_t cmd = 0x02;

            realData = cmd;

            
            info.push_back(cmd);

            // auto serializedData = ss.str();
            // TRACE("serializedData : " + to_string(serializedData.size()));
            // auto serializedWaypointData = Waypoint.str();
            // TRACE("serializedWaypointData : " + to_string(serializedWaypointData.size()));
            // size_t dataLength = serializedData.size() + serializedWaypointData.size();
            // uint8_t highByte = (dataLength >> 8) & 0xFF;
            // uint8_t lowByte = dataLength & 0xFF;

            TaskInfo taskInfo;
            taskInfo.flight = task->getFlightType();
            TRACE("taskInfo.flight :" + to_string(taskInfo.flight));
            taskInfo.rtk = task->getRtk();
            TRACE("taskInfo.rtk :" + to_string(taskInfo.rtk));
            taskInfo.record = task->getWhetherRecord();
            TRACE("taskInfo.record :" + to_string(taskInfo.record));
            taskInfo.timestamp = task->getTimestamp();
            TRACE("taskInfo.timestamp :" + to_string(taskInfo.timestamp));
            taskInfo.imgwidth = task->getImgwidth();
            TRACE("taskInfo.imgwidth :" + to_string(taskInfo.imgwidth));
            taskInfo.imgheight = task->getImgheigth();
            TRACE("taskInfo.imgheight :" + to_string(taskInfo.imgheight));
            int extralen = sizeof(taskInfo.rtk) + sizeof(taskInfo.flight) + sizeof(taskInfo.timestamp) + sizeof(taskInfo.record) + sizeof(taskInfo.imgwidth) + sizeof(taskInfo.imgheight);
            taskInfo.returnCode = task->getReturnCode();
            TRACE("taskInfo.returnCode :" + to_string(taskInfo.returnCode));
            if(task->getReturnCode() == 254){
               
                taskInfo.Gps = task->getReturnGPS();

                TRACE("stc_lon : " + to_string(taskInfo.Gps.stc_lon));
                TRACE("stc_lat : " + to_string(taskInfo.Gps.stc_lat));
                TRACE("stc_alt : " + to_string(taskInfo.Gps.stc_alt));
            
                extralen += sizeof(taskInfo.returnCode) + sizeof(taskInfo.Gps);
                TRACE("return code == 254 : " + to_string(extralen));
            } else {
                extralen += sizeof(taskInfo.returnCode);
                TRACE("return code != 254 : " + to_string(extralen));
            }
             
            // taskInfo.waypoints = commute_task_list;
            // stringstream taskData;
            // Serialization(taskData,taskInfo);
            // auto serializedData1 = taskData.str();
            // TRACE("serializedData : " + to_string(serializedData1.size()));

            // std::stringstream ss;
            // cereal::BinaryOutputArchive oarchive(ss);
            // oarchive(commute_task_list);

            std::stringstream ss;
            // {
            //     cereal::BinaryOutputArchive archive(ss);
            //     archive(commute_task_list);
            // }
            TRACE("begin Serialization  waypoint");
            Serialization(ss, commute_task_list);
            std::string serializedData = ss.str();
            TRACE("serializedData : " + to_string(serializedData.size()));

            // TaskInfo taskInfo;
            // taskInfo.waypoints = commute_task_list;
            // stringstream waypoint;
            // Serialization(waypoint,taskInfo.waypoints);
            // auto serializedData = waypoint.str();

           
            uint16_t data_len = serializedData.size() + extralen;
            TRACE("data_len : " + to_string(data_len));
            realData.resize(realData.size() + sizeof(data_len) + extralen);

            // TRACE("rtk " +  to_string(taskInfo.rtk) + " size :" + to_string(sizeof(taskInfo.rtk)));
            // TRACE("flight " +  to_string(taskInfo.flight) + " size :" + to_string(sizeof(taskInfo.flight)));
            // TRACE("timestamp " +  to_string(taskInfo.timestamp) + " size :" + to_string(sizeof(taskInfo.timestamp)));
            // TRACE("record " +  to_string(taskInfo.record) + " size :" + to_string(sizeof(taskInfo.record)));
            // TRACE("imgwidth " +  to_string(taskInfo.imgwidth) + " size :" + to_string(sizeof(taskInfo.imgwidth)));
            // TRACE("imgheight " +  to_string(taskInfo.imgheight) + " size :" + to_string(sizeof(taskInfo.imgheight)));
            // uint16_t data_len = serializedData.size() + extralen;
            // TRACE("extralen " +  to_string(extralen));
            // TRACE("data_len " +  to_string(data_len));
            // size_t dataLength = serializedData.size();

            uint16_t bigEndianValue =  ((data_len & 0xFF00) >> 8) | ((data_len & 0x00FF) << 8);
            int len = sizeof(cmd);
	        memcpy((char*)realData.data() + len, &bigEndianValue, sizeof(bigEndianValue));
            len += sizeof(bigEndianValue);
	        memcpy((char*)realData.data() + len, (char*)&taskInfo.rtk, sizeof(taskInfo.rtk));
	        len += sizeof(taskInfo.rtk);
	        memcpy((char*)realData.data() + len, (char*)&taskInfo.flight, sizeof(taskInfo.flight));
	        len += sizeof(taskInfo.flight);
	        memcpy((char*)realData.data() + len, (char*)&taskInfo.timestamp, sizeof(taskInfo.timestamp));
	        len += sizeof(taskInfo.timestamp);
	        memcpy((char*)realData.data() + len, (char*)&taskInfo.record, sizeof(taskInfo.record));
	        len += sizeof(taskInfo.record);
	        memcpy((char*)realData.data() + len, (char*)&taskInfo.imgwidth, sizeof(taskInfo.imgwidth));
	        len += sizeof(taskInfo.imgwidth);
	        memcpy((char*)realData.data() + len, (char*)&taskInfo.imgheight, sizeof(taskInfo.imgheight));
            len += sizeof(taskInfo.imgheight);
            memcpy((char*)realData.data() + len, (char*)&taskInfo.returnCode, sizeof(taskInfo.returnCode));
            len += sizeof(taskInfo.returnCode);
            if(taskInfo.returnCode == 254)
            {
                memcpy((char*)realData.data() + len, (char*)&taskInfo.Gps, sizeof(taskInfo.Gps));
            }
            copy(serializedData.begin(), serializedData.end(), std::back_inserter(realData));
            ssize_t bytesSent =  send(session->socket(), realData.data(), realData.size(), 0);

            // uint8_t highByte = (bigEndianValue >> 8) & 0xFF;
            // TRACE("highByte " +  to_string(highByte));
            // uint8_t lowByte = bigEndianValue & 0xFF;
            // TRACE("lowByte " +  to_string(lowByte));
            // info.push_back(lowByte);
            // info.push_back(highByte);

            // info.insert(info.end(), reinterpret_cast<uint8_t*>(&taskInfo.rtk), reinterpret_cast<uint8_t*>(&taskInfo.rtk) + sizeof(taskInfo.rtk));
            // info.insert(info.end(), reinterpret_cast<uint8_t*>(&taskInfo.flight), reinterpret_cast<uint8_t*>(&taskInfo.flight) + sizeof(taskInfo.flight));
            // info.insert(info.end(), reinterpret_cast<uint8_t*>(&taskInfo.timestamp), reinterpret_cast<uint8_t*>(&taskInfo.timestamp) + sizeof(taskInfo.timestamp));
            // info.insert(info.end(), reinterpret_cast<uint8_t*>(&taskInfo.record), reinterpret_cast<uint8_t*>(&taskInfo.record) + sizeof(taskInfo.record));
            // info.insert(info.end(), reinterpret_cast<uint8_t*>(&taskInfo.imgwidth), reinterpret_cast<uint8_t*>(&taskInfo.imgwidth) + sizeof(taskInfo.imgwidth));
            // info.insert(info.end(), reinterpret_cast<uint8_t*>(&taskInfo.imgheight), reinterpret_cast<uint8_t*>(&taskInfo.imgheight) + sizeof(taskInfo.imgheight));
            // info.insert(info.end(), serializedData.begin(), serializedData.end());

            // string infoString(info.begin(), info.end());
            // ssize_t bytesSent = send(session->socket(), infoString.c_str(), infoString.size(), 0);

            // ssize_t bytesSent =  send(session->socket(), info.data(), info.size(), 0);

            if (bytesSent == realData.size()) {
                TRACE("Data sent successfully. " +  to_string(bytesSent));
            } else if (bytesSent == -1) {
                TRACE("Failed to send data. Error code: " + std::to_string(errno));
            } else {
                TRACE("Partial data sent. Sent bytes: " + std::to_string(bytesSent));
            }
            ss.str("");
            ss.clear();
            for (const uint8_t byte : realData) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }

        } else {
            TRACE("data format error" );
        }
    }
    catch(exception& e){
        TRACE("getTaskInfo error");
    }
   // return task;
}

void DataProcessor::processTask(const uint8_t* data, int length, SQLiteConnection* SQLiteConnector, Drones* drone, Session* session)
{
    // Tasks* taskInfo = getTaskInfo(data,length, session);
    // if(taskInfo != nullptr)
    // {
    //     TRACE("infor not null ");
    //     //delete taskInfo;
    // }
}

void DataProcessor::processSNCode(const uint8_t* data, int length, SQLiteConnection* SQLiteConnector, Drones* drone, Session* session){
    DroneData droneData = getDroneSNCode(data,length);
    TRACE("session in processSNCode :" + to_string(session->socket()));
    string snCode = droneData.snCode;
    string version = droneData.version;
    float maxSpeed = droneData.maxSpeed;
    string model = droneData.model;
    string lens = droneData.lens;
    TRACE("SnCode : " + snCode);
    TRACE("version : " + version);
    TRACE("maxSpeed : " + to_string(maxSpeed));
    TRACE("model : " + model);
    TRACE("lens : " + lens);
    drone->setSnCode(snCode);
    drone->setDroneType(model);
    drone->setMaxSpeed(to_string(maxSpeed));
    drone->setSystemVersion(version);
    drone->setLen(lens);
    TRACE("drone snCode : " + drone->getSnCode()); 
    TRACE("socket : " + to_string(session->socket())); 
    SessionManager *m_sessionManager = SessionManager::GetInstance(); 
    m_sessionManager->insertToDroneSessionMap(snCode, session);
    // m_sessionManager->insertToDroneSessionMap1(drone,session);
    // droneSessionMap[snCode] = session;
    // droneSessionMap.insert(make_pair(snCode, session)); 
    string result;
    string selectQuery = "SELECT COUNT(*) FROM drone WHERE snCode = '" + drone->getSnCode() + "';";
    TRACE("selectQuery : " + selectQuery);
    if (SQLiteConnector->executeScalar(selectQuery, result)) {
        int count = stoi(result);
        if (count == 0) {
            string insertQuery = "INSERT INTO drone (snCode, systemVersion, maxSpeed, droneType, lensType) VALUES (?, ?, ?, ?, ?);";
            vector<string> placeholders = {
                drone->getSnCode(),
                version,
                to_string(maxSpeed),
                model,
                lens
            };
            if (SQLiteConnector->executeDroneInsertQuery(insertQuery, placeholders)) {
                TRACE("Insertion successful");
            } else {
                TRACE("Insertion failed");
            }
        } else {
            TRACE("Duplicate snCode found, insertion skipped");
        }
    } else {
        TRACE("Database query COUNT(*) failed");
    }
}   

DroneData DataProcessor::getDroneSNCode(const uint8_t* data, int length){
    TRACE("get snCode");
    //string SnCode; 
    DroneData droneData;
    try{
        if(length > 3){
            std::stringstream ss;
            ss.write((const char*)data, length);
            UnSerialization(ss,droneData);
            TRACE("FINISHED");
            //string snCode;
            string snCode = droneData.snCode;
            string version = droneData.version;
            float maxSpeed = droneData.maxSpeed;
            string model = droneData.model;
            string lens = droneData.lens;
            TRACE("SnCode : " + snCode);
            TRACE("version : " + version);
            TRACE("maxSpeed : " + to_string(maxSpeed));
            TRACE("model : " + model);
            TRACE("lens : " + lens);
            ss.str("");
            ss.clear();
            //return droneData;
        } else {
            TRACE("DroneSNCode data format error" );
            //SnCode = "error";
           // return droneData;
        }
    }
    catch(exception& e){
        TRACE("UnSerialization snCode error");
       // SnCode = "error";
        //return DroneData;
    }
     return droneData;
}

HeartbeatInfo DataProcessor::getHeartbeatInfo(const uint8_t* data, int length, Drones* drone, Session* session){
    TRACE(" ==== getHeartbeatInfo ===");
    HeartbeatInfo heartbeatInfo;
    TRACE("drone->getSnCode() : " + drone->getSnCode());
    TRACE("session in getHeartbeatInfo:" + to_string(session->socket()));
    try{
        if(length > 3){
            std::stringstream ss;
            ss.write((const char*)data, length);
            UnSerialization(ss,heartbeatInfo);
            TRACE("FINISHED");
            double longitude = heartbeatInfo.longitude;
            double latitude = heartbeatInfo.latitude;
            double altitude = heartbeatInfo.altitude;
            int missionID = heartbeatInfo.missionID;
            uint8_t status = heartbeatInfo.status;
            uint32_t index = heartbeatInfo.index;
            uint32_t count = heartbeatInfo.count;
            heartbeatInfo.isInitialized = true; 
            TRACE("longitude : " + to_string(longitude));
            TRACE("latitude : " + to_string(latitude));
            TRACE("altitude : " + to_string(altitude));
            TRACE("missionID : " + to_string(missionID));
            TRACE("status : " + to_string(status));
            TRACE("index : " + to_string(index));
            TRACE("count : " + to_string(count));
            SessionManager *m_sessionManager = SessionManager::GetInstance(); 
            m_sessionManager->insertToHeartbeatInfoSessionMap(session, heartbeatInfo);
            vector<uint8_t> info;
            info.push_back(0x00);
            info.push_back(0x00);
            info.push_back(0x00);
    
            ssize_t bytesSent =  send(session->socket(), info.data(), info.size(), 0);
            if (bytesSent == info.size()) {
                TRACE("heartbeat sent successfully. " +  to_string(bytesSent));
            } else if (bytesSent == -1) {
                TRACE("Failed to send data. Error code: " + std::to_string(errno));
            } else {
                TRACE("Partial data sent. Sent bytes: " + std::to_string(bytesSent));
            }
            ss.str("");
            ss.clear();
        } else {
            TRACE("HeartbeatInfo format error" );
        }
    }
    catch(exception& e){
        TRACE("UnSerialization HeartbeatInfo error");
    }
    return heartbeatInfo;
}


// bool DataProcessor::updateTaskStatus(HeartbeatInfo heartbeatInfo){
//     SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
//     if(heartbeatInfo.missionID == -1){
//         string query = "SELECT towerID, lineID, height, longitude, latitude, altitude, insulatorNum, towerShapeID, towerName, basicID FROM submission_tower_missiontype WHERE submissionID = ?";
//         vector<string> placeholders;
//         placeholders.push_back(to_string(submission_id));
//     } else {
//         string query = "SELECT towerID, lineID, height, longitude, latitude, altitude, insulatorNum, towerShapeID, towerName, basicID FROM submission_tower_missiontype WHERE submissionID = ?";
//     }
// }


vector<WaypointInformation> DataProcessor::GetSubmissionWaypoints(int submissionID)
{
    TRACE("==== GetSubmissionWaypoints ==== ");
	std::vector<BasicResultGet> tower_data;
	std::vector<AirlineTower> m_towerInfoForInitialRoute = Query_submission_towerinfo(submissionID);
	if (m_towerInfoForInitialRoute.size() < 3)
	{
		return std::vector<WaypointInformation>();
	}
	int missionType = Query_submisson_missionID(submissionID);
	std::vector<int> flagVec;//存储每个杆塔的photoFlag
	for (int i = 0; i < m_towerInfoForInitialRoute.size(); i++)
	{
		if (m_towerInfoForInitialRoute[i].stc_towerId == -1)
		{
			BasicResultGet temp;
			Gps temp_gps;
			temp_gps.stc_alt = 0;
			temp_gps.stc_lat = 0;
			temp_gps.stc_lon = 0;
			temp.stc_basicResult.stc_vecGps.push_back(temp_gps);
			temp.stc_basicResult.stc_vecGps.push_back(temp_gps);
			temp.stc_basicResult.stc_vecGps.push_back(temp_gps);
			tower_data.push_back(temp);
			flagVec.push_back(0);
			continue;
		}

		if (i % 3 == 0 && (missionType == TASKTYPEROUTESGENERATE || missionType == TASKTYPEROUTESPATROL) && m_towerInfoForInitialRoute[i].stc_path.empty()) {
			//pop_Message::Getpop_Message()->signal_Show_Message(QString("杆塔存在非主网打点！"), true, false);
            TRACE("tower point error");
		    return std::vector<WaypointInformation>();
		}

		BasicResultGet p_tower_data;
		if (i % 3 == 0 && (missionType == TASKTYPEROUTESGENERATE || missionType == TASKTYPEROUTESPATROL)) {
            TRACE("read file ");
			// string data = "";
			// m_cruiseInterface.Cruise_Read_File(m_towerInfoForInitialRoute[i].stc_path, data);
			// p_tower_data = m_cruiseJson.GetBasicResult(data.toUtf8());
		}
		else {
			p_tower_data.stc_isSuccess = true;
			Gps tempGps;
			tempGps.stc_lon = m_towerInfoForInitialRoute[i].stc_x;
			tempGps.stc_lat = m_towerInfoForInitialRoute[i].stc_y;
			tempGps.stc_alt = m_towerInfoForInitialRoute[i].stc_z;
			p_tower_data.stc_basicResult.stc_vecGps.push_back(tempGps);//这里暂时还是沿用以前的逻辑，但是配网实际只要一个点了
			p_tower_data.stc_basicResult.stc_vecGps.push_back(tempGps);
			p_tower_data.stc_basicResult.stc_vecGps.push_back(tempGps);
		}
		p_tower_data.stc_errCode = m_towerInfoForInitialRoute[i].height;//借用stc_errCode表示高度
		if (m_towerInfoForInitialRoute[i].towerVirtual == 2) {
			p_tower_data.stc_errMsg = "2";
		}
		else if (m_towerInfoForInitialRoute[i].towerVirtual == 1) {
			p_tower_data.stc_errMsg = "1";
		}
		else {
			p_tower_data.stc_errMsg = "0";
		}
		tower_data.push_back(p_tower_data);
		int photoflag = get_photoPosition(m_towerInfoForInitialRoute[i].stc_towerId);
		if (m_towerInfoForInitialRoute[i].pass == 0) {
			photoflag |= 0b10000;
		}
		flagVec.push_back(photoflag);
	}
    TRACE("tower_data size : " + to_string(tower_data.size()));
	int increasehigh = 0;
	vector<WaypointInformation> vecWaypoint;
	for (int i = 0; i < tower_data.size(); ) {
        TRACE(" i :" + to_string(i) + " , towerVirtual : " + to_string(m_towerInfoForInitialRoute[i].towerVirtual));
		if (m_towerInfoForInitialRoute[i].towerVirtual == 0) { //正常塔（tower_data 和m_towerInfoForInitialRoute一一对应）
			if (i + 2 >= tower_data.size()) {
				// //pop_Message::Getpop_Message()->signal_Show_Message(QString("杆塔数据异常！"), true, false);
				// m_cruiseInterface.Cruise_Write_File(QDir::currentPath() + "/temp_initial_route.pcd", write_error_pcl_file());
				return std::vector<WaypointInformation>();
		        }

			RecPoint temprecPoint;
			DatumPoint tmpdp;
			tmpdp.stc_x = tower_data[i].stc_basicResult.stc_vecGps[0].stc_lon;
			tmpdp.stc_y = tower_data[i].stc_basicResult.stc_vecGps[0].stc_lat;
			tmpdp.stc_z = tower_data[i].stc_basicResult.stc_vecGps[0].stc_alt;
			temprecPoint.rec_pt[0] = tmpdp;
			if (missionType == TASKTYPEROUTESGENERATE || missionType == TASKTYPEROUTESPATROL) {
				tmpdp.stc_x = tower_data[i].stc_basicResult.stc_vecGps[1].stc_lon;
				tmpdp.stc_y = tower_data[i].stc_basicResult.stc_vecGps[1].stc_lat;
				tmpdp.stc_z = tower_data[i].stc_basicResult.stc_vecGps[1].stc_alt;
				temprecPoint.rec_pt[1] = tmpdp;
				tmpdp.stc_x = tower_data[i].stc_basicResult.stc_vecGps[2].stc_lon;
				tmpdp.stc_y = tower_data[i].stc_basicResult.stc_vecGps[2].stc_lat;
				tmpdp.stc_z = tower_data[i].stc_basicResult.stc_vecGps[2].stc_alt;
				temprecPoint.rec_pt[2] = tmpdp;
			}
			else {
				temprecPoint.rec_pt[1] = tmpdp;
				temprecPoint.rec_pt[2] = tmpdp;
			}

			DatumPoint prev_temp;
			if (m_towerInfoForInitialRoute[i + 1].stc_towerId != -1) {
				prev_temp.stc_x = tower_data[i + 1].stc_basicResult.stc_vecGps[0].stc_lon;
				prev_temp.stc_y = tower_data[i + 1].stc_basicResult.stc_vecGps[0].stc_lat;
				prev_temp.stc_z = tower_data[i + 1].stc_basicResult.stc_vecGps[0].stc_alt;
			}
			else
			{
				prev_temp.stc_x = 0.0;
				prev_temp.stc_y = 0.0;
				prev_temp.stc_z = 0.0;
			}
			temprecPoint.prev_pt = prev_temp;

			DatumPoint next_temp;
			if (m_towerInfoForInitialRoute[i + 2].stc_towerId != -1) {
				next_temp.stc_x = tower_data[i + 2].stc_basicResult.stc_vecGps[0].stc_lon;
				next_temp.stc_y = tower_data[i + 2].stc_basicResult.stc_vecGps[0].stc_lat;
				next_temp.stc_z = tower_data[i + 2].stc_basicResult.stc_vecGps[0].stc_alt;
			}
			else
			{
				next_temp.stc_x = 0.0;
				next_temp.stc_y = 0.0;
				next_temp.stc_z = 0.0;
			}
			temprecPoint.next_pt = next_temp;

			temprecPoint.layer_num = m_towerInfoForInitialRoute[i].stc_insulator_layer;
			temprecPoint.left_angle = m_towerInfoForInitialRoute[i].stc_leftAngle;
			temprecPoint.right_angle = m_towerInfoForInitialRoute[i].stc_rightAngle;
			temprecPoint.yaw_rotate_angle_selected = m_towerInfoForInitialRoute[i].stc_offCenterAngle;
			temprecPoint.height = m_towerInfoForInitialRoute[i].stc_height;
			temprecPoint.ChannelInspection = m_towerInfoForInitialRoute[i].stc_ChannelInspection;
			temprecPoint.zoom = m_towerInfoForInitialRoute[i].zoom;
            TRACE("tower id:  " + to_string(m_towerInfoForInitialRoute[i].stc_towerId));
			int m_lineTypeID = Query_lintTypeID(m_towerInfoForInitialRoute[i].stc_towerId);
			//std::vector<RouteIndexInfo> temp_vec_routeIndexInfo;
			std::vector<WaypointInformation> temp_vec_routeIndexInfo_WaypointInformation;
			//= m_pointToLine.pointToLine_u1(temprecPoint);
            // TRACE("missionType :  " + to_string(missionType));
			if (missionType == TASKTYPEDISTRIBUTIONROUTE) {
                TRACE("missionType :  " + to_string(missionType));
				temp_vec_routeIndexInfo_WaypointInformation = m_point_to_line_v3.pointToLine_minor(temprecPoint, flagVec[i]);
			}
			else if (m_lineTypeID == 0)
			{
                TRACE(" m_lineTypeID == 0  ");
				temp_vec_routeIndexInfo_WaypointInformation = m_point_to_line_v3.pointToLine_U(temprecPoint);
			}
			else if (m_lineTypeID == 1)
			{
                TRACE(" m_lineTypeID == 1 ");
				temp_vec_routeIndexInfo_WaypointInformation = m_point_to_line_v3.pointToLine_OU(temprecPoint);
			}
            //TRACE("temp_vec_routeIndexInfo_WaypointInformation :  " + to_string(temp_vec_routeIndexInfo_WaypointInformation.size()));
			if (temp_vec_routeIndexInfo_WaypointInformation.size() == 1 && temp_vec_routeIndexInfo_WaypointInformation[0].stc_index == 0xFFFF)//stc_index为-1，因为是无符号数，改下判断条件
			{
                TRACE("1 benchmark data does not meet standards");
				//此时为假
				//pop_Message::Getpop_Message()->signal_Show_Message(QString("基准点数据不符合标准"), true, false);
				return std::vector<WaypointInformation>();
			}
			if (temp_vec_routeIndexInfo_WaypointInformation.size() == 0)
			{
                TRACE("2 benchmark data does not meet standards");
				//此时为假
				//m_cruiseInterface.Cruise_Write_File(QDir::currentPath() + "/temp_initial_route.pcd", write_error_pcl_file());
				//pop_Message::Getpop_Message()->signal_Show_Message(QString("图像展示失败\n基准点不符合标准"), true, false);
				return std::vector<WaypointInformation>();
			}
            TRACE("temp_vec_routeIndexInfo_WaypointInformation size :" + to_string(temp_vec_routeIndexInfo_WaypointInformation.size()));
			for (int a = 0; a < temp_vec_routeIndexInfo_WaypointInformation.size(); a++)
			{
				temp_vec_routeIndexInfo_WaypointInformation[a].tower_id = m_towerInfoForInitialRoute[i].stc_towerId;
				temp_vec_routeIndexInfo_WaypointInformation[a].Datum_point = tower_data[i].stc_basicResult.stc_vecGps;
				temp_vec_routeIndexInfo_WaypointInformation[a].task_id = submissionID;
			}
			if (increasehigh != 0) {
				WaypointInformation wptmp = temp_vec_routeIndexInfo_WaypointInformation[0];
				wptmp.stc_shootPhoto = 0;
				wptmp.stc_z += increasehigh;
				wptmp.tower_id = -1;
				wptmp.task_id = submissionID;
				wptmp.stc_gimbalPitch = -90;
				wptmp.necessary = true;
				temp_vec_routeIndexInfo_WaypointInformation.insert(temp_vec_routeIndexInfo_WaypointInformation.begin(), wptmp);
				increasehigh = 0;
			}
            TRACE("increasehigh != 0 temp_vec_routeIndexInfo_WaypointInformation size :" + to_string(temp_vec_routeIndexInfo_WaypointInformation.size()));
			if (tower_data[i].stc_errCode != 0)
			{
				int inx = temp_vec_routeIndexInfo_WaypointInformation.size() - 1;
				WaypointInformation wptmp = temp_vec_routeIndexInfo_WaypointInformation[inx];
				wptmp.stc_shootPhoto = 0;
				wptmp.stc_z += tower_data[i].stc_errCode;
				for (int a = 0; a < temp_vec_routeIndexInfo_WaypointInformation.size(); a++) {
					if (temp_vec_routeIndexInfo_WaypointInformation[a].necessary == true) {
						wptmp.stc_yaw = temp_vec_routeIndexInfo_WaypointInformation[a].stc_yaw;
						break;
					}
				}
				wptmp.stc_gimbalPitch = -90;
				wptmp.tower_id = -1;
				wptmp.task_id = submissionID;
				wptmp.necessary = true;
				temp_vec_routeIndexInfo_WaypointInformation.push_back(wptmp);
				increasehigh = tower_data[i].stc_errCode;
			}
            TRACE("tower_data[i].stc_errCode != 0 temp_vec_routeIndexInfo_WaypointInformation size :" + to_string(temp_vec_routeIndexInfo_WaypointInformation.size()));
			vecWaypoint.insert(vecWaypoint.end(), temp_vec_routeIndexInfo_WaypointInformation.begin(), temp_vec_routeIndexInfo_WaypointInformation.end());
            TRACE("in process vecWaypoint.size  : " + to_string(vecWaypoint.size()));
			i += 3;
		}
		else if (m_towerInfoForInitialRoute[i].towerVirtual == 1) {//虚拟塔
			WaypointInformation virtualwaypoint;
			virtualwaypoint.stc_x = m_towerInfoForInitialRoute[i].stc_x;
			virtualwaypoint.stc_y = m_towerInfoForInitialRoute[i].stc_y;
			virtualwaypoint.stc_z = m_towerInfoForInitialRoute[i].stc_z;
			virtualwaypoint.stc_shootPhoto = 0;
			virtualwaypoint.tower_id = m_towerInfoForInitialRoute[i].stc_towerId;
			virtualwaypoint.task_id = submissionID;
			virtualwaypoint.target_class_id.push_back(12);
			virtualwaypoint.necessary = true;
			if (increasehigh != 0) {
				WaypointInformation wptmp = virtualwaypoint;
				wptmp.tower_id = -1;
				wptmp.stc_z += increasehigh;
				wptmp.stc_gimbalPitch = -90;
				vecWaypoint.push_back(wptmp);
				increasehigh = 0;
			}
			vecWaypoint.push_back(virtualwaypoint);
			if (tower_data[i].stc_errCode != 0)
			{
				WaypointInformation wptmp = virtualwaypoint;
				wptmp.tower_id = -1;
				wptmp.stc_z += tower_data[i].stc_errCode;
				wptmp.stc_gimbalPitch = -90;
				vecWaypoint.push_back(wptmp);
				increasehigh = tower_data[i].stc_errCode;
			}
			i++;
		}
		else
		{
			i++;
		}
	}
    TRACE("vecWaypoint.size  : " + to_string(vecWaypoint.size()));
	return vecWaypoint;
}

vector<AirlineTower> DataProcessor::Query_submission_towerinfo(int submission_id)
{
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
    TRACE("Query_submission_towerinfo");
    std::vector<AirlineTower> vecTowerInfoForInitialRoute;
    //db submission_tower_missiontype without mode字段
    std::string query = "SELECT towerID, lineID, height, longitude, latitude, altitude, insulatorNum, towerShapeID, towerName, basicID FROM submission_tower_missiontype WHERE submissionID = ?";
    std::vector<std::string> placeholders;
    placeholders.push_back(std::to_string(submission_id));
    std::vector<std::vector<std::string>> queryResults;
    if (SQLiteConnector->executeQueryWithPlaceholder(query, placeholders, queryResults)) {
        for (const auto& row : queryResults) {
            AirlineTower temp; 
            TRACE("ROW SIZE : " + to_string(row.size()));    
            //towerID 0, lineID 1, height 2,
            //longitude 3 , latitude 4, altitude 5, 
            //insulatorNum 6,  towerShapeID 7, towerName 8, basicID 9
            temp.stc_towerId = std::stoi(row[0]);
            TRACE("temp.stc_towerId : " + to_string(temp.stc_towerId));
            temp.lineID = std::stoi(row[1]);
            TRACE("temp.lineID : " +  to_string(temp.lineID));
            // temp.mode = std::stoi(row[2]);
            // TRACE("Query_submission_towerinfo");
            // temp.height = std::stoi(row[2]);
            TRACE("height : " + row[2]);
            temp.height = 0;
            TRACE("temp.height : " + to_string(temp.height));  
            temp.stc_x = std::stod(row[3]);
            TRACE("temp.stc_x : " + to_string(temp.stc_x));
            temp.stc_y = std::stod(row[4]);
            TRACE("temp.stc_x : " + to_string(temp.stc_y));
            temp.stc_z = std::stod(row[5]);
            TRACE("temp.stc_z  : " + to_string(temp.stc_z));
            temp.zoom = std::stoi(row[6]);
            TRACE("temp.zoom  : " + to_string(temp.zoom));
            temp.towerShapeID = std::stoi(row[7]);
            TRACE("temp.towerShapeID  : " + to_string( temp.towerShapeID));

            string towerName = row[8];
            TRACE("towerName  : " + row[8]);
            if (towerName.front() == '@') {

			    temp.towerVirtual = 1;
		    }
            TRACE("temp.towerVirtual  : " + to_string(temp.towerVirtual));
            int basicid = stoi(row[9]);
            TRACE("basicid  : " + row[9]);
            if (basicid > 0) {
                std::string basicQuery = "SELECT savePath FROM basicList WHERE basicID = ?";
                std::vector<std::vector<std::string>> basicQueryResults;
                std::vector<std::string> basicPlaceholders = { std::to_string(basicid) };
                if (SQLiteConnector->executeQueryWithPlaceholder(basicQuery, basicPlaceholders, basicQueryResults)) {
                    if (!basicQueryResults.empty() && !basicQueryResults[0].empty()) {
                        temp.stc_path = string (basicQueryResults[0][0]);
                        TRACE("temp.stc_path  : " + temp.stc_path);
                    }
                }
            }
            TRACE("temp.stc_path  : " + temp.stc_path);
            temp.mode = 0;
            TRACE("temp.mode : " + to_string(temp.mode));

            vecTowerInfoForInitialRoute.push_back(temp);
		}

    } else {
        TRACE("Query failed ");
    }

	map<int, std::vector<TowerList>> towermap;
	AirlineTower prev;
	AirlineTower next;
	prev.stc_towerId = -1;
	next.stc_towerId = -1;
	std::vector<AirlineTower> returntower;
	std::vector<AirlineTower> alltower;

	//计算出所有杆塔（按线路飞需要计算）
	for (int i = 0; i < vecTowerInfoForInitialRoute.size();i++) {
		std::vector<TowerList> towers;
		auto iter = towermap.find(vecTowerInfoForInitialRoute[i].lineID);
		if (iter == towermap.end()) {
			towermap[vecTowerInfoForInitialRoute[i].lineID] = QueryTowersByLineID(vecTowerInfoForInitialRoute[i].lineID, true);
		}
		towers = towermap[vecTowerInfoForInitialRoute[i].lineID];
		//低2位为 00 直飞 01 提升高度 10 按线路飞 ；倒数第三位 0  顺序 1 倒序  ；倒数第四位 相邻 0 不相邻 1；倒数第五位 0 同线路 1 不同线路
		if (((vecTowerInfoForInitialRoute[i].mode & 0b11) == 0b0) || ((vecTowerInfoForInitialRoute[i].mode & 0b11) == 0b1)) {//直飞或提升高度
            TRACE("not sequence flight");
			auto ts = GetTowerShapeInformationByID(vecTowerInfoForInitialRoute[i].towerShapeID);
			vecTowerInfoForInitialRoute[i].stc_insulator_layer = ts.insulator_layer;
			vecTowerInfoForInitialRoute[i].stc_leftAngle = ts.towerTypeLeftAngle;
			vecTowerInfoForInitialRoute[i].stc_rightAngle = ts.towerTypeRightAngle;
			vecTowerInfoForInitialRoute[i].stc_height = ts.tower_height;
			vecTowerInfoForInitialRoute[i].stc_ChannelInspection = ts.ChannelInspection;
			vecTowerInfoForInitialRoute[i].stc_offCenterAngle = ts.towerTypeOffCenterAngle;
			alltower.push_back(vecTowerInfoForInitialRoute[i]);
		}
		else if ((vecTowerInfoForInitialRoute[i].mode & 0b11) == 0b10) {//按线路飞
            TRACE("begin sequence flight");
			if (i == vecTowerInfoForInitialRoute.size() - 1) {
                TRACE(" task mode error ");
				break;
			}
			if (vecTowerInfoForInitialRoute[i].mode & 0b100) {//倒序飞
                TRACE("reverse flight");
				std::vector<AirlineTower> tmptowers;
				bool bfind = false;
				bool bendfind = false;
				int findmode = 0;
				for (int j = towers.size() - 1; j > 0; j--) {
					if (towers[j].towerID == vecTowerInfoForInitialRoute[i].stc_towerId) {
						AirlineTower tmptw;
						tmptw.stc_towerId = towers[j].towerID;
						tmptw.lineID = towers[j].lineID;
						tmptw.stc_x = towers[j].longitude;
						tmptw.stc_y = towers[j].latitude;
						tmptw.stc_z = towers[j].altitude;
						tmptw.zoom = towers[j].insulatorNum;
						auto ts = GetTowerShapeInformationByID(vecTowerInfoForInitialRoute[i].towerShapeID);
						tmptw.stc_insulator_layer = ts.insulator_layer;
						tmptw.stc_leftAngle = ts.towerTypeLeftAngle;
						tmptw.stc_rightAngle = ts.towerTypeRightAngle;
						tmptw.stc_height = ts.tower_height;
						tmptw.stc_ChannelInspection = ts.ChannelInspection;
						tmptw.stc_offCenterAngle = ts.towerTypeOffCenterAngle;
						tmptw.mode = vecTowerInfoForInitialRoute[i].mode;
						findmode = vecTowerInfoForInitialRoute[i].mode;
						if (towers[j].basicID > 0) {
                            string select_sql = "SELECT savePath FROM basicList WHERE basicID = ?";
                            vector<vector<string>> queryResults;
                            vector<string> placeholders = {to_string(towers[j].basicID) };
                            if (SQLiteConnector->executeQueryWithPlaceholder(select_sql, placeholders, queryResults)) {
                                if (!queryResults.empty()) {
                                    const auto& row = queryResults[0];
                                    tmptw.stc_path = row[0];
                                }
                            }
						}
						tmptowers.push_back(tmptw);
						bfind = true;
						continue;
					}
					else if (bfind) {
						if (towers[j].towerID == vecTowerInfoForInitialRoute[i + 1].stc_towerId) {
							bendfind = true;
						}
						else {
							AirlineTower tmptw;
							tmptw.stc_towerId = towers[j].towerID;
							tmptw.lineID = towers[j].lineID;
							tmptw.stc_x = towers[j].longitude;
							tmptw.stc_y = towers[j].latitude;
							auto ts = GetTowerShapeInformationByID(towers[j].towerShapeID);
							tmptw.stc_z = towers[j].altitude + ts.tower_height;
							tmptw.zoom = towers[j].insulatorNum;
							tmptw.pass = 1;
							tmptw.mode = findmode;
                            string select_sql = "SELECT savePath FROM basicList WHERE basicID = ?";
                            vector<vector<string>> queryResults;
                            vector<string> placeholders = {to_string(towers[j].basicID) };
                            if (SQLiteConnector->executeQueryWithPlaceholder(select_sql, placeholders, queryResults)) {
                                if (!queryResults.empty()) {
                                    const auto& row = queryResults[0];
                                    tmptw.stc_path = row[0];
                                }
                            }
							tmptowers.push_back(tmptw);
						}
						if (bendfind) {
							alltower.insert(alltower.end(), tmptowers.begin(), tmptowers.end());
							break;
						}
					}
				}
				if (!bendfind) {
                    TRACE(" task mode error");
					break;
				}
			}
			else {//正序飞
                TRACE("sequence flight");
				std::vector<AirlineTower> tmptowers;
				bool bfind = false;
				bool bendfind = false;
				int findmode = 0;
				for (int j = 0; j < towers.size(); j++) {
					if (towers[j].towerID == vecTowerInfoForInitialRoute[i].stc_towerId) {
						AirlineTower tmptw;
						tmptw.stc_towerId = towers[j].towerID;
						tmptw.lineID = towers[j].lineID;
						tmptw.stc_x = towers[j].longitude;
						tmptw.stc_y = towers[j].latitude;
						tmptw.stc_z = towers[j].altitude;
						tmptw.zoom = towers[j].insulatorNum;

						auto ts = GetTowerShapeInformationByID(vecTowerInfoForInitialRoute[i].towerShapeID);
						tmptw.stc_insulator_layer = ts.insulator_layer;
						tmptw.stc_leftAngle = ts.towerTypeLeftAngle;
						tmptw.stc_rightAngle = ts.towerTypeRightAngle;
						tmptw.stc_height = ts.tower_height;
						tmptw.stc_ChannelInspection = ts.ChannelInspection;
						tmptw.stc_offCenterAngle = ts.towerTypeOffCenterAngle;

						tmptw.mode = vecTowerInfoForInitialRoute[i].mode;
						findmode = vecTowerInfoForInitialRoute[i].mode;

						if (towers[j].basicID > 0) {

                            string select_sql = "SELECT savePath FROM basicList WHERE basicID = ?";
                            vector<vector<string>> queryResults;
                            vector<string> placeholders = {to_string(towers[j].basicID) };
                            if (SQLiteConnector->executeQueryWithPlaceholder(select_sql, placeholders, queryResults)) {
                                if (!queryResults.empty()) {
                                    const auto& row = queryResults[0];
                                    tmptw.stc_path = row[0];
                                }
                            }
						}

						tmptowers.push_back(tmptw);
						bfind = true;
						continue;
					}
					else if (bfind) {
						if (towers[j].towerID == vecTowerInfoForInitialRoute[i + 1].stc_towerId) {
							bendfind = true;
						}
						else {
							AirlineTower tmptw;
							tmptw.stc_towerId = towers[j].towerID;
							tmptw.lineID = towers[j].lineID;
							tmptw.stc_x = towers[j].longitude;
							tmptw.stc_y = towers[j].latitude;
							auto ts = GetTowerShapeInformationByID(towers[j].towerShapeID);
							tmptw.stc_z = towers[j].altitude + ts.tower_height;
							tmptw.zoom = towers[j].insulatorNum;
							tmptw.pass = 1;
							tmptw.mode = findmode;
							if (towers[j].basicID > 0) {
                                string select_sql = "SELECT savePath FROM basicList WHERE basicID = ?";
                                vector<vector<string>> queryResults;
                                vector<string> placeholders = {to_string(towers[j].basicID) };
                                if (SQLiteConnector->executeQueryWithPlaceholder(select_sql, placeholders, queryResults)) {
                                    if (!queryResults.empty()) {
                                        const auto& row = queryResults[0];
                                        tmptw.stc_path = row[0];
                                    }
                                }
							}
							tmptowers.push_back(tmptw);
						}
						if (bendfind) {
							alltower.insert(alltower.end(), tmptowers.begin(), tmptowers.end());
							break;
						}
				}
	 			}
				if (!bendfind) {
					//qDebug() << QString("!!!!!!任务: %1 mode数据错误！!").arg(submission_id);
                    TRACE(" mode error");
					break;
				}
			}
		}
		
	}
	
	//所有真实杆塔做前后点
	for (int i = 0; i < alltower.size(); i++) {
		if (alltower[i].towerVirtual == 1) {
			returntower.push_back(alltower[i]);
		}
		else {
			std::vector<TowerList> towers;
			auto iter = towermap.find(alltower[i].lineID);
			if (iter == towermap.end()) {
				towermap[alltower[i].lineID] = QueryTowersByLineID(alltower[i].lineID, true);
			}
			towers = towermap[alltower[i].lineID];
			returntower.push_back(alltower[i]);
			prev = findNextTrueTower(towers, alltower[i].stc_towerId, !(alltower[i].mode & 0b100));
			prev.towerVirtual = 2;
			returntower.push_back(prev);
			next = findNextTrueTower(towers, alltower[i].stc_towerId, (alltower[i].mode & 0b100));
			next.towerVirtual = 2;
			returntower.push_back(next);
		}
	}
    TRACE(" returntower size : " + to_string(returntower.size()));
	return returntower;
}

vector<TowerList> DataProcessor::QueryTowersByLineID(int lineid, bool asc)
{
    TRACE(" === QueryTowersByLineID ===");
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
    vector<TowerList> towers;
    string ascordesc = asc ? "ASC" : "DESC";
    string select_sql = "SELECT * FROM towerList WHERE lineID = ? ORDER BY towerNumber " + ascordesc;
    vector<vector<string>> queryResults;
    vector<string> placeholders = { to_string(lineid) };
    if (SQLiteConnector->executeQueryWithPlaceholder(select_sql, placeholders, queryResults)) {
        for (const auto& row : queryResults) {
            // TRACE(" row : " + to_string(row.size()));
            TowerList temp;
            temp.towerID = stoi(row[0]);
            temp.towerName = row[1];
            temp.lineID = stoi(row[2]);
            temp.insulatorNum = stoi(row[3]);
            temp.tower_length = stod(row[4]);
            temp.tower_width = stod(row[5]);
            temp.tower_height = stod(row[6]);
            temp.towerType = row[7];
            temp.towerShapeID = stoi(row[8]);
            temp.longitude = stod(row[9]);
            temp.latitude = stod(row[10]);
            temp.altitude = stod(row[11]);
            temp.init_longitude = stod(row[12]);
            temp.init_latitude = stod(row[13]);
            temp.init_altitude = stod(row[14]);
            temp.basicID = stoi(row[15]);
            temp.AccurateLineID = stoi(row[16]);
            temp.createdTime = row[17];
            temp.createManID = stoi(row[18]);
            temp.tower_number = stoi(row[19]);
            temp.comment = row[20];
            temp.photoPosition = stoi(row[21]);

            // You can populate towerShapeName and lineName here using additional queries

            towers.push_back(temp);
        }
    }

    return towers;
}

TowerShapeList DataProcessor::GetTowerShapeInformationByID(int towershapeid)
{
    TRACE(" === GetTowerShapeInformationByID ===");
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
    TowerShapeList temp;
    string select_sql = "SELECT * FROM towerShapeList WHERE towerShapeNameID = ?";
    vector<vector<string>> queryResults;
    vector<string> placeholders = { to_string(towershapeid) };

    if (SQLiteConnector->executeQueryWithPlaceholder(select_sql, placeholders, queryResults)) {
        if (!queryResults.empty()) {
            const auto& row = queryResults[0];
            temp.tower_shape_id = stoi(row[0]);
            temp.tower_shape_name = row[1];
            temp.insulator_layer = stoi(row[2]);
            temp.initialroute_id = std::stoi(row[3]);
            temp.tower_length = stod(row[4]);
            temp.tower_width = stod(row[5]);
            temp.tower_height = stod(row[6]);
            temp.created_time = row[7];
            temp.createManID = std::stoi(row[8]);
            temp.lineTypeID = std::stoi(row[9]);
            temp.towerTypeLeftAngle = stod(row[10]);
            temp.towerTypeRightAngle = stod(row[11]);
            temp.comment = row[12];
            temp.ChannelInspection = std::stoi(row[13]);
            temp.towerTypeOffCenterAngle = stod(row[14]);
        }
    }

    return temp;
}

AirlineTower DataProcessor::findNextTrueTower(std::vector<TowerList> &towers, int towerid, bool desc)
{
	AirlineTower tw;
	tw.stc_towerId = -1;
	int increase = 1;
	int start = 0;
	int end = towers.size();
	if (desc) {
		increase = -1;
		end = -1;
		start = towers.size() - 1;
	}
	bool bfind = false;
	for (int i=start; i != end; i+=increase )
	{
		if (!bfind) {
			if (towers[i].towerID == towerid) {
				bfind = true;
			}
		}
		else {
			if (towers[i].towerName.front() == '@') {
				continue;
			}
			else {
				tw.stc_towerId = towers[i].towerID;
				tw.lineID = towers[i].lineID;
				tw.stc_x = towers[i].longitude;
				tw.stc_y = towers[i].latitude;
				tw.stc_z = towers[i].altitude;
				tw.zoom = towers[i].insulatorNum;
				tw.towerShapeID = towers[i].towerShapeID;
				return tw;
			}
		}
	}
	return tw;
}

int DataProcessor::Query_submisson_missionID(int submission_id){   
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
	int missionID = -1;
    string select_sql = "SELECT missionID FROM submissionList WHERE submissionID = ?";
    vector<vector<string>> queryResults;
    vector<string> placeholders = {to_string(submission_id)};
    if (SQLiteConnector->executeQueryWithPlaceholder(select_sql, placeholders, queryResults)) {
        if (!queryResults.empty()) {
            const auto& row = queryResults[0];
            missionID = stoi(row[0]);
        }
    }
    TRACE("missionID : " + to_string(missionID));
	return missionID;
}

int DataProcessor::get_photoPosition(int towerid){
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
	int photopos = 0;
    string select_sql = "SELECT photoPosition FROM towerList WHERE towerID = ? ";
    vector<vector<string>> queryResults;
    vector<string> placeholders = {to_string(towerid)};
    if (SQLiteConnector->executeQueryWithPlaceholder(select_sql, placeholders, queryResults)) {
        if (!queryResults.empty()) {
            const auto& row = queryResults[0];
            photopos = stoi(row[0]);
        }
    }
    // TRACE("photopos : " + to_string(photopos));
	return photopos;
}

int DataProcessor::Query_lintTypeID(int towerid){
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
	int lint_type_id;
    string select_sql = "SELECT towerShapeList.lineTypeID FROM towerShapeList WHERE towerShapeNameID IN( \
			SELECT towerList.towerShapeID FROM towerList WHERE towerID = ? ); ";
    vector<vector<string>> queryResults;
    vector<string> placeholders = {to_string(towerid)};
    if (SQLiteConnector->executeQueryWithPlaceholder(select_sql, placeholders, queryResults)) {
        if (!queryResults.empty()) {
            const auto& row = queryResults[0];
            lint_type_id = stoi(row[0]);
        }
    }
    TRACE("lint_type_id : " + to_string(lint_type_id));
    return lint_type_id;
}

TowerList DataProcessor::getFirstTower(int submission_id){
    TowerList temp;
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
	int lint_type_id;
    string query = "SELECT * FROM towerList WHERE towerID in (select towerID from submissiontowerList WHERE submissionID = ? ORDER BY towerNumber limit 1);";
    vector<string> placeholders;
    placeholders.push_back(to_string(submission_id));
    vector<vector<string>> queryResults;
    if (SQLiteConnector->executeQueryWithPlaceholder(query, placeholders, queryResults)) {
        for (const auto& row : queryResults) {
            temp.towerID = stoi(row[0]);
            temp.towerName = row[1]; 
            temp.lineID = stoi(row[2]); 
            temp.insulatorNum =  stoi(row[3]);
            temp.tower_length = stod(row[4]); 
            temp.tower_width = stod(row[5]); 
            temp.tower_height = stod(row[6]);
            temp.towerType = row[7];
            temp.towerShapeID = stoi(row[8]);
            temp.longitude =  stod(row[9]);
            temp.latitude =  stod(row[10]);
            temp.altitude =  stod(row[11]);
            temp.init_longitude =  stod(row[12]);
            temp.init_latitude = stod(row[13]);
            temp.init_altitude =  stod(row[14]);
            temp.basicID = stoi(row[15]);
            temp.AccurateLineID = stoi(row[16]);
            temp.createdTime = row[17];
            temp.createManID = stoi(row[18]);
            temp.tower_number = stoi(row[19]);
            temp.comment = row[20];
            temp.photoPosition = stoi(row[21]);
	    }
    }
	return temp;
}

vector<WaypointInformation> DataProcessor::generateMinorAirline(int submissionID, WaypointInformation current_point, int flighttasktype){
    //存装杆塔信息
	std::vector<BasicResultGet> tower_data;

	string data;

	// CommissionType2Post tempcommissionType2Post;

	// FlyingCommission1Post tempflyingCommission1Post;

	std::vector<WaypointInformation> vec_to_zj;
	if (flighttasktype == 2) {//续飞
        TRACE("flighttasktype == 2");
        FileSystem* fileInterface = new FileSystem();
		std::vector<WaypointInformation> vecComplete = readRoute(submissionID, fileInterface);
		vec_to_zj = getContinueWaypoints(submissionID, vecComplete);
        delete fileInterface;
	}
	else {
        TRACE("flighttasktype == " + to_string(flighttasktype));
		vec_to_zj = GetSubmissionWaypoints(submissionID);
	}
	if (vec_to_zj.size() < 3) {
		TRACE("data error in GenerateMinorAirline : " + to_string(vec_to_zj.size()));
		return std::vector<WaypointInformation>();
	}
    TRACE("continue getMinorAirline");
	WaypointInformation point_end = vec_to_zj[0];
	/////////////////////////////////////////把降落点的gps = landing_interest_point/////////////////////////////////////
	/////////////////////////////////////////如果没有降落点,最后//////////////////////////////////
	LongitudeAndLatitude landing_interest_point;
	landing_interest_point = Get_LongitudeAndLatitude(submissionID);
	landing_interest_point.stc_altitude = 0;
    TRACE("stc_y :"+ to_string(vec_to_zj[vec_to_zj.size() - 1].stc_y));
    TRACE("stc_x :"+ to_string( vec_to_zj[vec_to_zj.size() - 1].stc_x));
    TRACE("stc_z :"+ to_string(vec_to_zj[vec_to_zj.size() - 1].stc_z));
    CDatumPoint* m_CDatumPoint = new CDatumPoint();
	m_CDatumPoint->setOrigin(vec_to_zj[vec_to_zj.size() - 1].stc_y, vec_to_zj[vec_to_zj.size() - 1].stc_x, vec_to_zj[vec_to_zj.size() - 1].stc_z, true);
	//latitude can calculate y
	point_end.POI_Y = 100 * m_CDatumPoint->longitudeAndLatitudeToDatumPoint(landing_interest_point).stc_y;
	TRACE("point_end.POI_Y :" + to_string(point_end.POI_Y));
    ////longitude can calculate x
	point_end.POI_X = 100 * m_CDatumPoint->longitudeAndLatitudeToDatumPoint(landing_interest_point).stc_x;
	TRACE("point_end.POI_X :" + to_string(point_end.POI_X));
    ////relativeHeight can calculate z
	point_end.POI_Z = -45;
    TRACE("point_end.POI_Z :" + to_string(point_end.POI_Z));
	point_end.stc_x = landing_interest_point.stc_longitude;
	point_end.stc_y = landing_interest_point.stc_latitude;
	point_end.stc_z = landing_interest_point.stc_altitude;
	if (landing_interest_point.stc_latitude == 0 && landing_interest_point.stc_longitude == 0) {
		point_end.POI_Z = -1;
		//当前点复值过来
		point_end.stc_x = current_point.stc_x;
		point_end.stc_y = current_point.stc_y;
		point_end.stc_z = current_point.stc_z;
	}
	point_end.stc_index = vec_to_zj.size();
	point_end.stc_shootPhoto = true;
	point_end.target_class_id[0] = 7;
	point_end.necessary = false;
	vec_to_zj.push_back(point_end);
	WaypointInformation point_start = vec_to_zj[0];
	int flightHeight = Query_submisson_flightHeight(submissionID);
	point_start.stc_z = flightHeight+ current_point.stc_z;
	point_start.stc_shootPhoto = false;
	point_start.necessary = true;
	point_start.stc_gimbalPitch = -90;
	point_start.tower_id = -1;
	vec_to_zj.insert(vec_to_zj.begin(), point_start);

	/////////////////////////////////////////然后把vec_to_zj直接传给大神//////////////////////////////////

	////////////////////////更改速度参数//////////////////////////////////
	auto pre_iter = vec_to_zj.begin();
	for (std::vector<WaypointInformation>::iterator next_iter = ++vec_to_zj.begin(); next_iter != vec_to_zj.end() && pre_iter != vec_to_zj.end(); pre_iter++, next_iter++) {
		float dis = CalcPointDis(*pre_iter, *next_iter);
		if (dis >= 50.0) {
			pre_iter->maxFlightSpeed = 10.0;
		}
		else {
			pre_iter->maxFlightSpeed = 5.0;
		}
		for (auto item : pre_iter->target_class_id)
		{
			if (item == 0 && pre_iter->stc_shootPhoto == 1)
			{
				pre_iter->maxFlightSpeed = 1.2;
				break;
			}
		}
	}

	////////////////////////更改速度参数//////////////////////////////////


	// //更改子任务状态
	Change_subtask_state(submissionID);
    TRACE("vec_to_zj size :" + to_string(vec_to_zj.size()));
	return vec_to_zj;
}

LongitudeAndLatitude DataProcessor::Get_LongitudeAndLatitude(int submission_id){
    LongitudeAndLatitude temp;
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
	int lint_type_id;
    string query = "SELECT Longitude_of_landing_point,Latitude_of_landing_point FROM submissionList WHERE submissionID = ? ";
    vector<string> placeholders;
    placeholders.push_back(to_string(submission_id));
    vector<vector<string>> queryResults;
    if (SQLiteConnector->executeQueryWithPlaceholder(query, placeholders, queryResults)) {
        for (const auto& row : queryResults) {
            temp.stc_longitude = stod(row[0]);
            temp.stc_latitude = stod(row[1]);
        }
    }
    return temp;
}

int DataProcessor::GetDoublePhotoBySubmissionID(int submission_id){
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
    string query =  "SELECT doublePhoto FROM submissionList WHERE submissionID = ? ";;
    vector<string> placeholders;
    placeholders.push_back(to_string(submission_id));
    vector<vector<string>> queryResults;
    if (SQLiteConnector->executeQueryWithPlaceholder(query, placeholders, queryResults)) {
        for (const auto& row : queryResults) {
            return stoi(row[0]);
        }
    }
    return -1;

}

double DataProcessor::CalcPointDis(const WaypointInformation & l, const WaypointInformation & r)
{
	const double pi = 3.1415926535897923846;
	const float earth_radius = 6378137.0;
	double a = l.stc_y - r.stc_y;
	double b = l.stc_x - r.stc_x;
	a = a * pi / 180;
	b = b * pi / 180;
	return sqrt(pow(a * earth_radius, 2) + pow(cos(l.stc_y * pi / 180) * b * earth_radius, 2));
}

int DataProcessor::Query_submisson_flightHeight(int submission_id)
{
    int height = 0;
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
    string query =  "SELECT flightHeight FROM submissionList WHERE submissionID =  ? ";
    vector<string> placeholders;
    placeholders.push_back(to_string(submission_id));
    vector<vector<string>> queryResults;
    if (SQLiteConnector->executeQueryWithPlaceholder(query, placeholders, queryResults)) {
        for (const auto& row : queryResults) {
            height = stoi(row[0]);
        }
    }
    return height;
}

void DataProcessor::Change_subtask_state(int submission_id)
{
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance();
    std::string update_sql = "UPDATE submissionList SET completeStatus = '1' WHERE submissionID = " + std::to_string(submission_id) + ";";
    
    if (SQLiteConnector->executeNonQuery(update_sql)) {
        TRACE("Subtask state changed successfully.");
    } else {
        TRACE("Failed to change subtask state.");
    }
}

void DataProcessor::getRecord(const uint8_t* data, int length, Session* session){
     int submissionid = stoi(session->getTask()->getSubmissionID());
    try{
        FileSystem* fileInterface = new FileSystem();
        TRACE(" === getRecord ===");
        if(length > 3){

            uint8_t* data1 =  beforeSerialization(data,length);
            int newDataLen = length - 3;
            TRACE("newDataLen : " + to_string(newDataLen));

            list<WaypointInformation> way_point;
            stringstream ssid;
            stringstream ss;

            int missionid = 0;
            ssid << string((const char*)data1, sizeof(missionid));
            UnSerialization(ssid, missionid);
            TRACE("missionid : " + to_string(missionid));

            char flighttype = 0;
            ssid << string((const char*)data1 + sizeof(missionid), sizeof(flighttype));
            UnSerialization(ssid, flighttype);
            TRACE("flighttype : " + to_string(flighttype));

            unsigned int timestamp = 0;
            ssid << string((const char*)data1 + sizeof(missionid)+ sizeof(flighttype), sizeof(timestamp));
            UnSerialization(ssid, timestamp);
            TRACE("timestamp : " + to_string(timestamp));

            int len = sizeof(missionid) + sizeof(flighttype) + sizeof(timestamp);
            TRACE("len : " + to_string(len));
            if (newDataLen <= len) {
		        TRACE("error format " );
		        return;
	        }
            ss << string((const char*)data1 + len, newDataLen - len);
            string str = ss.str();
            TRACE(" str.size() : " + to_string(str.size()) +  " " + to_string(newDataLen - len) );
            for (const uint8_t byte : str) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
	        UnSerialization(ss, way_point);
            TRACE(" way_point.size() : " + to_string(way_point.size()));
        
            vector<WaypointInformation> wpvec;
            for (auto listiter = way_point.begin(); listiter != way_point.end(); listiter++) {
                wpvec.push_back(*listiter);
            }
            if (wpvec.size() == 0) {
                TRACE("return empty waypoints");
                return;
            }
           
            if (wpvec[wpvec.size()-1].waypointType == 9  && flighttype == 2) {
                //缺电返回
                TRACE("wpvec[wpvec.size()-1].waypointType == 9  && flighttype == 2");
                fileInterface->writeRoute(wpvec, submissionid, timestamp);
            } else {
                //正常返回
                TRACE(" ==== nomal return === ");
                uint32_t tmptimestamp = 0;
                if (fileInterface->checkBreakTask(submissionid, tmptimestamp)) {
                    TRACE(" checkBreakTask true ");
                    if (!fileInterface->mergeContinueWaypoints(submissionid, wpvec)) {
                        TRACE("MergeContinueWaypoints error!");
                        return; 
                    }
                   updateCompleteMissionStatus(submissionid);
                } else {
                    TRACE(" checkBreakTask false ");
                    fileInterface->writeRoute(wpvec, submissionid);
                    updateCompleteMissionStatus(submissionid);
                }
            }
            TRACE(" === send info === ");
            uint8_t answer = 0x01;
            vector<uint8_t> infoPackeage = makeInfoPackage(missionid, answer);
            for (const uint8_t byte : infoPackeage) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
            ssize_t bytesSent = send(session->socket(), infoPackeage.data(), infoPackeage.size(), 0);
            delete fileInterface;
        } 
    } catch(exception& e){
        uint8_t answer = 0x00; //fail
        vector<uint8_t> infoPackeage = makeInfoPackage(submissionid, answer);
        for (const uint8_t byte : infoPackeage) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        ssize_t bytesSent = send(session->socket(), infoPackeage.data(), infoPackeage.size(), 0);
        TRACE("getRecord UnSerialization error");
    }

}

vector<uint8_t> DataProcessor::makeInfoPackage(int missionid, uint8_t answer){
    vector<uint8_t> info;
    uint8_t cmd = 0x08;
    info.push_back(cmd);
    int answerLen = 5;
    uint16_t u16 = static_cast<uint16_t>(answerLen);
    info.push_back(static_cast<uint8_t>((u16 >> 8) & 0xFF)); // 高字节
    info.push_back(static_cast<uint8_t>(u16 & 0xFF));        // 低字节
        
    stringstream missionID;
    Serialization(missionID, missionid);
    string serializedData = missionID.str();
    TRACE(" serializedData :" + serializedData);

    for (size_t i = 0; i < 4; ++i) {
        uint8_t byte = static_cast<uint8_t>(serializedData[i]);
        info.push_back(byte);
    }

    info.push_back(answer);
    return info;
}

bool DataProcessor::updateCompleteMissionStatus(int submissionID){
    SQLiteConnection* SQLiteConnector = &SQLiteConnection::getInstance(); 
    std::string updateQuery = "UPDATE submissionList SET completeStatus = '2', droneSN = '' WHERE submissionID = '" + std::to_string(submissionID) + "';";
    bool result = SQLiteConnector->executeNonQuery(updateQuery);    
    return result;
}

vector<WaypointInformation> DataProcessor::getContinueWaypoints(int submissionID, std::vector<WaypointInformation> &completePoints){
	int completesz = completePoints.size();
	if (completesz < 1){
		return std::vector<WaypointInformation>();
	}
	std::vector<WaypointInformation> allwp = GetSubmissionWaypoints(submissionID);
	std::vector<WaypointInformation> continuewp;
	bool bfind = false;
	for (int i=0; i<allwp.size(); i++)
	{
		if (!bfind) {
			if (allwp[i].tower_id == completePoints[completesz - 1].tower_id) {
				bfind = true;
				for (int j = i; j < allwp.size(); j++) {
					if (allwp[j].necessary == true) {
						allwp[j].stc_shootPhoto = false;
						continuewp.push_back(allwp[j]);
						break;
					}
				}

				int curnum = 0;
				for (int k = completesz - 1; k > 0; k--) {
					if (completePoints[k].tower_id == allwp[i].tower_id) {
						curnum++;
					}
					else {
						break;
					}
				}
				i = i + curnum - 1;
			}
			else if (allwp[i].necessary == true) {
				allwp[i].stc_shootPhoto = false;
				continuewp.push_back(allwp[i]);
			}
		}
		else {
			continuewp.push_back(allwp[i]);
		}
	}
	return continuewp;
}

vector<WaypointInformation> DataProcessor::readRoute(int task_id, FileSystem* fileInterface)
{
	uint32_t timestamp = 0;
	string name;
   
	if (fileInterface->checkBreakTask(task_id, timestamp)) {
		name = to_string(task_id) + "_" + to_string(timestamp);
	}
	else {
		name = to_string(task_id);
	}
    return fileInterface->readDataFromFile(name);
}
