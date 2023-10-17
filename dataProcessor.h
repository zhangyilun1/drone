#ifndef _DATA_PROCESSOR_H_
#define _DATA_PROCESSOR_H_

#include "drones.h"
#include "sqliteConnection.h"
#include "log.h"
#include "session.h"
#include "common.h"
#include "task.h"
#include "file.h"
#include "md5.h"
#include "./algorithm/PointToLineV3.h"
#include <cereal/archives/binary.hpp>
#include <algorithm>
#include <cereal/types/string.hpp>
#include "config.h"
#include <nlohmann/json.hpp>


using std::min;
class Session;
// std::map<std::string, Session*> droneSessionMap;

class DataProcessor {

public:
    DataProcessor();
    ~DataProcessor();

    // void processDronesData(const char* data, int length);
    // string getDroneSNCode(const char* data, int length);
    
    // for php 
    void processReceivedData(const char* data, int length, Session* session);  
    // for drone 
    void processDronesData(const uint8_t* data, int length, Session* session);
    
    void processTask(const uint8_t* data, int length, SQLiteConnection* SQLiteConnector, Drones* drone, Session* session);
    //Tasks* getTaskInfo(const uint8_t* data, int length, Session* session);
    void getTaskInfo(const uint8_t* data, int length, Session* session);
    //void processUpgrade(const uint8_t* data, int length, SQLiteConnection* SQLiteConnector, Drones* drone, Session* session);
    void processUpgrade(const uint8_t* data, int length, Session* session);
    HeartbeatInfo getHeartbeatInfo(const uint8_t* data, int length, Drones* drone, Session* session);

    void getRecord(const uint8_t* data, int length, Session* session);

private:
    void processSNCode(const uint8_t* data, int length, SQLiteConnection* SQLiteConnector, Drones* drone, Session* session);
    DroneData getDroneSNCode(const uint8_t* data, int length);
    vector<WaypointInformation> GetSubmissionWaypoints(int submissionID);
    vector<AirlineTower> Query_submission_towerinfo(int submissionID);
    vector<TowerList> QueryTowersByLineID(int lineid, bool asc);
    TowerShapeList GetTowerShapeInformationByID(int towershapeid);
    AirlineTower findNextTrueTower(std::vector<TowerList> &towers, int towerid, bool desc);

    int Query_submisson_missionID(int submission_id);
    int get_photoPosition(int towerid);
    int Query_lintTypeID(int towerid);
    TowerList getFirstTower(int submission_id);
    vector<WaypointInformation> generateMinorAirline(int submissionID, WaypointInformation current_point, int flighttasktype);
    LongitudeAndLatitude Get_LongitudeAndLatitude(int submission_id);
    int GetDoublePhotoBySubmissionID(int submission_id);
    double CalcPointDis(const WaypointInformation & l, const WaypointInformation & r);
    int Query_submisson_flightHeight(int submission_id);
    void Change_subtask_state(int submission_id);
    bool updateCompleteMissionStatus(int submissionID);

    vector<vector<string>> querySystemVersionAndFilePath();
    bool compareVersionnNumberSQL(const char* data);
    char* fileProcess(string filePath);
    void sendFile(int clientSocket, const std::string& filePath, string md5Hash);
    void sendUpgradePacket(int clientSocket, const UpgradePacket& packet);
    bool versionNumberCompareWithDB(const char* snCode, const string dbVersionNumber);
    int whetherRoolBack(ConfigFile* mConfig);
    void sendFileBy10KB(char* buffer, int bufferSize, int clientSocket);
    vector<uint8_t> makeInfoPackage(int missionid, uint8_t answer);
    vector<WaypointInformation> getContinueWaypoints(int submissionID, std::vector<WaypointInformation> &completePoints);
    vector<WaypointInformation> readRoute(int task_id, FileSystem* fileInterface);
private:
    Drones* drone;
    Tasks* task;
    SQLiteConnection* SQLiteConnector;
    // map<string, Session*> droneSessionMap;
    Session* session;

public:
    TaskInfo taskInfo;
    PointToLineV3 m_point_to_line_v3;
    CDatumPoint m_CDatumPoint;
};

#endif
