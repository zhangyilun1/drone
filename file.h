#ifndef _FILE_H_
#define _FILE_H_

#include <iostream>
#include <filesystem>
#include <string>
#include <fstream>
#include "jsonData.h"
#include "common.h"
#include "log.h"
using namespace std;

class FileSystem {
public:
    FileSystem();
    ~FileSystem();
    bool checkBreakTask(int subtaskid, uint32_t &timestamp);
    bool deleteFileOrFolder(const string& strPath);
    bool writeRoute(const std::vector<WaypointInformation>& vec, int task_id, uint32_t timestamp);
    bool writeRoute(vector<WaypointInformation> vec, int task_id);
    bool isDirExist(const std::string& fullPath);
    bool getJsonDataAndWriteFile(std::vector<WaypointInformation> vec, const std::string& filename);
    bool writeDataToFile(const std::string& file_path, const std::string& file_content);
    bool mergeContinueWaypoints(int submissionID, std::vector<WaypointInformation>& waypoints);
    vector<WaypointInformation> readRoute(int task_id);
    vector<WaypointInformation> readDataFromFile(string name);
};
#endif