#ifndef _JSONDATA_H_
#define _JSONDATA_H_

#include <iostream>
#include "common.h"
#include "log.h"
#include <nlohmann/json.hpp>

using namespace std;

class JsonData
{

public:
    JsonData(/* args */);
    ~JsonData();
    bool waypoint2JSONSplitByTowerID(const vector<WaypointInformation>& vec, string& wholejson,
                                        vector<int>& toweridvec, vector<string>& towerstrvec);

    vector<WaypointInformation> Converting_the_aircraft_waypoint_structure_into_a_structure(const std::string& jsonStr);



};


#endif