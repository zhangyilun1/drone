#ifndef _DRONES_H_
#define _DRONES_H_

#include <iostream>
#include <string>
#include "common.h"
using namespace std;
class Drones {

public:

    Drones();
    ~Drones();

    string getSnCode();
    void setSnCode(string snCode);

    string getDroneType();
    void setDroneType(string droneType);

    string getMaxSpeed();
    void setMaxSpeed(string maxSpeed);

    string getSystemVersion();
    void setSystemVersion(string systemVersion);

    string getLen();
    void setLen(string len);

    HeartbeatInfo getHeartbeatInfo();
    void setSystemVersion(HeartbeatInfo infor);

private:
    string snCode;
    string droneType;
    string maxSpeed;
    string systemVersion;
    //string mode;
    string len;
    HeartbeatInfo infor;



};

#endif