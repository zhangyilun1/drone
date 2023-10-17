#pragma once
#include <vector>
#include "common.h"
#include "Coordinate.h"
#include <cmath>
#include "cruise_interface.h"

using namespace std;


class PointToLineV2
{
public:
	PointToLineV2();
	~PointToLineV2();
	//std::vector<RouteIndexInfo> pointToLine(RecPoint input);
	std::vector<WaypointInformation> pointToLine_U_2_V2(RecPoint input);
	std::vector<WaypointInformation> pointToLine_OU_1_V2(RecPoint input);
	std::vector<WaypointInformation> pointToLine_U_S2_V2(RecPoint input);
	std::vector<WaypointInformation> route_to_osdkroute_V2(CommissionType2Post tempcommissionType2Post);
private:
	double ang2rad(int in);
	float angplus(float angle_1, float angle_2);
	int rad2ang(double in);
	vector<double> matrixrotate2d(vector<double> in_point, int theta);
	vector<double> get_unit_directon2d(double x1, double y1, double x2, double y2);
	vector<double> get_unit_directon3d(double x1, double y1, double z1, double x2, double y2, double z2);
	double get_distance2d(double x1, double y1, double x2, double y2);
	double get_distance3d(double x1, double y1, double z1, double x2, double y2, double z2);
	int get_angle_tonorth(double  x2, double  y2);


};
