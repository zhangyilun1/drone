#include "PointToLine.h"
#include <cmath>
#include "cruise_interface.h"
using namespace std;
CPointToLine::CPointToLine()
{

}

CPointToLine::~CPointToLine()
{

}

double ang2rad(int in) {
	double radius = (double)in / 180 * 3.1415926;
	return radius;
}
float angplus(float angle_1, float angle_2) {
	float angle_out = angle_1 + angle_2;
	if (angle_out > 180) {
		angle_out = angle_out - 360;
	}
	if (angle_out < -180) {
		angle_out = angle_out + 360;
	}
	return angle_out;
}
int rad2ang(double in) {
	double degree = in / 3.1415926 * 180;
	return degree;
}

vector<double> matrixrotate2d(vector<double> in_point, int theta)
{
	double x = in_point[0];
	double y = in_point[1];
	double angle_rad = ang2rad(theta);
	double new_x = x * cos(angle_rad) - y * sin(angle_rad);
	double new_y = x * sin(angle_rad) + y * cos(angle_rad);
	vector<double> result;
	result.push_back(new_x);
	result.push_back(new_y);
	return result;
}
vector<double> get_unit_directon2d(double x1, double y1, double x2, double y2)
{
	double dx = x2 - x1;
	double dy = y2 - y1;
	double square = dx * dx + dy * dy;
	double dis_of_two = sqrt(square);
	double new_x = dx / dis_of_two;
	double new_y = dy / dis_of_two;
	vector<double> result;
	result.push_back(new_x);
	result.push_back(new_y);
	return result;
}
vector<double> get_unit_directon3d(double x1, double y1, double z1, double x2, double y2, double z2)
{
	double dx = x2 - x1;
	double dy = y2 - y1;
	double dz = z2 - z1;
	double square = dx * dx + dy * dy + dz * dz;
	double dis_of_two = sqrt(square);
	double new_x = dx / dis_of_two;
	double new_y = dy / dis_of_two;
	double new_z = dz / dis_of_two;
	vector<double> result;
	result.push_back(new_x);
	result.push_back(new_y);
	result.push_back(new_z);
	return result;
}
double get_distance2d(double x1, double y1, double x2, double y2)
{
	double dx = x2 - x1;
	double dy = y2 - y1;
	double square = dx * dx + dy * dy;
	double dis_of_two = sqrt(square);
	return dis_of_two;
}
double get_distance3d(double x1, double y1, double z1, double x2, double y2, double z2)
{
	double dx = x2 - x1;
	double dy = y2 - y1;
	double dz = z2 - z1;
	double square = dx * dx + dy * dy + dz * dz;
	double dis_of_two = sqrt(square);
	return dis_of_two;
}
int get_angle_tonorth(double  x2, double  y2)
{
	double z1 = 0;
	double z2 = 0;
	double x1 = 0;
	double y1 = 1;
	if (x1 == 0 && y1 == 0 && z1 == 0) return 0;
	if (x2 == 0 && y2 == 0 && z2 == 0) return 0;
	double a_dot_b = x1 * x2 + y1 * y2 + z1 * z2;
	double a_lenth = sqrt(x1 * x1 + y1 * y1 + z1 * z1);
	double b_lenth = sqrt(x2 * x2 + y2 * y2 + z2 * z2);
	double cos_angle = a_dot_b / a_lenth / b_lenth;
	double angle_rad = acos(cos_angle);
	int angle_deg = rad2ang(angle_rad);
	if (x2 < 0) {
		angle_deg = -angle_deg;
	}
	return angle_deg;
}



std::vector<RouteIndexInfo> CPointToLine::pointToLine_U_2(RecPoint input)
{
	vector<RouteIndexInfo> line_out;
	line_out.clear();
	DatumPoint initial_line_up_point = input.rec_pt[0];
	DatumPoint initial_line_right_point = input.rec_pt[1];
	DatumPoint initial_line_left_point = input.rec_pt[2];
	DatumPoint initial_line_prev_point = input.prev_pt;
	DatumPoint initial_line_next_point = input.next_pt;

	int layer_num = input.layer_num;
	int left_yaw_angle = input.left_angle;
	int right_yaw_angle = input.right_angle;
	int ChannelInspection = input.ChannelInspection;
	if (layer_num == 0)
	{
		return line_out;
	}
	if (initial_line_right_point.stc_x == 0 && initial_line_right_point.stc_y == 0)
	{
		return line_out;
	}
	if (initial_line_left_point.stc_x == 0 && initial_line_left_point.stc_y == 0)
	{
		return line_out;
	}
	int yaw_rotate_angle_selected = input.yaw_rotate_angle_selected;
	//double longlook_whole_tower_distance = input.longlook_whole_tower_distance;
	double longlook_whole_tower_distance = input.height + 10;

	double dis_right = get_distance2d(initial_line_right_point.stc_x, initial_line_right_point.stc_y, initial_line_up_point.stc_x, initial_line_up_point.stc_y);
	double dis_left = get_distance2d(initial_line_left_point.stc_x, initial_line_left_point.stc_y, initial_line_up_point.stc_x, initial_line_up_point.stc_y);
	vector<double> unit_rightpoint = get_unit_directon2d(initial_line_right_point.stc_x, initial_line_right_point.stc_y, initial_line_up_point.stc_x, initial_line_up_point.stc_y);
	vector<double> unit_leftpoint = get_unit_directon2d(initial_line_left_point.stc_x, initial_line_left_point.stc_y, initial_line_up_point.stc_x, initial_line_up_point.stc_y);
	vector<double> d_right_forward = matrixrotate2d(unit_rightpoint, -right_yaw_angle);
	vector<double> d_right_backward = matrixrotate2d(unit_rightpoint, right_yaw_angle);
	vector<double> d_right_prev_perpendicular = matrixrotate2d(unit_rightpoint, 90);
	vector<double> d_left_forward = matrixrotate2d(unit_leftpoint, left_yaw_angle);
	vector<double> d_left_backward = matrixrotate2d(unit_leftpoint, -left_yaw_angle);
	vector<double> d_left_next_perpendicular = matrixrotate2d(unit_leftpoint, 90);

	d_right_forward[0] = -d_right_forward[0] * dis_right;
	d_right_backward[0] = -d_right_backward[0] * dis_right;
	d_left_forward[0] = -d_left_forward[0] * dis_left;
	d_left_backward[0] = -d_left_backward[0] * dis_left;
	d_right_forward[1] = -d_right_forward[1] * dis_right;
	d_right_backward[1] = -d_right_backward[1] * dis_right;
	d_left_forward[1] = -d_left_forward[1] * dis_left;
	d_left_backward[1] = -d_left_backward[1] * dis_left;
	double unit_halfheight = fabs((initial_line_right_point.stc_z - initial_line_left_point.stc_z) / (layer_num - 1) / 2);
	//前置判断3个点符不符合标准，，return 什么你自己定义一下，以后我再慢慢补充规则
	if (fabs(initial_line_up_point.stc_z - initial_line_right_point.stc_z) < unit_halfheight) {
		//目前暂定 当第一个点的index为-1时，则为不符合标准
		vector<RouteIndexInfo> buffer;
		RouteIndexInfo buffer_first_data;
		buffer_first_data.stc_index = -1;
		buffer.push_back(buffer_first_data);
		return buffer;
	}
	//报错绝缘子层数设置异常
	if (layer_num == 0 || layer_num > 4) {
		vector<RouteIndexInfo> buffer;
		RouteIndexInfo buffer_first_data;
		buffer_first_data.stc_index = -1;
		buffer.push_back(buffer_first_data);
		return buffer;
	}
	RouteIndexInfo point_0;
	if (pow(initial_line_prev_point.stc_x, 2) + pow(initial_line_prev_point.stc_y, 2) <= 1)
	{
		point_0.stc_x = initial_line_up_point.stc_x + longlook_whole_tower_distance * d_right_prev_perpendicular[0];
		point_0.stc_y = initial_line_up_point.stc_y + longlook_whole_tower_distance * d_right_prev_perpendicular[1];
		point_0.stc_z = initial_line_up_point.stc_z + 10;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -35;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(1);
		//temp_target_class_id.push_back(3);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
		////////////////////////悬停刹车专用
		point_0.stc_x = initial_line_up_point.stc_x + 10 * d_right_prev_perpendicular[0];
		point_0.stc_y = initial_line_up_point.stc_y + 10 * d_right_prev_perpendicular[1];
		point_0.stc_z = initial_line_up_point.stc_z;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -90;
		point_0.stc_shootPhoto = 0;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.push_back(2);
		//temp_target_class_id.push_back(3);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
	}
	else
	{
		vector<double> unit_prev_directon = get_unit_directon3d(initial_line_up_point.stc_x, initial_line_up_point.stc_y, initial_line_up_point.stc_z, initial_line_prev_point.stc_x, initial_line_prev_point.stc_y, initial_line_prev_point.stc_z);
		if (ChannelInspection == 1) {
			double dis_between_2_tower = get_distance3d(initial_line_up_point.stc_x, initial_line_up_point.stc_y, initial_line_up_point.stc_z, initial_line_prev_point.stc_x, initial_line_prev_point.stc_y, initial_line_prev_point.stc_z);
			double quater_dis_between_2_tower = dis_between_2_tower / 4;
			for (int iii = 3; iii > 1; iii--) {
				point_0.stc_x = initial_line_up_point.stc_x + iii * quater_dis_between_2_tower * unit_prev_directon[0];
				point_0.stc_y = initial_line_up_point.stc_y + iii * quater_dis_between_2_tower * unit_prev_directon[1];
				point_0.stc_z = initial_line_up_point.stc_z + iii * quater_dis_between_2_tower * unit_prev_directon[2];
				point_0.stc_index = line_out.size() + 1;
				point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_prev_point.stc_x, initial_line_up_point.stc_y - initial_line_prev_point.stc_y);
				point_0.stc_gimbalPitch = -30;
				point_0.stc_shootPhoto = 1;
				point_0.stc_opticalZoomFocalLength = 1;
				std::vector<int> temp_target_class_id_tep;
				temp_target_class_id_tep.push_back(2);
				point_0.target_class_id = temp_target_class_id_tep;
				line_out.push_back(point_0);
			}
		}
		point_0.stc_x = initial_line_up_point.stc_x + longlook_whole_tower_distance * unit_prev_directon[0];
		point_0.stc_y = initial_line_up_point.stc_y + longlook_whole_tower_distance * unit_prev_directon[1];
		point_0.stc_z = initial_line_up_point.stc_z + longlook_whole_tower_distance * unit_prev_directon[2];
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_prev_point.stc_x, initial_line_up_point.stc_y - initial_line_prev_point.stc_y);
		point_0.stc_gimbalPitch = -30;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(1);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
		////////////////////////悬停刹车专用
		point_0.stc_x = initial_line_up_point.stc_x + 10 * unit_prev_directon[0];
		point_0.stc_y = initial_line_up_point.stc_y + 10 * unit_prev_directon[1];
		point_0.stc_z = initial_line_up_point.stc_z;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -90;
		point_0.stc_shootPhoto = 0;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.push_back(2);
		//temp_target_class_id.push_back(3);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
	}

	point_0.stc_x = initial_line_up_point.stc_x;
	point_0.stc_y = initial_line_up_point.stc_y;
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(d_right_prev_perpendicular[0], d_right_prev_perpendicular[1]);
	point_0.stc_gimbalPitch = -90;
	point_0.stc_shootPhoto = 1;
	point_0.stc_opticalZoomFocalLength = 1;
	std::vector<int> temp_target_class_id;
	temp_target_class_id.push_back(2);
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	point_0.stc_x = d_right_backward[0];
	point_0.stc_y = d_right_backward[1];
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_right_backward[0], initial_line_up_point.stc_y - d_right_backward[1]);
	point_0.stc_gimbalPitch = -38;
	point_0.stc_shootPhoto = 1;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	for (int m = 0; m <= layer_num; m++)
	{
		RouteIndexInfo point_1;
		point_1.stc_x = d_right_backward[0];
		point_1.stc_y = d_right_backward[1];
		point_1.stc_z = initial_line_right_point.stc_z + (1 - 2 * m) * unit_halfheight;
		point_1.stc_index = line_out.size() + 1;
		float initial_right_angle = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_right_point.stc_x, initial_line_up_point.stc_y - initial_line_right_point.stc_y);
		point_1.stc_yaw = (get_angle_tonorth(initial_line_up_point.stc_x - d_right_backward[0], initial_line_up_point.stc_y - d_right_backward[1]) + initial_right_angle) / 2;
		point_1.stc_gimbalPitch = 0;
		point_1.stc_shootPhoto = 1;
		point_1.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(0);
		if (m == 0) {
			temp_target_class_id.push_back(4);
		}
		point_1.target_class_id = temp_target_class_id;
		line_out.push_back(point_1);
	}
	point_0.stc_x = initial_line_right_point.stc_x;
	point_0.stc_y = initial_line_right_point.stc_y;
	point_0.stc_z = initial_line_left_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	if (pow(initial_line_prev_point.stc_x, 2) + pow(initial_line_prev_point.stc_y, 2) <= 1)
	{
		point_0.stc_yaw = get_angle_tonorth(d_right_prev_perpendicular[0], d_right_prev_perpendicular[1]);
	}
	else {
		point_0.stc_yaw = get_angle_tonorth(initial_line_prev_point.stc_x - initial_line_up_point.stc_x, initial_line_prev_point.stc_y - initial_line_up_point.stc_y);
	}
	point_0.stc_gimbalPitch = 3;
	point_0.stc_shootPhoto = 1;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(2);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);
	for (int m = layer_num; m >= 0; m--)
	{
		//qDebug() << m;
		RouteIndexInfo point_1;
		point_1.stc_x = d_right_forward[0];
		point_1.stc_y = d_right_forward[1];
		point_1.stc_z = initial_line_right_point.stc_z + (1 - 2 * m) * unit_halfheight;
		point_1.stc_index = line_out.size() + 1;
		float initial_right_angle = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_right_point.stc_x, initial_line_up_point.stc_y - initial_line_right_point.stc_y);
		point_1.stc_yaw = (get_angle_tonorth(initial_line_up_point.stc_x - d_right_forward[0], initial_line_up_point.stc_y - d_right_forward[1]) + initial_right_angle) / 2;
		point_1.stc_gimbalPitch = 0;
		point_1.stc_shootPhoto = 1;
		point_1.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(0);
		point_1.target_class_id = temp_target_class_id;
		line_out.push_back(point_1);
	}

	point_0.stc_x = d_right_forward[0];
	point_0.stc_y = d_right_forward[1];
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_right_forward[0], initial_line_up_point.stc_y - d_right_forward[1]);
	point_0.stc_gimbalPitch = -40;
	point_0.stc_shootPhoto = 0;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	point_0.stc_x = d_left_forward[0];
	point_0.stc_y = d_left_forward[1];
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_left_forward[0], initial_line_up_point.stc_y - d_left_forward[1]);
	point_0.stc_gimbalPitch = -40;
	point_0.stc_shootPhoto = 1;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	for (int m = layer_num; m >= 0; m--)
	{
		RouteIndexInfo point_1;
		point_1.stc_x = d_left_forward[0];
		point_1.stc_y = d_left_forward[1];
		point_1.stc_z = initial_line_left_point.stc_z + (2 * m - 1) * unit_halfheight;
		point_1.stc_index = line_out.size() + 1;
		float initial_left_angle = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_left_point.stc_x, initial_line_up_point.stc_y - initial_line_left_point.stc_y);
		point_1.stc_yaw = (get_angle_tonorth(initial_line_up_point.stc_x - d_left_forward[0], initial_line_up_point.stc_x - d_left_forward[1]) + initial_left_angle) / 2;
		point_1.stc_gimbalPitch = 0;
		point_1.stc_shootPhoto = 1;
		point_1.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(0);
		if (m == layer_num) {
			temp_target_class_id.push_back(4);
		}
		point_1.target_class_id = temp_target_class_id;
		line_out.push_back(point_1);
	}
	point_0.stc_x = initial_line_left_point.stc_x;
	point_0.stc_y = initial_line_left_point.stc_y;
	point_0.stc_z = initial_line_left_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	if (pow(initial_line_next_point.stc_x, 2) + pow(initial_line_next_point.stc_y, 2) <= 1)
	{
		point_0.stc_yaw = get_angle_tonorth(-d_right_prev_perpendicular[0], -d_right_prev_perpendicular[1]);
	}
	else {
		point_0.stc_yaw = get_angle_tonorth(initial_line_next_point.stc_x - initial_line_up_point.stc_x, initial_line_next_point.stc_y - initial_line_up_point.stc_y);
	}
	point_0.stc_gimbalPitch = 4;
	point_0.stc_shootPhoto = 1;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(2);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);
	for (int m = 0; m <= layer_num; m++)
	{
		RouteIndexInfo point_1;
		point_1.stc_x = d_left_backward[0];
		point_1.stc_y = d_left_backward[1];
		point_1.stc_z = initial_line_left_point.stc_z + (2 * m - 1) * unit_halfheight;
		point_1.stc_index = line_out.size() + 1;
		float initial_left_angle = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_left_point.stc_x, initial_line_up_point.stc_y - initial_line_left_point.stc_y);
		point_1.stc_yaw = (get_angle_tonorth(initial_line_up_point.stc_x - d_left_backward[0], initial_line_up_point.stc_x - d_left_backward[1]) + initial_left_angle) / 2;
		point_1.stc_gimbalPitch = 0;
		point_1.stc_shootPhoto = 1;
		point_1.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(0);
		point_1.target_class_id = temp_target_class_id;
		line_out.push_back(point_1);
	}

	point_0.stc_x = d_left_backward[0];
	point_0.stc_y = d_left_backward[1];
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_left_backward[0], initial_line_up_point.stc_y - d_left_backward[1]);
	point_0.stc_gimbalPitch = -40;
	point_0.stc_shootPhoto = 0;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);
	//以上3行杆塔垂直角度获取
	point_0.stc_x = initial_line_up_point.stc_x;
	point_0.stc_y = initial_line_up_point.stc_y;
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(d_right_prev_perpendicular[0], d_right_prev_perpendicular[1]);
	point_0.stc_gimbalPitch = -90;
	point_0.stc_shootPhoto = 0;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(2);
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	if (pow(initial_line_next_point.stc_x, 2) + pow(initial_line_next_point.stc_y, 2) <= 1)
	{
		point_0.stc_x = initial_line_up_point.stc_x + longlook_whole_tower_distance * d_left_next_perpendicular[0];
		point_0.stc_y = initial_line_up_point.stc_y + longlook_whole_tower_distance * d_left_next_perpendicular[1];
		point_0.stc_z = initial_line_up_point.stc_z + 10;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -40;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(1);
		//temp_target_class_id.push_back(5);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
	}
	else
	{
		vector<double> unit_next_directon = get_unit_directon3d(initial_line_up_point.stc_x, initial_line_up_point.stc_y, initial_line_up_point.stc_z, initial_line_next_point.stc_x, initial_line_next_point.stc_y, initial_line_next_point.stc_z);
		point_0.stc_x = initial_line_up_point.stc_x + longlook_whole_tower_distance * unit_next_directon[0];
		point_0.stc_y = initial_line_up_point.stc_y + longlook_whole_tower_distance * unit_next_directon[1];
		point_0.stc_z = initial_line_up_point.stc_z + longlook_whole_tower_distance * unit_next_directon[2];
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_next_point.stc_x, initial_line_up_point.stc_y - initial_line_next_point.stc_y);
		point_0.stc_gimbalPitch = -35;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.clear();
		temp_target_class_id.push_back(1);
		temp_target_class_id.push_back(5);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
	}
	return line_out;
}
/////////////////////////////U型精简两次
std::vector<RouteIndexInfo> CPointToLine::pointToLine_U_S2(RecPoint input)
{
	float peer_2_yaw_center = 12;
	vector<RouteIndexInfo> line_out;
	line_out.clear();
	DatumPoint initial_line_up_point = input.rec_pt[0];
	DatumPoint initial_line_right_point = input.rec_pt[1];
	DatumPoint initial_line_left_point = input.rec_pt[2];
	DatumPoint initial_line_prev_point = input.prev_pt;
	DatumPoint initial_line_next_point = input.next_pt;

	int layer_num = input.layer_num;
	int left_yaw_angle = input.left_angle;
	int right_yaw_angle = input.right_angle;
	int ChannelInspection = input.ChannelInspection;
	if (layer_num == 0)
	{
		return line_out;
	}
	if (initial_line_right_point.stc_x == 0 && initial_line_right_point.stc_y == 0)
	{
		return line_out;
	}
	if (initial_line_left_point.stc_x == 0 && initial_line_left_point.stc_y == 0)
	{
		return line_out;
	}
	int yaw_rotate_angle_selected = input.yaw_rotate_angle_selected;
	//double longlook_whole_tower_distance = input.longlook_whole_tower_distance;
	double longlook_whole_tower_distance = input.height + 10;

	double dis_right = get_distance2d(initial_line_right_point.stc_x, initial_line_right_point.stc_y, initial_line_up_point.stc_x, initial_line_up_point.stc_y);
	double dis_left = get_distance2d(initial_line_left_point.stc_x, initial_line_left_point.stc_y, initial_line_up_point.stc_x, initial_line_up_point.stc_y);
	vector<double> unit_rightpoint = get_unit_directon2d(initial_line_right_point.stc_x, initial_line_right_point.stc_y, initial_line_up_point.stc_x, initial_line_up_point.stc_y);
	vector<double> unit_leftpoint = get_unit_directon2d(initial_line_left_point.stc_x, initial_line_left_point.stc_y, initial_line_up_point.stc_x, initial_line_up_point.stc_y);
	vector<double> d_right_forward = matrixrotate2d(unit_rightpoint, -right_yaw_angle);
	vector<double> d_right_backward = matrixrotate2d(unit_rightpoint, right_yaw_angle);
	vector<double> d_right_prev_perpendicular = matrixrotate2d(unit_rightpoint, 90);
	vector<double> d_left_forward = matrixrotate2d(unit_leftpoint, left_yaw_angle);
	vector<double> d_left_backward = matrixrotate2d(unit_leftpoint, -left_yaw_angle);
	vector<double> d_left_next_perpendicular = matrixrotate2d(unit_leftpoint, 90);

	d_right_forward[0] = -d_right_forward[0] * dis_right;
	d_right_backward[0] = -d_right_backward[0] * dis_right;
	d_left_forward[0] = -d_left_forward[0] * dis_left;
	d_left_backward[0] = -d_left_backward[0] * dis_left;
	d_right_forward[1] = -d_right_forward[1] * dis_right;
	d_right_backward[1] = -d_right_backward[1] * dis_right;
	d_left_forward[1] = -d_left_forward[1] * dis_left;
	d_left_backward[1] = -d_left_backward[1] * dis_left;
	double unit_halfheight = fabs((initial_line_right_point.stc_z - initial_line_left_point.stc_z) / ((layer_num - 1)*2));
	//前置判断3个点符不符合标准，，return 什么你自己定义一下，以后我再慢慢补充规则
	if (fabs(initial_line_up_point.stc_z - initial_line_right_point.stc_z) < unit_halfheight) {
		//目前暂定 当第一个点的index为-1时，则为不符合标准
		vector<RouteIndexInfo> buffer;
		RouteIndexInfo buffer_first_data;
		buffer_first_data.stc_index = -1;
		buffer.push_back(buffer_first_data);
		return buffer;
	}
	//报错绝缘子层数设置异常
	if (layer_num <= 1 || layer_num > 4) {
		vector<RouteIndexInfo> buffer;
		RouteIndexInfo buffer_first_data;
		buffer_first_data.stc_index = -2;
		buffer.push_back(buffer_first_data);
		return buffer;
	}
	RouteIndexInfo point_0;
	if (pow(initial_line_prev_point.stc_x, 2) + pow(initial_line_prev_point.stc_y, 2) <= 1)
	{
		point_0.stc_x = initial_line_up_point.stc_x + longlook_whole_tower_distance * d_right_prev_perpendicular[0];
		point_0.stc_y = initial_line_up_point.stc_y + longlook_whole_tower_distance * d_right_prev_perpendicular[1];
		point_0.stc_z = initial_line_up_point.stc_z + 10;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -40;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(1);
		//temp_target_class_id.push_back(3);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
		////////////////////////悬停刹车专用
		point_0.stc_x = initial_line_up_point.stc_x + 10 * d_right_prev_perpendicular[0];
		point_0.stc_y = initial_line_up_point.stc_y + 10 * d_right_prev_perpendicular[1];
		point_0.stc_z = initial_line_up_point.stc_z;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -90;
		point_0.stc_shootPhoto = 0;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.push_back(2);
		//temp_target_class_id.push_back(3);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
	}
	else
	{
		vector<double> unit_prev_directon = get_unit_directon3d(initial_line_up_point.stc_x, initial_line_up_point.stc_y, initial_line_up_point.stc_z, initial_line_prev_point.stc_x, initial_line_prev_point.stc_y, initial_line_prev_point.stc_z);
		if (ChannelInspection == 1) {
			double dis_between_2_tower = get_distance3d(initial_line_up_point.stc_x, initial_line_up_point.stc_y, initial_line_up_point.stc_z, initial_line_prev_point.stc_x, initial_line_prev_point.stc_y, initial_line_prev_point.stc_z);
			double quater_dis_between_2_tower = dis_between_2_tower / 4;
			for (int iii = 3; iii > 1; iii--) {
				point_0.stc_x = initial_line_up_point.stc_x + iii * quater_dis_between_2_tower * unit_prev_directon[0];
				point_0.stc_y = initial_line_up_point.stc_y + iii * quater_dis_between_2_tower * unit_prev_directon[1];
				point_0.stc_z = initial_line_up_point.stc_z + iii * quater_dis_between_2_tower * unit_prev_directon[2];
				point_0.stc_index = line_out.size() + 1;
				point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_prev_point.stc_x, initial_line_up_point.stc_y - initial_line_prev_point.stc_y);
				point_0.stc_gimbalPitch = -45;
				point_0.stc_shootPhoto = 1;
				point_0.stc_opticalZoomFocalLength = 1;
				std::vector<int> temp_target_class_id_tep;
				temp_target_class_id_tep.push_back(2);
				point_0.target_class_id = temp_target_class_id_tep;
				line_out.push_back(point_0);
			}
		}
		point_0.stc_x = initial_line_up_point.stc_x + longlook_whole_tower_distance * unit_prev_directon[0];
		point_0.stc_y = initial_line_up_point.stc_y + longlook_whole_tower_distance * unit_prev_directon[1];
		point_0.stc_z = initial_line_up_point.stc_z + longlook_whole_tower_distance * unit_prev_directon[2];
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_prev_point.stc_x, initial_line_up_point.stc_y - initial_line_prev_point.stc_y);
		point_0.stc_gimbalPitch = -38;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(1);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
		////////////////////////悬停刹车专用
		point_0.stc_x = initial_line_up_point.stc_x + 10 * unit_prev_directon[0];
		point_0.stc_y = initial_line_up_point.stc_y + 10 * unit_prev_directon[1];
		point_0.stc_z = initial_line_up_point.stc_z;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -90;
		point_0.stc_shootPhoto = 0;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.push_back(2);
		//temp_target_class_id.push_back(3);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
	}

	point_0.stc_x = initial_line_up_point.stc_x;
	point_0.stc_y = initial_line_up_point.stc_y;
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(d_left_next_perpendicular[0], d_left_next_perpendicular[1]);
	point_0.stc_gimbalPitch = -90;
	point_0.stc_shootPhoto = 1;
	point_0.stc_opticalZoomFocalLength = 1;
	std::vector<int> temp_target_class_id;
	temp_target_class_id.push_back(2);
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	point_0.stc_x = d_right_backward[0];
	point_0.stc_y = d_right_backward[1];
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_right_backward[0], initial_line_up_point.stc_y - d_right_backward[1]);
	point_0.stc_gimbalPitch = -45;
	point_0.stc_shootPhoto = 1;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	for (int m = 1; m <= layer_num; m +=2)
	{
		RouteIndexInfo point_1;
		point_1.stc_x = d_right_backward[0];
		point_1.stc_y = d_right_backward[1];
		point_1.stc_z = initial_line_right_point.stc_z + (1 - 2 * m) * unit_halfheight;
		point_1.stc_index = line_out.size() + 1;
		float initial_right_angle = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_right_point.stc_x, initial_line_up_point.stc_y - initial_line_right_point.stc_y);
		//point_1.stc_yaw = (get_angle_tonorth(initial_line_up_point.stc_x - d_right_backward[0], initial_line_up_point.stc_y - d_right_backward[1]) + initial_right_angle) / 2;
		float yaw_2_center = get_angle_tonorth(initial_line_up_point.stc_x - d_right_backward[0], initial_line_up_point.stc_y - d_right_backward[1]);
		point_1.stc_yaw = angplus(yaw_2_center, peer_2_yaw_center);
		point_1.stc_gimbalPitch = 0;
		point_1.stc_shootPhoto = 1;
		point_1.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(0);
		//if (m == 0) {
		//	temp_target_class_id.push_back(4);
		//}
		point_1.target_class_id = temp_target_class_id;
		line_out.push_back(point_1);
	}
	if (ChannelInspection == 1) {
		point_0.stc_x = initial_line_right_point.stc_x;
		point_0.stc_y = initial_line_right_point.stc_y;
		point_0.stc_z = initial_line_left_point.stc_z;
		point_0.stc_index = line_out.size() + 1;
		if (pow(initial_line_prev_point.stc_x, 2) + pow(initial_line_prev_point.stc_y, 2) <= 1)
		{
			float yaw_2_another = get_angle_tonorth(d_right_prev_perpendicular[0], d_right_prev_perpendicular[1]);
			point_0.stc_yaw = angplus(yaw_2_another, 8);
			//point_0.stc_yaw = get_angle_tonorth(d_right_prev_perpendicular[0], d_right_prev_perpendicular[1]);
		}
		else {
			//float yaw_2_another = get_angle_tonorth(initial_line_prev_point.stc_x - initial_line_up_point.stc_x, initial_line_prev_point.stc_y - initial_line_up_point.stc_y);
			//point_0.stc_yaw = angplus(yaw_2_another, 8);
			point_0.stc_yaw = get_angle_tonorth(initial_line_prev_point.stc_x - initial_line_up_point.stc_x, initial_line_prev_point.stc_y - initial_line_up_point.stc_y);
		}
		point_0.stc_gimbalPitch = 3;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.clear();
		temp_target_class_id.push_back(2);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
	}

	for (int m = layer_num - 1; m >= 0; m -=2)
	{
		//qDebug() << m;
		RouteIndexInfo point_1;
		point_1.stc_x = d_right_forward[0];
		point_1.stc_y = d_right_forward[1];
		point_1.stc_z = initial_line_right_point.stc_z + (1 - 2 * m) * unit_halfheight;
		point_1.stc_index = line_out.size() + 1;
		float initial_right_angle = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_right_point.stc_x, initial_line_up_point.stc_y - initial_line_right_point.stc_y);
		//point_1.stc_yaw = (get_angle_tonorth(initial_line_up_point.stc_x - d_right_forward[0], initial_line_up_point.stc_y - d_right_forward[1]) + initial_right_angle) / 2;
		float yaw_2_center = get_angle_tonorth(initial_line_up_point.stc_x - d_right_forward[0], initial_line_up_point.stc_y - d_right_forward[1]);
		point_1.stc_yaw = angplus(yaw_2_center, -peer_2_yaw_center);
		point_1.stc_gimbalPitch = 0;
		point_1.stc_shootPhoto = 1;
		point_1.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(0);
		point_1.target_class_id = temp_target_class_id;
		line_out.push_back(point_1);
	}

	point_0.stc_x = d_right_forward[0];
	point_0.stc_y = d_right_forward[1];
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_right_forward[0], initial_line_up_point.stc_y - d_right_forward[1]);
	point_0.stc_gimbalPitch = -45;
	point_0.stc_shootPhoto = 0;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	point_0.stc_x = d_left_forward[0];
	point_0.stc_y = d_left_forward[1];
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_left_forward[0], initial_line_up_point.stc_y - d_left_forward[1]);
	point_0.stc_gimbalPitch = -40;
	point_0.stc_shootPhoto = 1;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	for (int m = layer_num - 1; m >= 0; m -=2)
	{
		RouteIndexInfo point_1;
		point_1.stc_x = d_left_forward[0];
		point_1.stc_y = d_left_forward[1];
		point_1.stc_z = initial_line_left_point.stc_z + (2 * m - 1) * unit_halfheight;
		point_1.stc_index = line_out.size() + 1;
		float initial_left_angle = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_left_point.stc_x, initial_line_up_point.stc_y - initial_line_left_point.stc_y);
		//point_1.stc_yaw = (get_angle_tonorth(initial_line_up_point.stc_x - d_left_forward[0], initial_line_up_point.stc_x - d_left_forward[1]) + initial_left_angle) / 2;
		float yaw_2_center = get_angle_tonorth(initial_line_up_point.stc_x - d_left_forward[0], initial_line_up_point.stc_x - d_left_forward[1]);
		point_1.stc_yaw = angplus(yaw_2_center, peer_2_yaw_center);
		point_1.stc_gimbalPitch = 0;
		point_1.stc_shootPhoto = 1;
		point_1.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(0);
		//if (m == layer_num) {
		//	temp_target_class_id.push_back(4);
		//}
		point_1.target_class_id = temp_target_class_id;
		line_out.push_back(point_1);
	}
	point_0.stc_x = initial_line_left_point.stc_x;
	point_0.stc_y = initial_line_left_point.stc_y;
	point_0.stc_z = initial_line_left_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	if (pow(initial_line_next_point.stc_x, 2) + pow(initial_line_next_point.stc_y, 2) <= 1)
	{
		float yaw_2_another = get_angle_tonorth(-d_right_prev_perpendicular[0], -d_right_prev_perpendicular[1]);
		point_0.stc_yaw = angplus(yaw_2_another, 8);
		//point_0.stc_yaw = get_angle_tonorth(-d_right_prev_perpendicular[0], -d_right_prev_perpendicular[1]);
	}
	else {
		//float yaw_2_another = get_angle_tonorth(initial_line_next_point.stc_x - initial_line_up_point.stc_x, initial_line_next_point.stc_y - initial_line_up_point.stc_y);
		//point_0.stc_yaw = angplus(yaw_2_another, 8);
		point_0.stc_yaw = get_angle_tonorth(initial_line_next_point.stc_x - initial_line_up_point.stc_x, initial_line_next_point.stc_y - initial_line_up_point.stc_y);
	}
	point_0.stc_gimbalPitch = 4;
	point_0.stc_shootPhoto = 1;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(2);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);
	for (int m = 1; m <= layer_num; m +=2)
	{
		RouteIndexInfo point_1;
		point_1.stc_x = d_left_backward[0];
		point_1.stc_y = d_left_backward[1];
		point_1.stc_z = initial_line_left_point.stc_z + (2 * m - 1) * unit_halfheight;
		point_1.stc_index = line_out.size() + 1;
		float initial_left_angle = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_left_point.stc_x, initial_line_up_point.stc_y - initial_line_left_point.stc_y);
		//point_1.stc_yaw = (get_angle_tonorth(initial_line_up_point.stc_x - d_left_backward[0], initial_line_up_point.stc_x - d_left_backward[1]) + initial_left_angle) / 2;
		float yaw_2_center = get_angle_tonorth(initial_line_up_point.stc_x - d_left_backward[0], initial_line_up_point.stc_x - d_left_backward[1]);
		point_1.stc_yaw = angplus(yaw_2_center, -peer_2_yaw_center);
		point_1.stc_gimbalPitch = 0;
		point_1.stc_shootPhoto = 1;
		point_1.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(0);
		if (m == layer_num) {
			temp_target_class_id.push_back(4);
		}
		point_1.target_class_id = temp_target_class_id;
		line_out.push_back(point_1);
	}

	point_0.stc_x = d_left_backward[0];
	point_0.stc_y = d_left_backward[1];
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_left_backward[0], initial_line_up_point.stc_y - d_left_backward[1]);
	point_0.stc_gimbalPitch = -45;
	point_0.stc_shootPhoto = 0;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);
	//以上3行杆塔垂直角度获取
	point_0.stc_x = initial_line_up_point.stc_x;
	point_0.stc_y = initial_line_up_point.stc_y;
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(d_right_prev_perpendicular[0], d_right_prev_perpendicular[1]);
	point_0.stc_gimbalPitch = -90;
	point_0.stc_shootPhoto = 0;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(2);
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	if (pow(initial_line_next_point.stc_x, 2) + pow(initial_line_next_point.stc_y, 2) <= 1)
	{
		point_0.stc_x = initial_line_up_point.stc_x + longlook_whole_tower_distance * d_left_next_perpendicular[0];
		point_0.stc_y = initial_line_up_point.stc_y + longlook_whole_tower_distance * d_left_next_perpendicular[1];
		point_0.stc_z = initial_line_up_point.stc_z + 10;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -45;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.clear();
		temp_target_class_id.push_back(1);
		//temp_target_class_id.push_back(5);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);

		//////////////////////调整机头至下一座塔
		/*
		point_0.stc_x = initial_line_up_point.stc_x + (longlook_whole_tower_distance + 10) * d_left_next_perpendicular[0];
		point_0.stc_y = initial_line_up_point.stc_y + (longlook_whole_tower_distance + 10) * d_left_next_perpendicular[1];
		point_0.stc_z = initial_line_up_point.stc_z + 10 * (longlook_whole_tower_distance + 10) / longlook_whole_tower_distance;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = 1-get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -45;
		point_0.stc_shootPhoto = 0;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.clear();
		temp_target_class_id.push_back(1);
		//temp_target_class_id.push_back(5);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
		*/

	}
	else
	{
		vector<double> unit_next_directon = get_unit_directon3d(initial_line_up_point.stc_x, initial_line_up_point.stc_y, initial_line_up_point.stc_z, initial_line_next_point.stc_x, initial_line_next_point.stc_y, initial_line_next_point.stc_z);
		point_0.stc_x = initial_line_up_point.stc_x + longlook_whole_tower_distance * unit_next_directon[0];
		point_0.stc_y = initial_line_up_point.stc_y + longlook_whole_tower_distance * unit_next_directon[1];
		point_0.stc_z = initial_line_up_point.stc_z + longlook_whole_tower_distance * unit_next_directon[2];
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_next_point.stc_x, initial_line_up_point.stc_y - initial_line_next_point.stc_y);
		point_0.stc_gimbalPitch = -38;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.clear();
		temp_target_class_id.push_back(1);
		//temp_target_class_id.push_back(5);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);

		//////////////////////调整机头至下一座塔
		/*
		point_0.stc_x = initial_line_up_point.stc_x + (longlook_whole_tower_distance + 10) * unit_next_directon[0];
		point_0.stc_y = initial_line_up_point.stc_y + (longlook_whole_tower_distance + 10) * unit_next_directon[1];
		point_0.stc_z = initial_line_up_point.stc_z + (longlook_whole_tower_distance + 10) * unit_next_directon[2];
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = 1-get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -45;
		point_0.stc_shootPhoto = 0;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.clear();;
		temp_target_class_id.push_back(1);
		//temp_target_class_id.push_back(5);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
		*/
	}
	return line_out;
}
////////////////////////////倒U一侧单次
std::vector<RouteIndexInfo> CPointToLine::pointToLine_OU_1(RecPoint input)
{
	vector<RouteIndexInfo> line_out;
	line_out.clear();
	DatumPoint initial_line_up_point = input.rec_pt[0];
	DatumPoint initial_line_right_point = input.rec_pt[1];
	DatumPoint initial_line_left_point = input.rec_pt[2];
	DatumPoint initial_line_prev_point = input.prev_pt;
	DatumPoint initial_line_next_point = input.next_pt;

	int layer_num = input.layer_num;
	int left_yaw_angle = input.left_angle;
	int right_yaw_angle = input.right_angle;
	int ChannelInspection = input.ChannelInspection;
	if (layer_num == 0)
	{
		return line_out;
	}
	if (initial_line_right_point.stc_x == 0 && initial_line_right_point.stc_y == 0)
	{
		return line_out;
	}
	if (initial_line_left_point.stc_x == 0 && initial_line_left_point.stc_y == 0)
	{
		return line_out;
	}
	int yaw_rotate_angle_selected = input.yaw_rotate_angle_selected;
	//double longlook_whole_tower_distance = input.longlook_whole_tower_distance;
	double longlook_whole_tower_distance = input.height;

	double dis_right = get_distance2d(initial_line_right_point.stc_x, initial_line_right_point.stc_y, initial_line_up_point.stc_x, initial_line_up_point.stc_y);
	double dis_left = get_distance2d(initial_line_left_point.stc_x, initial_line_left_point.stc_y, initial_line_up_point.stc_x, initial_line_up_point.stc_y);
	vector<double> unit_rightpoint = get_unit_directon2d(initial_line_right_point.stc_x, initial_line_right_point.stc_y, initial_line_up_point.stc_x, initial_line_up_point.stc_y);
	vector<double> unit_leftpoint = get_unit_directon2d(initial_line_left_point.stc_x, initial_line_left_point.stc_y, initial_line_up_point.stc_x, initial_line_up_point.stc_y);
	vector<double> d_right_forward = matrixrotate2d(unit_rightpoint, -right_yaw_angle);
	vector<double> d_right_backward = matrixrotate2d(unit_rightpoint, right_yaw_angle);
	vector<double> d_right_prev_perpendicular = matrixrotate2d(unit_rightpoint, 90);
	vector<double> d_left_forward = matrixrotate2d(unit_leftpoint, left_yaw_angle);
	vector<double> d_left_backward = matrixrotate2d(unit_leftpoint, -left_yaw_angle);
	vector<double> d_left_next_perpendicular = matrixrotate2d(unit_leftpoint, 90);

	d_right_forward[0] = -d_right_forward[0] * dis_right;
	d_right_backward[0] = -d_right_backward[0] * dis_right;
	d_left_forward[0] = -d_left_forward[0] * dis_left;
	d_left_backward[0] = -d_left_backward[0] * dis_left;
	d_right_forward[1] = -d_right_forward[1] * dis_right;
	d_right_backward[1] = -d_right_backward[1] * dis_right;
	d_left_forward[1] = -d_left_forward[1] * dis_left;
	d_left_backward[1] = -d_left_backward[1] * dis_left;
	double unit_halfheight = fabs((initial_line_right_point.stc_z - initial_line_left_point.stc_z) / (layer_num - 1) / 2);
	//前置判断3个点符不符合标准，，return 什么你自己定义一下，以后我再慢慢补充规则
	if (fabs(initial_line_up_point.stc_z - initial_line_right_point.stc_z) < unit_halfheight) {
		//目前暂定 当第一个点的index为-1时，则为不符合标准
		vector<RouteIndexInfo> buffer;
		RouteIndexInfo buffer_first_data;
		buffer_first_data.stc_index = -1;
		buffer.push_back(buffer_first_data);
		return buffer;
	}
	//报错绝缘子层数设置异常
	if (layer_num == 0 || layer_num > 4) {
		vector<RouteIndexInfo> buffer;
		RouteIndexInfo buffer_first_data;
		buffer_first_data.stc_index = -1;
		buffer.push_back(buffer_first_data);
		return buffer;
	}
	RouteIndexInfo point_0;
	if (pow(initial_line_prev_point.stc_x, 2) + pow(initial_line_prev_point.stc_y, 2) <= 1)
	{
		point_0.stc_x = initial_line_up_point.stc_x + longlook_whole_tower_distance * d_right_prev_perpendicular[0];
		point_0.stc_y = initial_line_up_point.stc_y + longlook_whole_tower_distance * d_right_prev_perpendicular[1];
		point_0.stc_z = initial_line_up_point.stc_z + 10;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -40;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(1);
		//temp_target_class_id.push_back(3);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
		///////悬停刹车专用
		point_0.stc_x = initial_line_up_point.stc_x + 10* d_right_prev_perpendicular[0];
		point_0.stc_y = initial_line_up_point.stc_y + 10* d_right_prev_perpendicular[1];
		point_0.stc_z = initial_line_up_point.stc_z;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -90;
		point_0.stc_shootPhoto = 0;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.push_back(2);
		//temp_target_class_id.push_back(3);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);

	}
	else
	{
		vector<double> unit_prev_directon = get_unit_directon3d(initial_line_up_point.stc_x, initial_line_up_point.stc_y, initial_line_up_point.stc_z, initial_line_prev_point.stc_x, initial_line_prev_point.stc_y, initial_line_prev_point.stc_z);
		if (ChannelInspection == 1) {
			double dis_between_2_tower = get_distance3d(initial_line_up_point.stc_x, initial_line_up_point.stc_y, initial_line_up_point.stc_z, initial_line_prev_point.stc_x, initial_line_prev_point.stc_y, initial_line_prev_point.stc_z);
			double quater_dis_between_2_tower = dis_between_2_tower / 4;
			for (int iii = 3; iii > 1; iii--) {
				point_0.stc_x = initial_line_up_point.stc_x + iii * quater_dis_between_2_tower * unit_prev_directon[0];
				point_0.stc_y = initial_line_up_point.stc_y + iii * quater_dis_between_2_tower * unit_prev_directon[1];
				point_0.stc_z = initial_line_up_point.stc_z + iii * quater_dis_between_2_tower * unit_prev_directon[2];
				point_0.stc_index = line_out.size() + 1;
				point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_prev_point.stc_x, initial_line_up_point.stc_y - initial_line_prev_point.stc_y);
				point_0.stc_gimbalPitch = -45;
				point_0.stc_shootPhoto = 1;
				point_0.stc_opticalZoomFocalLength = 1;
				std::vector<int> temp_target_class_id_tep;
				temp_target_class_id_tep.push_back(2);
				point_0.target_class_id = temp_target_class_id_tep;
				line_out.push_back(point_0);
			}
		}
		point_0.stc_x = initial_line_up_point.stc_x + longlook_whole_tower_distance * unit_prev_directon[0];
		point_0.stc_y = initial_line_up_point.stc_y + longlook_whole_tower_distance * unit_prev_directon[1];
		point_0.stc_z = initial_line_up_point.stc_z + longlook_whole_tower_distance * unit_prev_directon[2];
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_prev_point.stc_x, initial_line_up_point.stc_y - initial_line_prev_point.stc_y);
		point_0.stc_gimbalPitch = -35;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(1);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);

		///////悬停刹车专用
		point_0.stc_x = initial_line_up_point.stc_x + 10 * unit_prev_directon[0];
		point_0.stc_y = initial_line_up_point.stc_y + 10 * unit_prev_directon[1];
		point_0.stc_z = initial_line_up_point.stc_z;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -90;
		point_0.stc_shootPhoto = 0;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.push_back(2);
		//temp_target_class_id.push_back(3);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
	}

	point_0.stc_x = initial_line_up_point.stc_x;
	point_0.stc_y = initial_line_up_point.stc_y;
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(d_left_next_perpendicular[0], d_left_next_perpendicular[1]);
	point_0.stc_gimbalPitch = -90;
	point_0.stc_shootPhoto = 1;
	point_0.stc_opticalZoomFocalLength = 1;
	std::vector<int> temp_target_class_id;
	temp_target_class_id.push_back(2);
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	point_0.stc_x = d_right_backward[0];
	point_0.stc_y = d_right_backward[1];
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_right_backward[0], initial_line_up_point.stc_y - d_right_backward[1]);
	point_0.stc_gimbalPitch = -40;
	point_0.stc_shootPhoto = 1;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);
	/*
	for (int m = 0; m < layer_num; m++)
	{
		RouteIndexInfo point_1;
		point_1.stc_x = d_right_backward[0];
		point_1.stc_y = d_right_backward[1];
		point_1.stc_z = initial_line_right_point.stc_z + (1 - 2 * m) * unit_halfheight;
		point_1.stc_index = line_out.size() + 1;
		point_1.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_right_backward[0], initial_line_up_point.stc_y - d_right_backward[1]);
		point_1.stc_gimbalPitch = 0;
		point_1.stc_shootPhoto = 1;
		point_1.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(0);
		//if (m == 0) {
		//	temp_target_class_id.push_back(4);
		//}
		point_1.target_class_id = temp_target_class_id;
		line_out.push_back(point_1);
	}
	*/

	for (int m = 0; m < layer_num; m+=2)
	{
		RouteIndexInfo point_1;
		point_1.stc_x = d_right_backward[0];
		point_1.stc_y = d_right_backward[1];
		point_1.stc_z = initial_line_right_point.stc_z + (1 - 2 * m) * unit_halfheight;
		point_1.stc_index = line_out.size() + 1;
		point_1.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_right_backward[0], initial_line_up_point.stc_y - d_right_backward[1]);
		point_1.stc_gimbalPitch = 0;
		if (m != 0) {
			point_1.stc_gimbalPitch = 6;
		}
		point_1.stc_shootPhoto = 1;
		point_1.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(0);
		//if (m == 0) {
		//	temp_target_class_id.push_back(4);
		//}
		point_1.target_class_id = temp_target_class_id;
		line_out.push_back(point_1);
	}


	if (ChannelInspection == 1) {
		point_0.stc_x = initial_line_right_point.stc_x;
		point_0.stc_y = initial_line_right_point.stc_y;
		point_0.stc_z = initial_line_left_point.stc_z;
		point_0.stc_index = line_out.size() + 1;
		if (pow(initial_line_prev_point.stc_x, 2) + pow(initial_line_prev_point.stc_y, 2) <= 1)
		{
			float yaw_2_another = get_angle_tonorth(d_right_prev_perpendicular[0], d_right_prev_perpendicular[1]);
			point_0.stc_yaw = angplus(yaw_2_another, 8);
			//point_0.stc_yaw = get_angle_tonorth(d_right_prev_perpendicular[0], d_right_prev_perpendicular[1]);
		}
		else {
			//float yaw_2_another = get_angle_tonorth(initial_line_prev_point.stc_x - initial_line_up_point.stc_x, initial_line_prev_point.stc_y - initial_line_up_point.stc_y);
			//point_0.stc_yaw = angplus(yaw_2_another, 8);
			point_0.stc_yaw = get_angle_tonorth(initial_line_prev_point.stc_x - initial_line_up_point.stc_x, initial_line_prev_point.stc_y - initial_line_up_point.stc_y);
		}
		point_0.stc_gimbalPitch = 3;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.clear();
		temp_target_class_id.push_back(2);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
	}

	point_0.stc_x = d_right_forward[0];
	point_0.stc_y = d_right_forward[1];
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_right_forward[0], initial_line_up_point.stc_y - d_right_forward[1]);
	point_0.stc_gimbalPitch = -40;
	point_0.stc_shootPhoto = 0;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	point_0.stc_x = d_left_forward[0];
	point_0.stc_y = d_left_forward[1];
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_left_forward[0], initial_line_up_point.stc_y - d_left_forward[1]);
	point_0.stc_gimbalPitch = -40;
	point_0.stc_shootPhoto = 1;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	for (int m = layer_num; m > 0; m-=2)
	{
		RouteIndexInfo point_1;
		point_1.stc_x = d_left_forward[0];
		point_1.stc_y = d_left_forward[1];
		point_1.stc_z = initial_line_left_point.stc_z + (2 * m - 1) * unit_halfheight;
		point_1.stc_index = line_out.size() + 1;
		point_1.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_left_forward[0], initial_line_up_point.stc_x - d_left_forward[1]);
		point_1.stc_gimbalPitch = 0;
		if (m != layer_num) {
			point_1.stc_gimbalPitch = 6;
		}
		point_1.stc_shootPhoto = 1;
		point_1.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(0);
		//if (m == layer_num) {
		//	temp_target_class_id.push_back(4);
		//}
		point_1.target_class_id = temp_target_class_id;
		line_out.push_back(point_1);
	}
	if (ChannelInspection == 1) {
		point_0.stc_x = initial_line_left_point.stc_x;
		point_0.stc_y = initial_line_left_point.stc_y;
		point_0.stc_z = initial_line_left_point.stc_z;
		point_0.stc_index = line_out.size() + 1;
		if (pow(initial_line_next_point.stc_x, 2) + pow(initial_line_next_point.stc_y, 2) <= 1)
		{
			float yaw_2_another = get_angle_tonorth(-d_right_prev_perpendicular[0], -d_right_prev_perpendicular[1]);
			point_0.stc_yaw = angplus(yaw_2_another, 8);
			//point_0.stc_yaw = get_angle_tonorth(-d_right_prev_perpendicular[0], -d_right_prev_perpendicular[1]);
		}
		else {
			//float yaw_2_another = get_angle_tonorth(initial_line_next_point.stc_x - initial_line_up_point.stc_x, initial_line_next_point.stc_y - initial_line_up_point.stc_y);
			//point_0.stc_yaw = angplus(yaw_2_another, 8);
			point_0.stc_yaw = get_angle_tonorth(initial_line_next_point.stc_x - initial_line_up_point.stc_x, initial_line_next_point.stc_y - initial_line_up_point.stc_y);
		}
		point_0.stc_gimbalPitch = 4;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.clear();
		temp_target_class_id.push_back(2);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
	}
	point_0.stc_x = d_left_backward[0];
	point_0.stc_y = d_left_backward[1];
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - d_left_backward[0], initial_line_up_point.stc_y - d_left_backward[1]);
	point_0.stc_gimbalPitch = -40;
	point_0.stc_shootPhoto = 0;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);
	//以上3行杆塔垂直角度获取
	point_0.stc_x = initial_line_up_point.stc_x;
	point_0.stc_y = initial_line_up_point.stc_y;
	point_0.stc_z = initial_line_up_point.stc_z;
	point_0.stc_index = line_out.size() + 1;
	point_0.stc_yaw = get_angle_tonorth(d_right_prev_perpendicular[0], d_right_prev_perpendicular[1]);
	point_0.stc_gimbalPitch = -90;
	point_0.stc_shootPhoto = 0;
	point_0.stc_opticalZoomFocalLength = 1;
	temp_target_class_id.clear();
	temp_target_class_id.push_back(2);
	temp_target_class_id.push_back(4);
	point_0.target_class_id = temp_target_class_id;
	line_out.push_back(point_0);

	if (pow(initial_line_next_point.stc_x, 2) + pow(initial_line_next_point.stc_y, 2) <= 1)
	{
		point_0.stc_x = initial_line_up_point.stc_x + longlook_whole_tower_distance * d_left_next_perpendicular[0];
		point_0.stc_y = initial_line_up_point.stc_y + longlook_whole_tower_distance * d_left_next_perpendicular[1];
		point_0.stc_z = initial_line_up_point.stc_z + 10;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -40;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		std::vector<int> temp_target_class_id;
		temp_target_class_id.push_back(1);
		//temp_target_class_id.push_back(5);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);

		//////////////////////调整机头至下一座塔
		point_0.stc_x = initial_line_up_point.stc_x + (longlook_whole_tower_distance + 10) * d_left_next_perpendicular[0];
		point_0.stc_y = initial_line_up_point.stc_y + (longlook_whole_tower_distance + 10) * d_left_next_perpendicular[1];
		point_0.stc_z = initial_line_up_point.stc_z + 10 * (longlook_whole_tower_distance + 10) / longlook_whole_tower_distance;
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = 1 - get_angle_tonorth(initial_line_up_point.stc_x - point_0.stc_x, initial_line_up_point.stc_y - point_0.stc_y);
		point_0.stc_gimbalPitch = -45;
		point_0.stc_shootPhoto = 0;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.clear();
		temp_target_class_id.push_back(1);
		//temp_target_class_id.push_back(5);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
	}
	else
	{
		vector<double> unit_next_directon = get_unit_directon3d(initial_line_up_point.stc_x, initial_line_up_point.stc_y, initial_line_up_point.stc_z, initial_line_next_point.stc_x, initial_line_next_point.stc_y, initial_line_next_point.stc_z);
		point_0.stc_x = initial_line_up_point.stc_x + longlook_whole_tower_distance * unit_next_directon[0];
		point_0.stc_y = initial_line_up_point.stc_y + longlook_whole_tower_distance * unit_next_directon[1];
		point_0.stc_z = initial_line_up_point.stc_z + longlook_whole_tower_distance * unit_next_directon[2];
		point_0.stc_index = line_out.size() + 1;
		point_0.stc_yaw = get_angle_tonorth(initial_line_up_point.stc_x - initial_line_next_point.stc_x, initial_line_up_point.stc_y - initial_line_next_point.stc_y);
		point_0.stc_gimbalPitch = -35;
		point_0.stc_shootPhoto = 1;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.clear();
		temp_target_class_id.push_back(1);
		//temp_target_class_id.push_back(5);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);

		//////////////////////调整机头至下一座塔
		point_0.stc_x = initial_line_up_point.stc_x + (longlook_whole_tower_distance + 10) * unit_next_directon[0];
		point_0.stc_y = initial_line_up_point.stc_y + (longlook_whole_tower_distance + 10) * unit_next_directon[1];
		point_0.stc_z = initial_line_up_point.stc_z + (longlook_whole_tower_distance + 10) * unit_next_directon[2];
		point_0.stc_index = line_out.size() + 1;
		if (pow(initial_line_next_point.stc_x, 2) + pow(initial_line_next_point.stc_y, 2) <= 1)
		{
			float yaw_2_another = get_angle_tonorth(-d_right_prev_perpendicular[0], -d_right_prev_perpendicular[1]);
			point_0.stc_yaw = yaw_2_another;
			//point_0.stc_yaw = get_angle_tonorth(-d_right_prev_perpendicular[0], -d_right_prev_perpendicular[1]);
		}
		else {
			//float yaw_2_another = get_angle_tonorth(initial_line_next_point.stc_x - initial_line_up_point.stc_x, initial_line_next_point.stc_y - initial_line_up_point.stc_y);
			//point_0.stc_yaw = angplus(yaw_2_another, 8);
			point_0.stc_yaw = get_angle_tonorth(initial_line_next_point.stc_x - initial_line_up_point.stc_x, initial_line_next_point.stc_y - initial_line_up_point.stc_y);
		}
		point_0.stc_yaw = point_0.stc_yaw - 1;
		point_0.stc_gimbalPitch = -45;
		point_0.stc_shootPhoto = 0;
		point_0.stc_opticalZoomFocalLength = 1;
		temp_target_class_id.clear();;
		temp_target_class_id.push_back(1);
		//temp_target_class_id.push_back(5);
		point_0.target_class_id = temp_target_class_id;
		line_out.push_back(point_0);
	}
	return line_out;
}




std::vector<WaypointInformation> CPointToLine::route_to_osdkroute(CommissionType2Post tempcommissionType2Post) {
	std::vector<WaypointInformation> vector_commute_task;
	QString temp_poi = "";
	for (int i = 0; i < tempcommissionType2Post.stc_fplist.size(); i++)
	{
		WaypointInformation temp_commuteTask;
		CDatumPoint m_CDatumPoint;

		//转化存在问题
		for (int j = 0; j < tempcommissionType2Post.stc_fplist[i].target_class_id.size(); j++)
		{
			temp_commuteTask.target_class_id.push_back(tempcommissionType2Post.stc_fplist[i].target_class_id[j]);
			//temp_poi += QString::number(temp_commuteTask.targets[j]);
			//temp_poi += "  ";
		}

		CruiseInstance temp_CruiseInstance;
		QString url = temp_CruiseInstance.read_url_xml("MAXSPEED");
		temp_commuteTask.maxFlightSpeed = url.toDouble();
		temp_commuteTask.stc_x = tempcommissionType2Post.stc_fplist[i].stc_gps.stc_lon;
		temp_commuteTask.stc_y = tempcommissionType2Post.stc_fplist[i].stc_gps.stc_lat;
		temp_commuteTask.stc_z = tempcommissionType2Post.stc_fplist[i].stc_gps.stc_alt;

		temp_commuteTask.stc_yaw = tempcommissionType2Post.stc_fplist[i].stc_yaw;
		temp_commuteTask.stc_gimbalPitch = tempcommissionType2Post.stc_fplist[i].stc_gimbalPitch;


		



		
		//////////////////////////////////////////////////////////////////////kevin///////////////////////////////////

		//其他点
		m_CDatumPoint.setOrigin(tempcommissionType2Post.stc_fplist[i].stc_gps.stc_lat, tempcommissionType2Post.stc_fplist[i].stc_gps.stc_lon, tempcommissionType2Post.stc_fplist[i].stc_gps.stc_alt, true);
		Coordinate aaaa;
		aaaa.stc_x = 30 * sin(ang2rad(temp_commuteTask.stc_yaw));
		aaaa.stc_y = 30 * cos(ang2rad(temp_commuteTask.stc_yaw));
		aaaa.stc_z = 30 * tan(ang2rad(temp_commuteTask.stc_gimbalPitch));

		LongitudeAndLatitude daaaa = m_CDatumPoint.DatumPointToLongitudeAndLatitude(aaaa);
		double calculate_interest_point_lon = daaaa.stc_longitude;
		double calculate_interest_point_lat = daaaa.stc_latitude;
		
		//double calculate_interest_point_lon = tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lon;
		//double calculate_interest_point_lat = tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lat;
		double calculate_interest_point_height;
		if (tempcommissionType2Post.stc_fplist[i].stc_gimbalPitch == 0) {
			calculate_interest_point_height = tempcommissionType2Post.stc_fplist[i].stc_gps.stc_alt;
		}
		else {
			calculate_interest_point_height = tempcommissionType2Post.stc_fplist[i].stc_gps.stc_alt + aaaa.stc_z;
		}

		//通道巡检，右侧以最后的点作为interest point
		if (tempcommissionType2Post.stc_fplist[i].stc_gimbalPitch == 3) {
			//calculate_interest_point_lon = tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lon + (tempcommissionType2Post.stc_fplist[0].stc_gps.stc_lon - tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lon) * 3;
			//calculate_interest_point_lat = tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lat + (tempcommissionType2Post.stc_fplist[0].stc_gps.stc_lat - tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lat) * 3;
			aaaa.stc_x = 300 * sin(ang2rad(temp_commuteTask.stc_yaw));
			aaaa.stc_y = 300 * cos(ang2rad(temp_commuteTask.stc_yaw));
			aaaa.stc_z = 300 * sin(ang2rad(temp_commuteTask.stc_gimbalPitch));

			daaaa = m_CDatumPoint.DatumPointToLongitudeAndLatitude(aaaa);
			calculate_interest_point_lon = daaaa.stc_longitude;
			calculate_interest_point_lat = daaaa.stc_latitude;
			calculate_interest_point_height = tempcommissionType2Post.stc_fplist[i].stc_gps.stc_alt;
		}
		//通道巡检，左侧以第一个点作为interest point
		if (tempcommissionType2Post.stc_fplist[i].stc_gimbalPitch == 4) {
			//calculate_interest_point_lon = tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lon + (tempcommissionType2Post.stc_fplist[tempcommissionType2Post.stc_fplist.size() - 1].stc_gps.stc_lon - tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lon) * 3;
			//calculate_interest_point_lat = tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lat + (tempcommissionType2Post.stc_fplist[tempcommissionType2Post.stc_fplist.size() - 1].stc_gps.stc_lat - tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lat) * 3;
			
			aaaa.stc_x = 300 * sin(ang2rad(temp_commuteTask.stc_yaw));
			aaaa.stc_y = 300 * cos(ang2rad(temp_commuteTask.stc_yaw));
			aaaa.stc_z = 300 * sin(ang2rad(temp_commuteTask.stc_gimbalPitch));

			daaaa = m_CDatumPoint.DatumPointToLongitudeAndLatitude(aaaa);
			calculate_interest_point_lon = daaaa.stc_longitude;
			calculate_interest_point_lat = daaaa.stc_latitude;
			calculate_interest_point_height = tempcommissionType2Post.stc_fplist[i].stc_gps.stc_alt;
		}
		//正上方，最后一个点的方向为interest point
		if (tempcommissionType2Post.stc_fplist[i].stc_gimbalPitch == -90) {
			//calculate_interest_point_lon = tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lon + (tempcommissionType2Post.stc_fplist[tempcommissionType2Post.stc_fplist.size() - 1].stc_gps.stc_lon - tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lon) * 50;
			//calculate_interest_point_lat = tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lat + (tempcommissionType2Post.stc_fplist[tempcommissionType2Post.stc_fplist.size() - 1].stc_gps.stc_lat - tempcommissionType2Post.stc_fplist[1].stc_gps.stc_lat) * 50;
			
			aaaa.stc_x = 3000 * sin(ang2rad(temp_commuteTask.stc_yaw));
			aaaa.stc_y = 3000 * cos(ang2rad(temp_commuteTask.stc_yaw));
			aaaa.stc_z = sin(ang2rad(temp_commuteTask.stc_gimbalPitch));

			daaaa = m_CDatumPoint.DatumPointToLongitudeAndLatitude(aaaa);
			calculate_interest_point_lon = daaaa.stc_longitude;
			calculate_interest_point_lat = daaaa.stc_latitude;

			calculate_interest_point_height = tempcommissionType2Post.stc_fplist[i].stc_gps.stc_alt -30;
		}
		

		LongitudeAndLatitude calculate_interest_point;
		calculate_interest_point.stc_altitude = calculate_interest_point_height;
		calculate_interest_point.stc_latitude = calculate_interest_point_lat;
		calculate_interest_point.stc_longitude = calculate_interest_point_lon;
		m_CDatumPoint.setOrigin(tempcommissionType2Post.stc_fplist[i].stc_gps.stc_lat, tempcommissionType2Post.stc_fplist[i].stc_gps.stc_lon, tempcommissionType2Post.stc_fplist[i].stc_gps.stc_alt, true);
		////latitude can calculate y
		temp_commuteTask.POI_Y = m_CDatumPoint.longitudeAndLatitudeToDatumPoint(calculate_interest_point).stc_y;
		////longitude can calculate x
		temp_commuteTask.POI_X = m_CDatumPoint.longitudeAndLatitudeToDatumPoint(calculate_interest_point).stc_x;
		////relativeHeight can calculate z
		temp_commuteTask.POI_Z = m_CDatumPoint.longitudeAndLatitudeToDatumPoint(calculate_interest_point).stc_z;
		if (tempcommissionType2Post.stc_fplist[i].stc_shootPhoto == 0) {
			temp_commuteTask.POI_Z = 1;
		}
		vector_commute_task.push_back(temp_commuteTask);
	}

	return vector_commute_task;
}
