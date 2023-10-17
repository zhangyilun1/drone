#include "CoordinateSystem.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

CoordinateSystem::CoordinateSystem() {

}

CoordinateSystem::~CoordinateSystem() {

}

bool CoordinateSystem::outOfChina(double lat, double lon) {
	return !(lon > 73.66 && lon < 135.05 && lat > 3.86 && lat < 53.55);
}

// WGS84=>BD09 ��������ϵ=>�ٶ�����ϵ 
int CoordinateSystem::wgs2bd(double lat, double lon, double* pLat, double* pLon) {
	double lat_ = 0.0, lon_ = 0.0;
	wgs2gcj(lat, lon, &lat_, &lon_);
	gcj2bd(lat_, lon_, pLat, pLon);
	return 0;
}

// GCJ02=>BD09 ��������ϵ=>�ٶ�����ϵ  
int CoordinateSystem::gcj2bd(double lat, double lon, double* pLat, double* pLon) {
	double x = lon, y = lat;
	double z = sqrt(x * x + y * y) + 0.00002 * sin(y * x_pi);
	double theta = atan2(y, x) + 0.000003 * cos(x * x_pi);
	*pLon = z * cos(theta) + 0.0065;
	*pLat = z * sin(theta) + 0.006;
	return 0;
}

// BD09=>GCJ02 �ٶ�����ϵ=>��������ϵ 
int CoordinateSystem::bd2gcj(double lat, double lon, double* pLat, double* pLon) {
	double x = lon - 0.0065, y = lat - 0.006;
	double z = sqrt(x * x + y * y) - 0.00002 * sin(y * x_pi);
	double theta = atan2(y, x) - 0.000003 * cos(x * x_pi);
	*pLon = z * cos(theta);
	*pLat = z * sin(theta);
	return 0;
}

// WGS84=>GCJ02 ��������ϵ=>��������ϵ
int CoordinateSystem::wgs2gcj(double lat, double lon, double* pLat, double* pLon) {
	if (outOfChina(lat, lon))
	{
		*pLat = lat;
		*pLon = lon;
		return 0;
	}
	double dLat = transformLat(lon - 105.0, lat - 35.0);
	double dLon = transformLon(lon - 105.0, lat - 35.0);
	double radLat = lat / 180.0 * pi;
	double magic = sin(radLat);
	magic = 1 - ee * magic * magic;
	double sqrtMagic = sqrt(magic);
	dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
	dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
	*pLat = lat + dLat;
	*pLon = lon + dLon;
	return 0;
}

// GCJ02=>WGS84 ��������ϵ=>��������ϵ(����)
int CoordinateSystem::gcj2wgs(double lat, double lon, double* pLat, double* pLon) {
	if (outOfChina(lat, lon))
	{
		*pLat = lat;
		*pLon = lon;
		return 0;
	}
	double *offset;
	offset = OffSet(lat, lon);
	*pLat = lat - offset[0];
	*pLon = lon - offset[1];
	return 0;
}

// GCJ02=>WGS84 ��������ϵ=>��������ϵ����ȷ��
int CoordinateSystem::gcj2wgs_Exactly(double gcjlat, double gcjlon, double* wgs_Lat, double* wgs_lon) {
	if (outOfChina(gcjlat, gcjlon))
	{
		*wgs_Lat = gcjlat;
		*wgs_lon = gcjlon;
		return 0;
	}
	double initDelta = 0.01;
	double threshold = 0.000000001;
	double dLat = initDelta, dLon = initDelta;
	double mLat = gcjlat - dLat, mLon = gcjlon - dLon;
	double pLat = gcjlat + dLat, pLon = gcjlon + dLon;
	double wgsLat = 0.0, wgslon = 0.0, i = 0.0, newgcjlat = 0.0, newgcjlon = 0.0;

	while (true) {
		wgsLat = (mLat + pLat) / 2;
		wgslon = (mLon + pLon) / 2;
		wgs2gcj(wgsLat, wgslon, &newgcjlat, &newgcjlon);
		dLon = newgcjlon - gcjlon;
		dLat = newgcjlat - gcjlat;
		if ((fabs(dLat) < threshold) && (fabs(dLon) < threshold))
			break;

		if (dLat > 0)
			pLat = wgsLat;
		else
			mLat = wgsLat;
		if (dLon > 0)
			pLon = wgslon;
		else
			mLon = wgslon;

		if (++i > 10000)
			break;
	}
	*wgs_Lat = wgsLat;
	*wgs_lon = wgslon;
	return 0;
}

// BD09=>WGS84 �ٶ�����ϵ=>��������ϵ(����)
int CoordinateSystem::bd2wgs(double lat, double lon, double* pLat, double* pLon) {
	double lat_ = 0.0, lon_ = 0.0;
	bd2gcj(lat, lon, &lat_, &lon_);
	gcj2wgs(lat_, lon_, pLat, pLon);
	return 0;
}

// BD09=>WGS84 �ٶ�����ϵ=>��������ϵ(��ȷ)
int CoordinateSystem::bd2wgs_Exactly(double lat, double lon, double* pLat, double* pLon) {
	double lat_ = 0.0, lon_ = 0.0;
	bd2gcj(lat, lon, &lat_, &lon_);
	gcj2wgs_Exactly(lat_, lon_, pLat, pLon);
	return 0;
}



// ƫ����
double * CoordinateSystem::OffSet(double lat, double lon) {
	double Latlon[2] = { 0.0,0.0 };
	double dLat = transformLat(lon - 105.0, lat - 35.0);
	double dLon = transformLon(lon - 105.0, lat - 35.0);
	double radLat = lat / 180.0 * pi;
	double magic = sin(radLat);
	magic = 1 - ee * magic * magic;
	double sqrtMagic = sqrt(magic);
	dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
	dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
	Latlon[0] = dLat;
	Latlon[1] = dLon;
	return Latlon;
}

// γ��ƫ����
double CoordinateSystem::transformLat(double x, double y) {
	double ret = 0.0;
	ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(fabs(x));
	ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
	ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
	ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;
	return ret;
}

// ����ƫ����
double CoordinateSystem::transformLon(double x, double y) {
	double ret = 0.0;
	ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(fabs(x));
	ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
	ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
	ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
	return ret;
}

//WGS84���� 	120.1067916,30.3430350
//GCJ02���� 	120.111599,30.340772
//BD09����	 	120.118000,30.347094
