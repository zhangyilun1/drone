#pragma once
class CoordinateSystem
{
public:
	CoordinateSystem();
	~CoordinateSystem();

	static CoordinateSystem* GetCoordinateSystem()
	{
		static CoordinateSystem cs;
		return &cs;
	}
	bool outOfChina(double lat, double lon);
	int wgs2bd(double lat, double lon, double* pLat, double* pLon); // WGS84=>BD09 ��������ϵ=>�ٶ�����ϵ 
	int gcj2bd(double lat, double lon, double* pLat, double* pLon);	// GCJ02=>BD09 ��������ϵ=>�ٶ�����ϵ 
	int bd2gcj(double lat, double lon, double* pLat, double* pLon);	// BD09=>GCJ02 �ٶ�����ϵ=>��������ϵ 
	int wgs2gcj(double lat, double lon, double* pLat, double* pLon);// WGS84=>GCJ02 ��������ϵ=>��������ϵ
	int gcj2wgs(double lat, double lon, double* pLat, double* pLon);// GCJ02=>WGS84 ��������ϵ=>��������ϵ(����)
	int bd2wgs(double lat, double lon, double* pLat, double* pLon);	// BD09=>WGS84 �ٶ�����ϵ=>��������ϵ(����)
	int gcj2wgs_Exactly(double lat, double lon, double* wgs_Lat, double* wgs_lon);// GCJ02=>WGS84 ��������ϵ=>��������ϵ����ȷ��
	int bd2wgs_Exactly(double lat, double lon, double* pLat, double* pLon);// BD09=>WGS84 �ٶ�����ϵ=>��������ϵ(��ȷ)
	double *OffSet(double lat, double lon);	// ƫ����
	double transformLat(double x, double y);// γ��ƫ����
	double transformLon(double x, double y);// ����ƫ����

private:
	/*
	 pi: Բ���ʡ�
	 a: ������������ͶӰ��ƽ���ͼ����ϵ��ͶӰ���ӡ�
	 ee: �����ƫ���ʡ�
	 x_pi: Բ����ת������
	*/
	double pi = 3.14159265358979324;
	double a = 6378245.0;
	double ee = 0.00669342162296594323;
	double x_pi = 3.14159265358979324 * 3000.0 / 180.0;
};

