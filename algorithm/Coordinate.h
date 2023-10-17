#pragma once
#include <math.h>
#include "../common.h"
/*
*�÷���
*	step1:
*				�ȵ��� setOrigin() ����ԭ��� ��γ�Ⱥ��� ����Բ���xyz���꣨���һ�������������������Ǿ�γ�Ȼ������꣩
*			���˷����������������Բ������ã�
*	
*	step2:
*				������Բ��ľ�γ���Լ����Σ������ setDatumPoint() ���� setLongitudeAndLatitude() ������Ŀ�����ľ�γ�Ȼ�������
*
*	step3:
*				�ڶ�������ɺ������������ݣ�����DatumPointToLongitudeAndLatitude() ����Ŀ������ ���� ת ��γ��
*			����longitudeAndLatitudeToDatumPoint() ����Ŀ������ ��γ�� ת ����
*
*/
class CDatumPoint
{
public:
	//����Ŀ��������
	void setDatumPoint(double x = 0.0, double y = 0.0, double z = 0.0);

	//����Ŀ���ľ�γ�ȼ�����
	void setLongitudeAndLatitude(double longitude , double latitude, double altitude);


	//����ԭ��gps���꣬���һ���������Ƿ�Ϊgps
	void setOrigin(double x = 0.0, double y = 0.0, double z = 0.0, bool isGPS = false);			

	//����xyz����תgps
	LongitudeAndLatitude DatumPointToLongitudeAndLatitude(Coordinate temp);

	//���ݾ�γ����xyz
	Coordinate longitudeAndLatitudeToDatumPoint(LongitudeAndLatitude temp);

	//�����ת��γ�Ȳ�
	LongitudeAndLatitude distance_in_meter_to_LongitudeAndLatitude_dev(Coordinate temp, double m_GPSY1);

	//��γ�Ȳ�ת�����
	Coordinate LongitudeAndLatitude_dev_to_distance_in_meter(LongitudeAndLatitude temp, double latitude);

private:
	double m_OriginX1;				//ԭ��xyz
	double m_OriginY1;
	double m_OriginZ1;

	double m_GPSX1;					//ԭ�㾭��
	double m_GPSY1;					//γ��
	double m_GPSZ1;					//����

	double m_OriginX2;				//��ǰ��xyz
	double m_OriginY2;
	double m_OriginZ2;

	double m_GPSX2;					//��ǰ�㾭��
	double m_GPSY2;					//γ��
	double m_GPSZ2;					//����
};

