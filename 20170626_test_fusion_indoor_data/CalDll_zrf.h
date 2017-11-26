#ifndef CALLDLL_ZRF_H
#define CALLDLL_ZRF_H

//#ifndef MYPRJDLL
//#define MYPRJDLLIMPORTEXPORT _declspec(dllimport)
//#else
//#define MYPRJDLLIMPORTEXPORT _declspec(dllexport)
//#endif

#include "common_zrf.h"
#include <vector>
#include <string>

class /*MYPRJDLLIMPORTEXPORT*/ MyPrj
{
public:
	MyPrj(void);
	MyPrj(std::string posPath,std::string rcdPath,std::string savePath);
	~MyPrj(void);
private:
	std::vector<Pos> vpos;
	std::string posPath;
	std::string rcdPath;
	std::string savePath;

private:
	double iR[9];
	int    posID;

public:
	Cali CaliPar;
	int  m_nMount;		//0�����ء�1������ء�2��������״̬
	double m_center_lon;//���뾭��
	double m_frequency;	//Ĭ��Ϊ0.005������200HZ��Ҳ������0.01����100HZ
	int m_gps_utc_flag;	//GPS��UTC��ʱ���ֵ 17��
	int m_add_week_second_to_pos;
	int m_week;			//0���������죬1��������1,2��������2
	int m_time_class;	//0,����GPS���룬1����GPS������
	int m_nIndex;		//����ǰ������ӦPOS�����, ����Ϊlas�ṩƫ����
	int m_nScanner;		//1--VUX-1, 2--VZ1000, 3--Faro,  4--Vel16
	double transformR[9];

public:
	void setPosPath(std::string Path);
	void setRcdPath(std::string Path);
	void setSavePath(std::string Path);
	void setDefaultParameter();
	void setParameter(double dx, double dy, double dz, double droll, double dpitch, double dyaw);
	void setPos(double centerMeridianLine);
	int addOffsetGPSWeekSecond(int offsetSecond);
	int getRotation(double inRoll, double inPitch, double inYaw, double outR[]);
	int getRotationFromGeocentric(Pos inPos, double R[]);

	int CalData(Record vred,Pot &vpot);
	int getFrequency();	// ��pos�Զ���ȡƵ��
	Pot getOffsetForLas();
};

#endif