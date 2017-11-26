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
	int  m_nMount;		//0代表车载、1代表机载、2代表其他状态
	double m_center_lon;//中央经线
	double m_frequency;	//默认为0.005，代表200HZ，也可以是0.01，即100HZ
	int m_gps_utc_flag;	//GPS与UTC的时间差值 17秒
	int m_add_week_second_to_pos;
	int m_week;			//0代表星期天，1代表星期1,2代表星期2
	int m_time_class;	//0,代表GPS天秒，1代表GPS整周秒
	int m_nIndex;		//代表当前解算点对应POS的序号, 用来为las提供偏移量
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
	int getFrequency();	// 从pos自动获取频率
	Pot getOffsetForLas();
};

#endif