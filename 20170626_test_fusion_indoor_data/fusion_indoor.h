#ifndef FUSION_INDOOR_H
#define FUSION_INDOOR_H

#include <string>
#include <vector>

typedef struct
{
	double x, y, z;
	double qx, qy, qz, qw;
	double R[9];
	unsigned __int64 time;
}SlamPos;

void UpdateMinMax(double min[3], double max[3], const double v[3]);
int Quat2RotMat(double x, double y, double z, double w, double R[9]);
int getRotation(double inRoll, double inPitch, double inYaw, double outR[]);
int fusion(std::string posFile, std::string inPcdPath, std::string outputPath);

#endif