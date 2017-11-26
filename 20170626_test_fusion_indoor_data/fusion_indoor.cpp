#include "stdafx.h"

#include "fusion_indoor.h"
#include "findFiles.h"

#include <iostream>
#include <fstream>

#include "liblas/liblas.hpp"
#include "CalDll_zrf.h"

using namespace std;

void UpdateMinMax(double min[3], double max[3], const double v[3])
{
	if (v[0]<min[0]) min[0] = v[0]; else if (v[0]>max[0]) max[0] = v[0];
	if (v[1]<min[1]) min[1] = v[1]; else if (v[1]>max[1]) max[1] = v[1];
	if (v[2]<min[2]) min[2] = v[2]; else if (v[2]>max[2]) max[2] = v[2];
}

int Quat2RotMat(double x, double y, double z, double w, double R[9])
{
	R[0] = 1 - 2 * y*y - 2 * z*z;
	R[1] = 2 * x*y + 2 * w*z;
	R[2] = 2 * x*z - 2 * w*y;

	R[3] = 2 * x*y - 2 * w*z;
	R[4] = 1 - 2 * x*x - 2 * z*z;
	R[5] = 2 * z*y + 2 * w*x;

	R[6] = 2 * x*z + 2 * w*y;
	R[7] = 2 * y*z - 2 * w*x;
	R[8] = 1 - 2 * x*x - 2 * y*y;

	//for (int i = 0; i < 3; i++)
	//{
	//	printf("\n");
	//	for (int j = 0; j < 3; j++)
	//	{
	//		printf("%6.3lf\t", R[i * 3 + j]);
	//	}
	//}
	

	return 1;
}

int getRotation(double inRoll, double inPitch, double inYaw, double outR[])
{
	//outR[0] = cos(inRoll)*cos(inYaw) + sin(inPitch)*sin(inRoll)*sin(inYaw);
	//outR[1] = cos(inPitch)*sin(inYaw);
	//outR[2] = -sin(inRoll)*cos(inYaw) + sin(inPitch)*cos(inRoll)*sin(inYaw);
	//outR[3] = -cos(inRoll)*sin(inYaw) + sin(inPitch)*sin(inRoll)*cos(inYaw);
	//outR[4] = cos(inPitch)*cos(inYaw);
	//outR[5] = sin(inRoll)*sin(inYaw) + sin(inPitch)*cos(inRoll)*cos(inYaw);
	//outR[6] = cos(inPitch)*sin(inRoll);
	//outR[7] = -sin(inPitch); // + or - ?
	//outR[8] = cos(inRoll)*cos(inPitch);

	//case 'zxy'
	//%[cy*cz - sy*sx*sz, cy*sz + sy*sx*cz, -sy*cx]
	//%[-sz*cx, cz*cx, sx]
	//%[sy*cz + cy*sx*sz, sy*sz - cy*sx*cz, cy*cx]

	double sx = sin(inRoll), cx = cos(inRoll);
	double sy = sin(inPitch), cy = cos(inPitch);
	double sz = sin(inYaw), cz = cos(inYaw);

	outR[0] = cy*cz - sy*sx*sz;
	outR[3] = cy*sz + sy*sx*cz;
	outR[6] = -sy*cx;
	outR[1] = -sz*cx;
	outR[4] = cz*cx;
	outR[7] = sx;
	outR[2] = sy*cz + cy*sx*sz;
	outR[5] = sy*sz - cy*sx*cz;
	outR[8] = cy*cx;


	return 0;
}

int fusion(std::string posFile, std::string inPcdPath, std::string outputPath)
{
	/*
		读取pos文件
	*/
#pragma region readpos
	vector<SlamPos> pos;
	FILE *inFp;
	fopen_s(&inFp, posFile.c_str(), "r");
	if (!inFp)
	{
		cout << "Failed to open pos File!" << endl;
		return 1;
	}

	char line[512];
	fgets(line, 512, inFp);
	while (1)
	{
		char lineStr[512];
		if (!fgets(lineStr, 512, inFp))
		{
			break;
		}

		SlamPos tpPos;
		sscanf_s(lineStr, "%lf%lf%lf%lf%lf%lf%lf%I64u", &tpPos.x, &tpPos.y, &tpPos.z, &tpPos.qx, &tpPos.qy, &tpPos.qz, &tpPos.qw, &tpPos.time);

		pos.push_back(tpPos);
	}
	fclose(inFp);
#pragma endregion


	/*
		获取安装角矩阵
	*/
	double iR[9];
//	getRotation(-0.0349, -1.4137,  0.7278, iR); // 1.4137 -0.0349  -0.7278	√
	getRotation(1.4137,  -0.0349, -0.7278, iR); // 1.4137 -0.0349  -0.7278	√

	getRotation(1.4137, -0.0349, -0.7278, iR);

	/*
		读取pcd点云进行融合
	*/
#pragma region readpcdandfusion
	vector<string> pcdFiles;
	findFiles(inPcdPath, pcdFiles, "pcd", 1);
	
	size_t i = 20000;
	unsigned int j = 0;
	double x, y, z;
	while (i < pcdFiles.size())
	{
		string outFileName = outputPath + pcdFiles[i].substr(pcdFiles[i].find_last_of('\\'),
			pcdFiles[i].find_last_of('.') - pcdFiles[i].find_last_of('\\')) + string(".las");

		unsigned int point_count = 0;
		std::ofstream ofs(outFileName.c_str(), std::ios::out | std::ios::binary);
		if (!ofs.is_open())
		{
			cout << "Failed to create output Las File!" << endl;
			return 1;
		}
		liblas::Header header;

		// 设置文件头，点数、格式、缩放因子、偏移量

		header.SetVersionMajor(1);
		header.SetVersionMinor(2);
		header.SetDataFormatId(liblas::PointFormatName::ePointFormat3);
		header.SetScale(0.001, 0.001, 0.001);
		header.SetOffset(int(0), int(0), 0);

		liblas::Writer writer(ofs, header);

		liblas::Point point(&header);

		double minPt[3] = { 9999999, 9999999, 9999999 };
		double maxPt[3] = { 0, 0, 0 };
		double pt[3] = { 0 };

		for (; i < pcdFiles.size(); i++)
		{
			string tStr = pcdFiles[i].substr(pcdFiles[i].find_last_of('\\') + 1,
				pcdFiles[i].find_last_of('.') - pcdFiles[i].find_last_of('\\') - 1);

			unsigned __int64 t;
			sscanf_s(tStr.c_str(), "%I64u", &t);

			if (t < pos[0].time)
			{
				continue;
			}

			/*
			   查找文件对应的pos
			*/
#pragma region searchpos
			SlamPos				curPos;
			unsigned __int64	curT = 0;

			for (; j < pos.size() - 1; j++)
			{
				if (t >= pos[j].time && t <= pos[j + 1].time)
				{
					// interpolation for laser point in pos
					curT = t;
					double weightTime = 1.0 * (t - pos[j].time) / (pos[j + 1].time - pos[j].time);

					curPos.x = pos[j].x + (pos[j + 1].x - pos[j].x) * weightTime;
					curPos.y = pos[j].y + (pos[j + 1].y - pos[j].y) * weightTime;
					curPos.z = pos[j].z + (pos[j + 1].z - pos[j].z) * weightTime;

					curPos.qx = pos[j].qx + (pos[j + 1].qx - pos[j].qx) * weightTime;
					curPos.qy = pos[j].qy + (pos[j + 1].qy - pos[j].qy) * weightTime;
					curPos.qz = pos[j].qz + (pos[j + 1].qz - pos[j].qz) * weightTime;
					curPos.qw = pos[j].qw + (pos[j + 1].qw - pos[j].qw) * weightTime;

					Quat2RotMat(curPos.qx, curPos.qy, curPos.qz, curPos.qw, curPos.R);

					break;

				}
			}

			if (curT == 0)
			{
				return 1;
			}
#pragma endregion

			FILE *inPcd;
			fopen_s(&inPcd, pcdFiles[i].c_str(), "r");
			if (!inPcd)
			{
				cout << "Failed to open:" << pcdFiles[i].c_str() << endl;
				return 0;
			}

			Record	rd;
			Pot		pt;

			/*
			   跳过文件头
			*/
			for (int k = 0; k < 11; k++)
			{
				fgets(line, 512, inPcd);
			}


			while (true)
			{
				char rcdLine[512];
				if (!fgets(rcdLine, 512, inPcd))
				{
					break;
				}

				sscanf_s(rcdLine, "%lf%lf%lf%d", &x, &y, &z, &rd.i);

				rd.x = x;
				rd.y = y;
				rd.z = z;

				pt.x = iR[0] * rd.x + iR[1] * rd.y + iR[2] * rd.z;
				pt.y = iR[3] * rd.x + iR[4] * rd.y + iR[5] * rd.z;
				pt.z = iR[6] * rd.x + iR[7] * rd.y + iR[8] * rd.z;

				rd.x = pt.x - 0.058;
				rd.y = pt.y - 0.188;
				rd.z = pt.z - 0.082;

				pt.x = curPos.R[0] * rd.x + curPos.R[3] * rd.y + curPos.R[6] * rd.z + curPos.x;
				pt.y = curPos.R[1] * rd.x + curPos.R[4] * rd.y + curPos.R[7] * rd.z + curPos.y;
				pt.z = curPos.R[2] * rd.x + curPos.R[5] * rd.y + curPos.R[8] * rd.z + curPos.z;
				

				point.SetX(pt.x);
				point.SetY(pt.y);
				point.SetZ(pt.z);
				point.SetIntensity(rd.i);
				point.SetColor(liblas::Color(HCOLOR[rd.i % 255][0] * 256, HCOLOR[rd.i % 255][1] * 256, HCOLOR[rd.i % 255][2] * 256));
				point.SetTime(curT * 1.0 / 1000000);

				writer.WritePoint(point);
				point_count++;
			}
			fclose(inPcd);

			if (point_count > 10000000)
			{
				break;
			}
		}
		header.SetPointRecordsCount(point_count);
		header.SetMax(maxPt[0], maxPt[1], maxPt[2]);
		header.SetMin(minPt[0], minPt[1], minPt[2]);

		writer.SetHeader(header);
		writer.WriteHeader();

		i++;
	}
#pragma endregion

	return 0;
}