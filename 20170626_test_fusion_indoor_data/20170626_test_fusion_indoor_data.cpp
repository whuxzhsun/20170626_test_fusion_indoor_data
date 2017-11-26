// 20170626_test_fusion_indoor_data.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "fusion_indoor.h"


int _tmain(int argc, _TCHAR* argv[])
{
	fusion("D:\\temp_data\\20170608_songkai\\rePOS2.txt",
		"D:\\temp_data\\20170608_songkai\\pcd_data",
		"D:\\temp_data\\20170608_songkai\\las");

	return 0;
}

