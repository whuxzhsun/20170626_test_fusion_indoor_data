#include "stdafx.h"
#include "findFiles.h"
#include <io.h>

/*
1)Parameter1:root dir you need to search.
2)Parameter2:search result
3)Parameter3:search filter:
	"":subdir;  
	"*":subdir and subfile(when mod==1 means except search deeply);
	"txt":subfile that is txtfile
4)Parameter4:seach mod
	1:no deep searching
	other: deep searching
*/
void findFiles(std::string path, std::vector<std::string>& files,std::string filter,int mod)
{
	//文件句柄
	long   hFile   =   0;
	//文件信息
	struct _finddata_t fileinfo;
	std::string p;
	std::string t_filter="\\*."+filter;
	if((hFile = _findfirst(p.assign(path).append(t_filter.c_str()).c_str(),&fileinfo)) !=  -1)
	{
		if (mod==1)
		{
			do
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name) );
				
			}while(_findnext(hFile, &fileinfo)  == 0);
		}
		else
		{
			do
			{
				//如果是目录,迭代之
				//如果不是,加入列表
				if((fileinfo.attrib &  _A_SUBDIR))
				{
					if(strcmp(fileinfo.name,".") != 0  &&  strcmp(fileinfo.name,"..") != 0)
						findFiles( p.assign(path).append("\\").append(fileinfo.name), files,filter,mod );
				}
				else
				{
					files.push_back(p.assign(path).append("\\").append(fileinfo.name) );
				}
			}while(_findnext(hFile, &fileinfo)  == 0);
		}
		_findclose(hFile);
	}
}