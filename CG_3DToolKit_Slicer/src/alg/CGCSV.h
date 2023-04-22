//CGPointCloud.h
//

#ifndef CGCSV_H
#define CGCSV_H

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <CGPCLHeader.h>

using namespace std;

namespace CG
{

// 获取CSV尺寸
void GetCSVSize(const string &fileCSV, float &X, float &Y, float &XPitch, float &YPitch, int &rowNum, int &colNum);
// 读取CSV文件
void CSVFile2PointCloud(const string &fileCSV, float &X, float &Y, float &XPitch, float &YPitch, int &rowNum, int &colNum, PointCloudT::Ptr cloud);
// 写入CSV文件
void PointCloud2CSVFile(const string &fileCSV, float &X, float &Y, float &XPitch, float &YPitch, int &rowNum, int &colNum, PointCloudT::Ptr cloud);

}

#endif // CGCSV_H
