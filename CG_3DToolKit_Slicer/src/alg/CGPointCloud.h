#ifndef CGPOINTCLOUD_H
#define CGPOINTCLOUD_H

#pragma once

#include <CGOCVHeader.h>
#include <CGPCLHeader.h>
#include <CGVTKHeader.h>

using namespace std;
using namespace cv;

namespace CG
{

void PointCloud2Color(PointCloudT::Ptr cloud, PointCloudT::Ptr color_cloud);
void PointCloud2Gray(PointCloudT::Ptr cloud, PointCloudT::Ptr gray_cloud);

void PointCloud4Elevation(PointCloudT::Ptr cloud, cv::Mat &ElevationImage);
void PointCloud4Gray(PointCloudT::Ptr cloud, cv::Mat &GrayImage);
void PointCloud4Intensity(PointCloudT::Ptr cloud, cv::Mat &IntensityImage);

void PointCloudMinMaxZ(PointCloudT::Ptr cloud, float &CloudMaxZ, float &CloudMinZ);
void FromDepthImage2PointCloud(cv::Mat &ImageDepth, float XPitch, float YPitch, float DownLimitThres, float UpLimitThres, PointCloudT::Ptr cloud);

void DownSampleFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr sub_cloud, float leaf);
void SORFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const float thres);
void PassThroughFilter(PointCloudT::Ptr cloud, PointCloudT::Ptr filter_cloud, const String Direction, const float LimitsDown, const float LimitsUp);

void LoadPCDFile(const string filename, vtkActor *actor);
void LoadCSVFile(const string filename, vtkActor *actor);
void LoadTXTFile(const string filename, vtkActor *actor);

void VTKPointCloudElevation(PointCloudT::Ptr cloud, vtkActor *actor);
void VTKPointCloudGray(PointCloudT::Ptr cloud, vtkActor *actor);
void VTKPointCloudIntensity(PointCloudT::Ptr cloud, vtkActor *actor);

bool IsOrderPointCloud();

}

#endif // CGPOINTCLOUD_H
