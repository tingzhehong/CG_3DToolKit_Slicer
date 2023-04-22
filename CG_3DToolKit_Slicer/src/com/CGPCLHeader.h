#ifndef CGPCLHEADER_H
#define CGPCLHEADER_H

#pragma once

// C++
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>

// PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/angles.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct CG_Color
{
    float R;
    float G;
    float B;
};

static CG_Color colorLUT[51] = {
    //红
    1.0f, 0.0f, 0.0f,
    1.0f, 0.1f, 0.0f,
    1.0f, 0.2f, 0.0f,
    1.0f, 0.3f, 0.0f,
    1.0f, 0.4f, 0.0f,
    1.0f, 0.5f, 0.0f,
    1.0f, 0.6f, 0.0f,
    1.0f, 0.7f, 0.0f,
    1.0f, 0.8f, 0.0f,
    1.0f, 0.9f, 0.0f,
    //黄
    1.0f, 1.0f, 0.0f,
    0.9f, 1.0f, 0.0f,
    0.8f, 1.0f, 0.0f,
    0.7f, 1.0f, 0.0f,
    0.6f, 1.0f, 0.0f,
    0.5f, 1.0f, 0.0f,
    0.4f, 1.0f, 0.0f,
    0.3f, 1.0f, 0.0f,
    0.2f, 1.0f, 0.0f,
    0.1f, 1.0f, 0.0f,
    //绿
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.1f,
    0.0f, 1.0f, 0.2f,
    0.0f, 1.0f, 0.3f,
    0.0f, 1.0f, 0.4f,
    0.0f, 1.0f, 0.5f,
    0.0f, 1.0f, 0.6f,
    0.0f, 1.0f, 0.7f,
    0.0f, 1.0f, 0.8f,
    0.0f, 1.0f, 0.9f,
    //青
    0.0f, 1.0f, 1.0f,
    0.0f, 0.9f, 1.0f,
    0.0f, 0.8f, 1.0f,
    0.0f, 0.7f, 1.0f,
    0.0f, 0.6f, 1.0f,
    0.0f, 0.5f, 1.0f,
    0.0f, 0.4f, 1.0f,
    0.0f, 0.3f, 1.0f,
    0.0f, 0.2f, 1.0f,
    0.0f, 0.1f, 1.0f,
    //蓝
    0.0f, 0.0f, 1.0f,
    0.1f, 0.0f, 1.0f,
    0.2f, 0.0f, 1.0f,
    0.3f, 0.0f, 1.0f,
    0.4f, 0.0f, 1.0f,
    0.5f, 0.0f, 1.0f,
    0.6f, 0.0f, 1.0f,
    0.7f, 0.0f, 1.0f,
    0.8f, 0.0f, 1.0f,
    0.9f, 0.0f, 1.0f,
    //黑
    0.0f, 0.0f, 0.0f,
};

extern PointCloudT::Ptr g_PointCloud;

#endif // CGPCLHEADER_H
