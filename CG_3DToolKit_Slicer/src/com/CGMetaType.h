#ifndef CGMETATYPE_H
#define CGMETATYPE_H

#include <CGOCVHeader.h>
#include <CGPCLHeader.h>
#include <CGVTKHeader.h>

Q_DECLARE_METATYPE(cv::Mat)
Q_DECLARE_METATYPE(CG_IMG)
Q_DECLARE_METATYPE(PointT)
Q_DECLARE_METATYPE(PointCloudT::Ptr)

#endif // CGMETATYPE_H
