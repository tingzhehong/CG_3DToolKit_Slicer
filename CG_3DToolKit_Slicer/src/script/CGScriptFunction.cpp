//CGScriptFunction.cpp
//

#include "CGScriptFunction.h"
#include "CGMetaType.h"
#include <QDebug>

using namespace std;
using namespace cv;


QScriptValue ScriptAdd(QScriptContext *ctx, QScriptEngine *eng)
{
    Q_UNUSED(eng);
    double a = ctx->argument(0).toNumber();
    double b = ctx->argument(1).toNumber();
    return a + b;
}

QScriptValue ScriptSub(QScriptContext *ctx, QScriptEngine *eng)
{
    Q_UNUSED(eng);
    double a = ctx->argument(0).toNumber();
    double b = ctx->argument(1).toNumber();
    return a - b;
}

QScriptValue ScriptMul(QScriptContext *ctx, QScriptEngine *eng)
{
    Q_UNUSED(eng);
    double a = ctx->argument(0).toNumber();
    double b = ctx->argument(1).toNumber();
    return a * b;
}

QScriptValue ScriptDiv(QScriptContext *ctx, QScriptEngine *eng)
{
    Q_UNUSED(eng);
    double a = ctx->argument(0).toNumber();
    double b = ctx->argument(1).toNumber();
    return a / b;
}

QScriptValue ScriptGaussianFilter(QScriptContext *ctx, QScriptEngine *eng)
{
    cv::Mat imgSrc;
    cv::Mat imgDst;
    int sizeX = ctx->argument(1).toInt32();
    int sizeY = ctx->argument(2).toInt32();

    if (ctx->argument(0).toVariant().canConvert<CG_IMG>())
        imgSrc = ctx->argument(0).toVariant().value<CG_IMG>().GrayImage.clone();
    if (ctx->argument(0).toVariant().canConvert<cv::Mat>())
        imgSrc = ctx->argument(0).toVariant().value<cv::Mat>().clone();

    cv::GaussianBlur(imgSrc, imgDst, cv::Size(sizeX, sizeY), 0, 0);

    QScriptValue ret = eng->newVariant(QVariant::fromValue(imgDst));
    return ret;
}

QScriptValue ScriptMedianFilter(QScriptContext *ctx, QScriptEngine *eng)
{
    cv::Mat imgSrc;
    cv::Mat imgDst;
    int ksize = ctx->argument(1).toInt32();

    if (ctx->argument(0).toVariant().canConvert<CG_IMG>())
        imgSrc = ctx->argument(0).toVariant().value<CG_IMG>().GrayImage.clone();
    if (ctx->argument(0).toVariant().canConvert<cv::Mat>())
        imgSrc = ctx->argument(0).toVariant().value<cv::Mat>().clone();

    cv::medianBlur(imgSrc, imgDst, ksize);

    QScriptValue ret = eng->newVariant(QVariant::fromValue(imgDst));
    return ret;
}

QScriptValue ScriptVoxelFilter(QScriptContext *ctx, QScriptEngine *eng)
{
    PointCloudT::Ptr src_cloud(new PointCloudT);
    PointCloudT::Ptr dst_cloud(new PointCloudT);

    pcl::PCLPointCloud2::Ptr original_cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr filtered_cloud(new pcl::PCLPointCloud2);

    if (ctx->argument(0).toVariant().canConvert<PointCloudT::Ptr>())
        src_cloud = ctx->argument(0).toVariant().value<PointCloudT::Ptr>();

    float leaf = ctx->argument(1).toVariant().toFloat();

    pcl::toPCLPointCloud2(*src_cloud, *original_cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> VoxeGridlFilter;
    VoxeGridlFilter.setInputCloud(original_cloud);
    VoxeGridlFilter.setLeafSize(leaf, leaf, leaf);
    VoxeGridlFilter.filter(*filtered_cloud);
    pcl::fromPCLPointCloud2(*filtered_cloud, *dst_cloud);

    QScriptValue ret = eng->newVariant(QVariant::fromValue(dst_cloud));;
    return ret;
}
