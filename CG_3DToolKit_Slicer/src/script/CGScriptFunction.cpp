//CGScriptFunction.cpp
//

#include "CGScriptFunction.h"
#include "CGMetaType.h"

using namespace std;
using namespace cv;


QScriptValue ScriptAdd(QScriptContext *ctx, QScriptEngine *eng)
{
    double a = ctx->argument(0).toNumber();
    double b = ctx->argument(1).toNumber();
    return a + b;
}

QScriptValue ScriptSub(QScriptContext *ctx, QScriptEngine *eng)
{
    double a = ctx->argument(0).toNumber();
    double b = ctx->argument(1).toNumber();
    return a - b;
}

QScriptValue ScriptMul(QScriptContext *ctx, QScriptEngine *eng)
{
    double a = ctx->argument(0).toNumber();
    double b = ctx->argument(1).toNumber();
    return a * b;
}

QScriptValue ScriptDiv(QScriptContext *ctx, QScriptEngine *eng)
{
    double a = ctx->argument(0).toNumber();
    double b = ctx->argument(1).toNumber();
    return a / b;
}

QScriptValue ScriptGaussianFilter(QScriptContext *ctx, QScriptEngine *eng)
{
    cv::Mat imgSrc = ctx->argument(0).toVariant().value<cv::Mat>();
    cv::Mat imgDst;
    int sizeX = ctx->argument(1).toInt32();
    int sizeY = ctx->argument(2).toInt32();
    cv::GaussianBlur(imgSrc, imgDst, cv::Size(sizeX, sizeY), 0, 0);

    QScriptValue ret = eng->newVariant(QVariant::fromValue(imgDst));
    return ret;
}
