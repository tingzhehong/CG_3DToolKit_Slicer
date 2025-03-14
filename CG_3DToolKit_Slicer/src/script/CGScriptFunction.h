﻿#ifndef CGSCRIPTFUNCTION_H
#define CGSCRIPTFUNCTION_H

#pragma once

#include <CGOCVHeader.h>
#include <CGPCLHeader.h>
#include <CGVTKHeader.h>
#include <QScriptEngine>
#include <QScriptValue>
#include <QScriptValueList>

typedef std::function<QScriptValue(QScriptContext *ctx, QScriptEngine *eng)>    ScriptFunction; 

//Math script function
QScriptValue ScriptAdd(QScriptContext *ctx, QScriptEngine *eng);
QScriptValue ScriptSub(QScriptContext *ctx, QScriptEngine *eng);
QScriptValue ScriptMul(QScriptContext *ctx, QScriptEngine *eng);
QScriptValue ScriptDiv(QScriptContext *ctx, QScriptEngine *eng);

//Filter script function
QScriptValue ScriptGaussianFilter(QScriptContext *ctx, QScriptEngine *eng);
QScriptValue ScriptMeanFilter(QScriptContext *ctx, QScriptEngine *eng);
QScriptValue ScriptMedianFilter(QScriptContext *ctx, QScriptEngine *eng);
QScriptValue ScriptVoxelFilter(QScriptContext *ctx, QScriptEngine *eng);

//TODO:

#endif // CGSCRIPTFUNCTION_H
