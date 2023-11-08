﻿#ifndef CGSCRIPTFUNCTION_H
#define CGSCRIPTFUNCTION_H

#pragma once

#include <CGOCVHeader.h>
#include <CGPCLHeader.h>
#include <CGVTKHeader.h>
#include <QScriptEngine>
#include <QScriptValue>
#include <QScriptValueList>


QScriptValue ScriptAdd(QScriptContext *ctx, QScriptEngine *eng);
QScriptValue ScriptSub(QScriptContext *ctx, QScriptEngine *eng);
QScriptValue ScriptMul(QScriptContext *ctx, QScriptEngine *eng);
QScriptValue ScriptDiv(QScriptContext *ctx, QScriptEngine *eng);

//TODO:
QScriptValue ScriptGaussianFilter(QScriptContext *ctx, QScriptEngine *eng);

#endif // CGSCRIPTFUNCTION_H
