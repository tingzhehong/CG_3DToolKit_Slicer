﻿#ifndef CGPROFILEVIEW_H
#define CGPROFILEVIEW_H

#include <CGBaseWidget.h>
#include <QtCharts>

class CGProfileForm2D;
class CGProfileForm3D;
class CGChartView;
class vtkActor;

class CGProfileView : public CGBaseWidget
{
    Q_OBJECT

public:
    explicit CGProfileView(QWidget *parent = nullptr);
    ~CGProfileView();

signals:
    void SignalRequest();

public:
    void InitUi() override;
    void InitConnections() override;

    void Request();
    void Apply();

public:
    QPixmap *pPixmap = nullptr;
    vtkActor *pActor = nullptr;

private:
    CGProfileForm2D *m_Form2D;
    CGProfileForm3D *m_Form3D;

    CGChartView *chartView;
    QChart      *chart;
    QLineSeries *ProfileSeries;

};

#endif // CGPROFILEVIEW_H