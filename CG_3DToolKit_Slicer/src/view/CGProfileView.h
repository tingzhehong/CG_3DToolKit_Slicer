#ifndef CGPROFILEVIEW_H
#define CGPROFILEVIEW_H

#include <CGBaseWidget.h>
#include <CGOCVHeader.h>
#include <QtCharts>
#include <CGImage2DGraphicsItemAdapter.h>
#include <CGImage3DSectionItemVertical.h>
#include <CGImage3DSectionItemHorizontal.h>
#include <CGImage3DSectionLineItem.h>

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
    void SingalSectionToolRemoved();

public slots:
    void OnPlotProfile();
    void OnUseTool();
    void OnDelTool();
    void OnSetVerticalLine(double pos);
    void OnSetHorizontalLine(double pos);
    void OnSetTwoPointLine(double pos_1[], double pos_2[]);

public:
    void InitUi() override;
    void InitConnections() override;
    void UseSectionVerticalTool();
    void UseSectionHorizontalTool();
    void UseSectionLineTool();

    void Request();
    void Apply();

public:
    QPixmap *pPixmap = nullptr;
    vtkActor *pActor = nullptr;

private:
    void TwoPointLineProfileHandle();
    void RectProfileHandle();
    void CircleProfileHandle();
    void HorizontalLineProfileHandle();
    void VerticalLineProfileHandle();

    void PlotLineProfileHandle();
    void PlotRectProfileHandle();
    void PlotArcProfileHandle();

    float DepthImageBackGround(cv::Mat &ImageDepth);

public:
    CGProfileForm2D *m_Form2D;
    CGProfileForm3D *m_Form3D;
    CGImage3DSectionItemVertical *m_SectionItemVertical;
    CGImage3DSectionItemHorizontal *m_SectionItemHorizontal;
    CGImage3DSectionLineItem *m_SectionLineItem;
    QTimer *m_pPlotTimer;

private:
    CGChartView *chartView;
    QChart      *chart;
    QLineSeries *ProfileSeries;
    std::vector<float> ProfileVec;

    bool bSectionItemVertical = false;
    bool bSectionItemHorizontal = false;
    bool bSectionLineItem = false;

    float BackGroundValue = FLT_MIN;
};

#endif // CGPROFILEVIEW_H
