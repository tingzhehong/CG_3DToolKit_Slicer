#include "CGProfileView.h"
#include <CGProfileForm2D.h>
#include <CGProfileForm3D.h>
#include <CGChartView.h>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>
#include <QDebug>
#include <Utils.h>
#include <CGVTKUtils.h>
#include <CGVTKHeader.h>

using namespace std;

CGProfileView::CGProfileView(QWidget *parent)
    : CGBaseWidget(parent)
    , m_Form2D(new CGProfileForm2D)
    , m_Form3D(new CGProfileForm3D)
{
    InitUi();
    InitConnections();
    setWindowTitle(tr(u8"轮廓分析"));
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
}

CGProfileView::~CGProfileView()
{

}

void CGProfileView::InitUi()
{
    chartView = new CGChartView(this);
    chartView->setRenderHint(QPainter::Antialiasing);
    chart = new QChart();

    chart->setTitle(tr(u8"轮廓 X-Z"));
    chart->setTitleBrush(QBrush(Qt::white));
    chart->setBackgroundBrush(QBrush(QColor(25, 50, 75)));

    ProfileSeries = new QLineSeries();
    ProfileSeries->setName(tr(u8"Z"));
    ProfileSeries->setColor(Qt::green);
    chart->addSeries(ProfileSeries);

    QValueAxis *axisX = new QValueAxis();
    axisX->setTitleText(tr(u8"X(mm)"));
    axisX->setRange(0, 16);
    axisX->setTitleBrush(QBrush(Qt::white));
    axisX->setLabelsColor(Qt::white);
    axisX->setLinePenColor(Qt::white);
    axisX->setGridLineColor(Qt::darkGray);

    QValueAxis *axisY = new QValueAxis();
    axisY->setTitleText(tr(u8"Z(mm)"));
    axisY->setRange(-16, 16);
    axisY->setTitleBrush(QBrush(Qt::white));
    axisY->setLabelsColor(Qt::white);
    axisY->setLinePenColor(Qt::white);
    axisY->setGridLineColor(Qt::darkGray);

    chart->setAxisX(axisX, ProfileSeries);
    chart->setAxisY(axisY, ProfileSeries);

    chartView->setChart(chart);

    QHBoxLayout *pImgLayout = new QHBoxLayout();
    pImgLayout->addWidget(m_Form2D);
    pImgLayout->addWidget(m_Form3D);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addLayout(pImgLayout);
    pMainLayout->addWidget(chartView);

    setLayout(pMainLayout);
}

void CGProfileView::InitConnections()
{

}

void CGProfileView::Request()
{
    emit SignalRequest();
}

void CGProfileView::Apply()
{
    if (m_Form2D->bGraphicsScene)
        m_Form2D->m_pGraphicsView->RemoveALLItems();

    if (!pPixmap->size().isEmpty())
    {
        m_Form2D->m_pPixmap = pPixmap;
        m_Form2D->m_pItem->setPixmap(*m_Form2D->m_pPixmap);
        m_Form2D->m_pScene->addItem(m_Form2D->m_pItem);
        m_Form2D->m_pGraphicsView->setScene(m_Form2D->m_pScene);
        m_Form2D->m_pGraphicsView->ImageWidth = m_Form2D->m_pPixmap->width();
        m_Form2D->m_pGraphicsView->ImageHeight = m_Form2D->m_pPixmap->height();

        if (!m_Form2D->bGraphicsScene)
        {
            m_Form2D->m_pGraphicsView->ResetGraphicsView();
            m_Form2D->bGraphicsScene = true;
        }
    }

    m_Form3D->m_CGVTKWidget->defaultRenderer()->RemoveActor(m_Form3D->m_Actor);
    m_Form3D->m_Actor = pActor;
    m_Form3D->m_CGVTKWidget->defaultRenderer()->AddActor(m_Form3D->m_Actor);
    m_Form3D->m_CGVTKWidget->defaultRenderer()->ResetCamera();
    m_Form3D->m_CGVTKWidget->update();
}

void CGProfileView::TwoPointLineProfileHandle()
{

}

void CGProfileView::RectProfileHandle()
{

}

void CGProfileView::CircleProfileHandle()
{

}

void CGProfileView::HorizontalLineProfileHandle()
{
    ProfileVec.clear();
    ProfileSeries->clear();

    for (int i = 0; i < g_Image.DepthImage.cols; i += 4)
    {
        float x = m_Form2D->m_Line.y1();
        float y = g_Image.DepthImage.at<float>(x, i);

        ProfileVec.push_back(y);
        ProfileSeries->append(i * g_XPitch, y);
    }

    if (ProfileVec.empty()) return;
    float maxValue = *max_element(ProfileVec.begin(), ProfileVec.end());
    float minValue = *min_element(ProfileVec.begin(), ProfileVec.end());
    chart->axisY()->setRange(minValue, maxValue);
    chart->axisX()->setRange(m_Form2D->m_Line.x1() * g_XPitch, m_Form2D->m_Line.x2() * g_XPitch);

}

void CGProfileView::VerticalLineProfileHandle()
{
    ProfileVec.clear();
    ProfileSeries->clear();

    for (int i = 0; i < g_Image.DepthImage.rows; i += 4)
    {
        float x = m_Form2D->m_Line.x1();
        float y = g_Image.DepthImage.at<float>(i, x);

        ProfileVec.push_back(y);
        ProfileSeries->append(i * g_YPitch, y);
    }

    if (ProfileVec.empty()) return;
    float maxValue = *max_element(ProfileVec.begin(), ProfileVec.end());
    float minValue = *min_element(ProfileVec.begin(), ProfileVec.end());
    chart->axisY()->setRange(minValue, maxValue);
    chart->axisX()->setRange(m_Form2D->m_Line.y1() * g_YPitch, m_Form2D->m_Line.y2() * g_YPitch);
}
