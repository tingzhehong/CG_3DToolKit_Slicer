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
    , m_SectionItemVertical(new CGImage3DSectionItemVertical)
    , m_SectionItemHorizontal(new CGImage3DSectionItemHorizontal)
    , m_SectionLineItem(new CGImage3DSectionLineItem)
    , m_pPlotTimer(new QTimer())
{
    InitUi();
    InitConnections();
    setWindowTitle(tr(u8"轮廓分析"));
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
}

CGProfileView::~CGProfileView()
{
    delete m_pPlotTimer;
}

void CGProfileView::OnPlotProfile()
{
    if (CGImage2DGraphicsItemAdapter::getInstance()->m_Status == true)
    {
        if (g_Image.DepthImage.empty())   return;

        int type = CGImage2DGraphicsItemAdapter::getInstance()->m_SendType;

        if (type == 0)         // 0 = line
            PlotLineProfileHandle();

        if (type == 1)         // 1 = rect
            PlotRectProfileHandle();

        if (type == 2)         // 2 = arc
            PlotArcProfileHandle();
    }

    return;
}

void CGProfileView::OnUseTool()
{
    m_Form2D->OnUseTool();
    m_pPlotTimer->start();
}

void CGProfileView::OnDelTool()
{
    m_Form2D->OnDelTool();
    m_pPlotTimer->stop();

    if (bSectionItemVertical)
        m_SectionItemVertical->RemoveSectionItem();
    if (bSectionItemHorizontal)
        m_SectionItemHorizontal->RemoveSectionItem();
    if (bSectionLineItem)
        m_SectionLineItem->RemoveSectionItem();
}

void CGProfileView::OnSetHorizontalLine(double pos)
{
    if (pos < 0 || pos > 1)
        return;
    m_Form2D->SetHorizontalLine(1 - pos);
}

void CGProfileView::OnSetTwoPointLine(double pos_1[], double pos_2[])
{
    if (pos_1[0] < 0 || pos_1[0] > 1 || pos_2[0] < 0 || pos_2[0] > 1)
        return;
    if (pos_1[1] < 0 || pos_1[1] > 1 || pos_2[1] < 0 || pos_2[1] > 1)
        return;
    m_Form2D->SetTwoPointLine(pos_1, pos_2);
}

void CGProfileView::OnSetVerticalLine(double pos)
{
    if (pos < 0 || pos > 1)
        return;
    m_Form2D->SetVerticalLine(pos);
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

    m_pPlotTimer->setInterval(200);

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
    connect(m_pPlotTimer, &QTimer::timeout, this, &CGProfileView::OnPlotProfile);
    connect(m_SectionItemVertical, &CGImage3DSectionItemVertical::SignalPositionChange, this, &CGProfileView::OnSetVerticalLine);
    connect(m_SectionItemHorizontal, &CGImage3DSectionItemHorizontal::SignalPositionChange, this, &CGProfileView::OnSetHorizontalLine);
    connect(m_SectionLineItem, &CGImage3DSectionLineItem::SignalPositionChange, this, &CGProfileView::OnSetTwoPointLine);
}

void CGProfileView::UseSectionVerticalTool()
{
    if (!m_Form2D->bGraphicsScene)
        return;
    OnDelTool();

    m_SectionItemVertical->SetVTKWidget(m_Form3D->m_CGVTKWidget);
    m_SectionItemVertical->SetActor(m_Form3D->m_Actor);
    m_SectionItemVertical->InitSectionItem();
    m_SectionItemVertical->SetInteractorStyleMouseEvent();

    bSectionItemVertical = true;
}

void CGProfileView::UseSectionHorizontalTool()
{
    if (!m_Form2D->bGraphicsScene)
        return;
    OnDelTool();

    m_SectionItemHorizontal->SetVTKWidget(m_Form3D->m_CGVTKWidget);
    m_SectionItemHorizontal->SetActor(m_Form3D->m_Actor);
    m_SectionItemHorizontal->InitSectionItem();
    m_SectionItemHorizontal->SetInteractorStyleMouseEvent();

    bSectionItemHorizontal = true;
}

void CGProfileView::UseSectionLineTool()
{
    if (!m_Form2D->bGraphicsScene)
        return;
    OnDelTool();

    m_SectionLineItem->SetVTKWidget(m_Form3D->m_CGVTKWidget);
    m_SectionLineItem->SetActor(m_Form3D->m_Actor);
    m_SectionLineItem->InitSectionItem();
    m_SectionLineItem->SetInteractorStyleMoveActor();

    bSectionLineItem = true;
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
    m_Form2D->m_pGraphicsView->InstallFilter();

    m_Form3D->m_CGVTKWidget->defaultRenderer()->RemoveActor(m_Form3D->m_Actor);
    m_Form3D->m_Actor = pActor;
    m_Form3D->m_CGVTKWidget->defaultRenderer()->AddActor(m_Form3D->m_Actor);
    m_Form3D->m_CGVTKWidget->defaultRenderer()->ResetCamera();
    m_Form3D->m_CGVTKWidget->update();

    m_pPlotTimer->stop();
}

void CGProfileView::TwoPointLineProfileHandle()
{
    ProfileVec.clear();
    ProfileSeries->clear();

    // 轮廓线方向
    bool directLine;
    if (fabs(m_Form2D->m_Line.x1() - m_Form2D->m_Line.x2()) > abs(m_Form2D->m_Line.y1() - m_Form2D->m_Line.y2()))
        directLine = true;  // 横线
    else
        directLine = false; // 竖线

    // 垂直竖线
    if (m_Form2D->m_Line.x1() == m_Form2D->m_Line.x2())
    {
        if (m_Form2D->m_Line.y1() <= m_Form2D->m_Line.y2())
        {
            for (int i = (int)m_Form2D->m_Line.y1(); i <= (int)m_Form2D->m_Line.y2(); i += 4)
            {
                float x = m_Form2D->m_Line.x1();
                float y = g_Image.DepthImage.at<float>(i, x);

                ProfileVec.push_back(y);
                ProfileSeries->append(i * g_YPitch, y);
            }
            chart->axisX()->setRange(m_Form2D->m_Line.y1() * g_YPitch, m_Form2D->m_Line.y2() * g_YPitch);
        }
        else
        {
            for (int i = (int)m_Form2D->m_Line.y1(); i <= (int)m_Form2D->m_Line.y2(); i -= 4)
            {
                float x = m_Form2D->m_Line.x1();
                float y = g_Image.DepthImage.at<float>(i, x);

                ProfileVec.push_back(y);
                ProfileSeries->append(i * g_YPitch, y);
            }
            chart->axisX()->setRange(m_Form2D->m_Line.y2() * g_YPitch, m_Form2D->m_Line.y1() * g_YPitch);
        }
        if (ProfileVec.empty()) return;
        float maxValue = *max_element(ProfileVec.begin(), ProfileVec.end());
        float minValue = *min_element(ProfileVec.begin(), ProfileVec.end());
        chart->axisY()->setRange(minValue, maxValue);

        return;
    }

    // 直线方程
    float k = (m_Form2D->m_Line.y1() - m_Form2D->m_Line.y2()) / (m_Form2D->m_Line.x1() - m_Form2D->m_Line.x2());
    float b = ((m_Form2D->m_Line.x1() * m_Form2D->m_Line.y2()) - (m_Form2D->m_Line.x2() * m_Form2D->m_Line.y1())) / (m_Form2D->m_Line.x1() - m_Form2D->m_Line.x2());

    // 轮廓线横线
    if (directLine)
    {
        if (m_Form2D->m_Line.x1() <= m_Form2D->m_Line.x2())
        {
            for (int i = (int)m_Form2D->m_Line.x1(); i <= (int)m_Form2D->m_Line.x2(); i += 4)
            {
                float x = fabs(k * i + b);
                float y = g_Image.DepthImage.at<float>(x, i);

                ProfileVec.push_back(y);
                ProfileSeries->append(i * g_XPitch, y);
            }
            chart->axisX()->setRange(m_Form2D->m_Line.x1() * g_XPitch, m_Form2D->m_Line.x2() * g_XPitch);
        }
        else
        {
            for (int i = (int)m_Form2D->m_Line.x1(); i >= (int)m_Form2D->m_Line.x2(); i -= 4)
            {
                float x = fabs(k * i + b);
                float y = g_Image.DepthImage.at<float>(x, i);

                ProfileVec.push_back(y);
                ProfileSeries->append(i * g_XPitch, y);
            }
            chart->axisX()->setRange(m_Form2D->m_Line.x2() * g_XPitch, m_Form2D->m_Line.x1() * g_XPitch);
        }
    }
    // 轮廓线竖线
    else
    {
        if (m_Form2D->m_Line.y1() <= m_Form2D->m_Line.y2())
        {
            for (int i = (int)m_Form2D->m_Line.y1(); i <= (int)m_Form2D->m_Line.y2(); i += 4)
            {
                float x = fabs((i - b) / k);
                float y = g_Image.DepthImage.at<float>(i, x);

                ProfileVec.push_back(y);
                ProfileSeries->append(i * g_YPitch, y);
            }
            chart->axisX()->setRange(m_Form2D->m_Line.y1() * g_YPitch, m_Form2D->m_Line.y2() * g_YPitch);
        }
        else
        {
            for (int i = (int)m_Form2D->m_Line.y1(); i >= (int)m_Form2D->m_Line.y2(); i -= 4)
            {
                float x = fabs((i - b) / k);
                float y = g_Image.DepthImage.at<float>(i, x);

                ProfileVec.push_back(y);
                ProfileSeries->append(i * g_YPitch, y);
            }
            chart->axisX()->setRange(m_Form2D->m_Line.y2() * g_YPitch, m_Form2D->m_Line.y1() * g_YPitch);
        }
    }

    if (ProfileVec.empty()) return;
    float maxValue = *max_element(ProfileVec.begin(), ProfileVec.end());
    float minValue = *min_element(ProfileVec.begin(), ProfileVec.end());
    chart->axisY()->setRange(minValue, maxValue);
}

void CGProfileView::RectProfileHandle()
{
     m_Form2D->m_Line.setLine(m_Form2D->m_Rect.x(), m_Form2D->m_Rect.y() + m_Form2D->m_Rect.height()/2,
                              m_Form2D->m_Rect.x() + m_Form2D->m_Rect.width(), m_Form2D->m_Rect.y() + m_Form2D->m_Rect.height()/2);
     TwoPointLineProfileHandle();
}

void CGProfileView::CircleProfileHandle()
{
    ProfileVec.clear();
    ProfileSeries->clear();

    float r = 0;
    float x = m_Form2D->m_Rect.x();
    float y = m_Form2D->m_Rect.y();
    m_Form2D->m_Rect.width() == m_Form2D->m_Rect.height() ? r = m_Form2D->m_Rect.width() / 2 : r = m_Form2D->m_Rect.height() / 2;
    float a = x + r;
    float b = y + r;
    float c = r / 90;

    for (int i = 0; i < 360;  ++i)
    {
        float X = a + r * cos(i * M_PI / 180);
        float Y = b + r * sin(i * M_PI / 180);

        float V = g_Image.DepthImage.at<float>(Y, X);

        ProfileVec.push_back(V);
        ProfileSeries->append((x + i * c) * g_XPitch, V);
    }

    if (ProfileVec.empty()) return;
    float maxValue = *max_element(ProfileVec.begin(), ProfileVec.end());
    float minValue = *min_element(ProfileVec.begin(), ProfileVec.end());
    chart->axisY()->setRange(minValue, maxValue);
    chart->axisX()->setRange((x + 0 * c) * g_XPitch, (x + 360 * c) * g_XPitch);
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

void CGProfileView::PlotLineProfileHandle()
{
    m_Form2D->m_Line.setLine(CGImage2DGraphicsItemAdapter::getInstance()->GetLine().x1(),
                             CGImage2DGraphicsItemAdapter::getInstance()->GetLine().y1(),
                             CGImage2DGraphicsItemAdapter::getInstance()->GetLine().x2(),
                             CGImage2DGraphicsItemAdapter::getInstance()->GetLine().y2());

    if (m_Form2D->m_Line.x1() < 0 || m_Form2D->m_Line.x2() < 0 || m_Form2D->m_Line.x1() > g_Image.DepthImage.cols || m_Form2D->m_Line.x2() > g_Image.DepthImage.cols) return;
    if (m_Form2D->m_Line.y1() < 0 || m_Form2D->m_Line.y2() < 0 || m_Form2D->m_Line.y1() > g_Image.DepthImage.rows || m_Form2D->m_Line.y2() > g_Image.DepthImage.rows) return;

    switch (m_Form2D->m_CurrentToolType)
    {
    case CGProfileForm2D::ToolType::TwoPointLineTool:
        TwoPointLineProfileHandle();
        break;

    case CGProfileForm2D::ToolType::HorizontalLineTool:
        HorizontalLineProfileHandle();
        break;

    case CGProfileForm2D::ToolType::VerticalLineTool:
        VerticalLineProfileHandle();
        break;

    default:
        break;
    }
}

void CGProfileView::PlotRectProfileHandle()
{
    m_Form2D->m_Rect.setRect(CGImage2DGraphicsItemAdapter::getInstance()->GetRect().x(),
                             CGImage2DGraphicsItemAdapter::getInstance()->GetRect().y(),
                             CGImage2DGraphicsItemAdapter::getInstance()->GetRect().width(),
                             CGImage2DGraphicsItemAdapter::getInstance()->GetRect().height());

    if (m_Form2D->m_Rect.x() < 0 || m_Form2D->m_Rect.x() + m_Form2D->m_Rect.width() > g_Image.DepthImage.cols) return;
    if (m_Form2D->m_Rect.y() < 0 || m_Form2D->m_Rect.y() + m_Form2D->m_Rect.height() > g_Image.DepthImage.rows) return;

    switch (m_Form2D->m_CurrentToolType)
    {
    case CGProfileForm2D::ToolType::RectTool:
        RectProfileHandle();
        break;

    case CGProfileForm2D::ToolType::CircleTool:
        CircleProfileHandle();
        break;

    default:
        break;
    }
}

void CGProfileView::PlotArcProfileHandle()
{

}
