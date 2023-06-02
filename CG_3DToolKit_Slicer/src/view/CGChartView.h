#ifndef CGCHARTVIEW_H
#define CGCHARTVIEW_H

#include <QtCharts>
#include <QChartView>
#include <QObject>
#include <QWidget>
#include <QWheelEvent>
#include <QMouseEvent>

class QLabel;
class QGraphicsLineItem;
class CGChartView : public QChartView
{
    Q_OBJECT

public:
    explicit CGChartView(QWidget* parent = nullptr);
    ~CGChartView() = default;

signals:
    void SignalMouseMovePoint(QPoint point);

private:
    QPoint BeginPoint;
    QPoint EndPoint;
    QPoint CurrentPoint;

    QGraphicsLineItem *LineV;
    QGraphicsLineItem *LineH;

    QGraphicsLineItem *Line1V;
    QGraphicsLineItem *Line2V;
    QGraphicsLineItem *Line1H;
    QGraphicsLineItem *Line2H;

    bool bMouseLeftPressed = false;
    bool bMouseRightPressed = false;
    bool bRemoveItem = true;

public:
    QLabel *ValueLable;
    QLabel *DeltaLable;

protected:
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
};

#endif // CGCHARTVIEW_H
