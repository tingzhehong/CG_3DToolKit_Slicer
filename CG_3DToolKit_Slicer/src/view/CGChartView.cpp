#include "CGChartView.h"
#include <QPointF>
#include <QLabel>
#include <QGraphicsLineItem>
#include <QDebug>

CGChartView::CGChartView(QWidget *parent): QChartView(parent)
{
    ValueLable = new QLabel("0.000", this);
    ValueLable->setStyleSheet("color: red; font: bold");
    ValueLable->clear();

    DeltaLable = new QLabel("0.000", this);
    DeltaLable->setStyleSheet("color: red; font: bold");
    DeltaLable->clear();

    LineV = new QGraphicsLineItem();
    LineV->setFlag(QGraphicsItem::ItemIsSelectable, true);
    LineV->setFlag(QGraphicsItem::ItemIsMovable, true);
    LineV->setFlag(QGraphicsItem::ItemIsFocusable, true);
    LineV->setPen(QPen(Qt::red, 2, Qt::DashLine));

    LineH = new QGraphicsLineItem();
    LineH->setFlag(QGraphicsItem::ItemIsSelectable, true);
    LineH->setFlag(QGraphicsItem::ItemIsMovable, true);
    LineH->setFlag(QGraphicsItem::ItemIsFocusable, true);
    LineH->setPen(QPen(Qt::blue, 2, Qt::DashLine));

    Line1V = new QGraphicsLineItem();
    Line1V->setFlag(QGraphicsItem::ItemIsSelectable, true);
    Line1V->setFlag(QGraphicsItem::ItemIsMovable, true);
    Line1V->setFlag(QGraphicsItem::ItemIsFocusable, true);
    Line1V->setPen(QPen(Qt::yellow, 2, Qt::DashLine));

    Line2V = new QGraphicsLineItem();
    Line2V->setFlag(QGraphicsItem::ItemIsSelectable, true);
    Line2V->setFlag(QGraphicsItem::ItemIsMovable, true);
    Line2V->setFlag(QGraphicsItem::ItemIsFocusable, true);
    Line2V->setPen(QPen(Qt::yellow, 2, Qt::DashLine));

    Line1H = new QGraphicsLineItem();
    Line1H->setFlag(QGraphicsItem::ItemIsSelectable, true);
    Line1H->setFlag(QGraphicsItem::ItemIsMovable, true);
    Line1H->setFlag(QGraphicsItem::ItemIsFocusable, true);
    Line1H->setPen(QPen(Qt::red, 1, Qt::DashLine));

    Line2H = new QGraphicsLineItem();
    Line2H->setFlag(QGraphicsItem::ItemIsSelectable, true);
    Line2H->setFlag(QGraphicsItem::ItemIsMovable, true);
    Line2H->setFlag(QGraphicsItem::ItemIsFocusable, true);
    Line2H->setPen(QPen(Qt::red, 1, Qt::DashLine));
}

void CGChartView::wheelEvent(QWheelEvent *event)
{
    ValueLable->clear();
    DeltaLable->clear();
    scene()->removeItem(LineV);
    scene()->removeItem(LineH);
    scene()->removeItem(Line1V);
    scene()->removeItem(Line1H);
    scene()->removeItem(Line2V);
    scene()->removeItem(Line2H);
    bRemoveItem = true;

    if (event->delta() > 0)
    {
        chart()->zoom(1.1);
    }
    else
    {
        chart()->zoom(0.9);
    }
    QChartView::wheelEvent(event);
}

void CGChartView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        QPoint pt = event->pos();
        QPointF ptf = chart()->mapToValue(pt);
        ValueLable->setText(QString::asprintf(" X=%0.3f  Z=%0.3f", ptf.x(), ptf.y()));
        ValueLable->setGeometry(pt.x(), pt.y() - 15, 130, 10);
        DeltaLable->clear();

        CurrentPoint = pt;
        QLineF theLine(ptf, ptf);
        LineV->setLine(theLine);
        LineH->setLine(theLine);
        Line1V->setLine(theLine);
        Line1H->setLine(theLine);
        Line2V->setLine(theLine);
        Line2H->setLine(theLine);

        if (bRemoveItem)
        {
            scene()->addItem(LineV);
            scene()->addItem(LineH);
            scene()->addItem(Line1V);
            scene()->addItem(Line1H);
            scene()->addItem(Line2V);
            scene()->addItem(Line2H);
            bRemoveItem = false;
        }
        bMouseLeftPressed = true;
    }
    if (event->button() == Qt::RightButton)
    {
        ValueLable->clear();
        DeltaLable->clear();
        scene()->removeItem(LineV);
        scene()->removeItem(LineH);
        scene()->removeItem(Line1V);
        scene()->removeItem(Line1H);
        scene()->removeItem(Line2V);
        scene()->removeItem(Line2H);

        QPoint pt = event->pos();
        BeginPoint = pt;
        bMouseRightPressed = true;
        bRemoveItem = true;
    }
    QChartView::mousePressEvent(event);
}

void CGChartView::mouseReleaseEvent(QMouseEvent *event)
{
    bMouseLeftPressed = false;
    bMouseRightPressed = false;

    Q_UNUSED(event);
}

void CGChartView::mouseMoveEvent(QMouseEvent *event)
{
    QPoint pt = event->pos();

    if (bMouseLeftPressed)
    {
        QLineF theLineV(QPointF(CurrentPoint.x() + (event->pos().x() - CurrentPoint.x()) / 2, CurrentPoint.y()),
                        QPointF(CurrentPoint.x() + (event->pos().x() - CurrentPoint.x()) / 2, event->pos().y()));

        QLineF theLineH(QPointF(CurrentPoint.x(), event->pos().y() - 3),
                        QPointF(event->pos().x(), event->pos().y() - 3));

        LineV->setLine(theLineV);
        LineH->setLine(theLineH);

        Line1V->setLine(CurrentPoint.x(), 90, CurrentPoint.x(), 320);
        Line2V->setLine(pt.x(), 90, pt.x(), 320);

        QLineF theLineH1(QPointF(CurrentPoint.x(), CurrentPoint.y()),
                        QPointF(event->pos().x(), CurrentPoint.y()));

        QLineF theLineH2(QPointF(CurrentPoint.x(), event->pos().y()),
                        QPointF(event->pos().x(), event->pos().y()));

        Line1H->setLine(theLineH1);
        Line2H->setLine(theLineH2);

        QPoint pt2 = event->pos();
        QPointF ptf2 = chart()->mapToValue(pt2);
        QPointF ptf1 = chart()->mapToValue(CurrentPoint);
        qreal dx = ptf2.x() - ptf1.x();
        qreal dy = ptf2.y() - ptf1.y();
        DeltaLable->setText(QString::asprintf(" (%0.3f, %0.3f)", dx, dy));
        DeltaLable->setGeometry(pt.x(), pt.y(), 120, 10);
    }

    if (bMouseRightPressed)
    {
        EndPoint = pt - BeginPoint;
        BeginPoint = pt;
        chart()->scroll(-EndPoint.x(), EndPoint.y());
    }
    QChartView::mouseMoveEvent(event);
}
