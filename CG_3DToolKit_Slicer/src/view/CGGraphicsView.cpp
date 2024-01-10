#include "CGGraphicsView.h"
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsPixmapItem>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QDebug>


CGGraphicsView::CGGraphicsView(QWidget *parent): QGraphicsView(parent)
{
    InintGraphicsView();
}

CGGraphicsView::~CGGraphicsView()
{

}

void CGGraphicsView::InintGraphicsView()
{
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setCursor(Qt::PointingHandCursor);
    setRenderHint(QPainter::Antialiasing);
    setBackgroundRole(QPalette::Dark);
    setStyleSheet("background-color:rgb(25, 50, 75)");
    setFrameStyle(Qt::FramelessWindowHint);
    InstallFilter();
}

void CGGraphicsView::ResetGraphicsView()
{
    scene()->setSceneRect(mapToScene(rect()).boundingRect());
    scene()->setSceneRect(ImageWidth / 2 - rect().width() / 2,
                          ImageHeight / 2 - rect().height() / 2,
                          scene()->sceneRect().width(),
                          scene()->sceneRect().height());
    scene()->update();

    m_Scale = scene()->sceneRect().height() / ImageHeight;
    scale(m_Scale, m_Scale);
    scene()->setSceneRect(mapToScene(rect()).boundingRect());
    scene()->update();
}

void CGGraphicsView::RemoveALLItems()
{
    QList<QGraphicsItem *> ListItems = scene()->items();

    scene()->items().clear();
    int num = ListItems.count();
    for (int i = 0; i < num; i++)
    {
        scene()->removeItem(ListItems[i]);
    }
    scene()->update();
}

void CGGraphicsView::RemoveToolItems()
{
    QList<QGraphicsItem *> ListItems = scene()->items();

    scene()->items().clear();
    int num = ListItems.count();
    for (int j = 0; j < num - 1; j++)
    {
        scene()->removeItem(ListItems[j]);
    }
    scene()->update();
}

void CGGraphicsView::ZoomIn()
{
    m_Scale *= 1.1;
    scale(1.1,1.1);

    scene()->setSceneRect(mapToScene(rect()).boundingRect());
    scene()->update();
}

void CGGraphicsView::ZoomOut()
{
    m_Scale *= 0.9;
    scale(0.9,0.9);

    scene()->setSceneRect(mapToScene(rect()).boundingRect());
    scene()->update();
}

void CGGraphicsView::AutoFit()
{
    if (ImageWidth == 0) return;
    if (ImageHeight == 0) return;

    qreal ViewWidth = width();
    qreal ViewHeight = height();
    qreal ScaleWidth = ViewWidth / (ImageWidth / 2);
    qreal ScaleHeight = ViewHeight / (ImageHeight / 2);
    qreal s = ScaleWidth > ScaleHeight ? ScaleWidth : ScaleHeight;

    m_Scale *= s;
    scale(s,s);

    if (!scene()) return;
    scene()->setSceneRect(mapToScene(rect()).boundingRect());
    scene()->update();
}

void CGGraphicsView::InstallFilter()
{
    viewport()->installEventFilter(this);
    setMouseTracking(false);
}

void CGGraphicsView::RemoveFilter()
{
    viewport()->removeEventFilter(this);
    setMouseTracking(true);
}

void CGGraphicsView::_wheelEvent(QWheelEvent *event)
{
    if (event->delta() > 0)
    {
        m_Scale *= 1.1;
        scale(1.1,1.1);
    }
    else
    {
        m_Scale *= 0.9;
        scale(0.9,0.9);
    }

    if (!scene()) return;
    scene()->setSceneRect(mapToScene(rect()).boundingRect());
    scene()->update();
}

void CGGraphicsView::_mousePressEvent(QMouseEvent *event)
{
    m_MousePressed = true;
    m_LastPointF = event->pos() * (1 / m_Scale);
}

void CGGraphicsView::_mouseReleaseEvent(QMouseEvent *event)
{
    m_MousePressed = false;
    Q_UNUSED(event);
}

void CGGraphicsView::_mouseMoveEvent(QMouseEvent *event)
{
    if (m_MousePressed)
    {
        setMouseTracking(false);
        m_CurrentPointF = event->pos() * (1 / m_Scale) - m_LastPointF;
        m_LastPointF = event->pos() * (1 / m_Scale);

        if (!scene()) return;
        scene()->setSceneRect(scene()->sceneRect().x() - m_CurrentPointF.x(),
                              scene()->sceneRect().y() - m_CurrentPointF.y(),
                              scene()->sceneRect().width(),
                              scene()->sceneRect().height());
        scene()->update();
    }
    {
        setMouseTracking(true);
        X = mapToScene(event->pos()).x();
        Y = mapToScene(event->pos()).y();
        m_Coordinate = qMakePair(X, Y);
        emit SignalCoordinate(m_Coordinate);
    }
}

void CGGraphicsView::_mouseDoubleClickEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
}

bool CGGraphicsView::eventFilter(QObject *watched, QEvent *event)
{
    if (event->type() == QEvent::Wheel)
    {
        _wheelEvent(static_cast<QWheelEvent*>(event));
    }
    else if (event->type() == QEvent::MouseButtonPress)
    {
        _mousePressEvent(static_cast<QMouseEvent*>(event));
    }
    else if (event->type() == QEvent::MouseButtonRelease)
    {
        _mouseReleaseEvent(static_cast<QMouseEvent*>(event));
    }
    else if (event->type() == QEvent::MouseMove)
    {
        _mouseMoveEvent(static_cast<QMouseEvent*>(event));
    }
    else
    {
        return false;
    }

    return true;

    Q_UNUSED(watched);
}
