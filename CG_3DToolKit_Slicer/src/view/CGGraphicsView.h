﻿#ifndef CGGRAPHICSVIEW_H
#define CGGRAPHICSVIEW_H

#include <QGraphicsView>

class CGGraphicsView : public QGraphicsView
{
    Q_OBJECT

public:
    explicit CGGraphicsView(QWidget* parent = nullptr);
    ~CGGraphicsView();

signals:
    void SignalCoordinate(const QPair<qreal, qreal> Coordinate);

public:
    void InintGraphicsView();
    void ResetGraphicsView();
    void RemoveALLItems();
    void RemoveToolItems();
    void ZoomIn();
    void ZoomOut();
    void AutoFit();

    void InstallFilter();
    void RemoveFilter();

protected:
    void _wheelEvent(QWheelEvent *event);
    void _mousePressEvent(QMouseEvent *event);
    void _mouseReleaseEvent(QMouseEvent *event);
    void _mouseMoveEvent(QMouseEvent *event);
    void _mouseDoubleClickEvent(QMouseEvent *event);

    bool eventFilter(QObject *watched, QEvent *event);

public:
    int ImageWidth = 0;
    int ImageHeight = 0;

private:
    QPointF m_LastPointF;
    QPointF m_CurrentPointF;
    qreal   m_Scale = 1;
    bool    m_MousePressed;

    qreal X, Y;
    QPair<qreal, qreal> m_Coordinate;
};

#endif // CGGRAPHICSVIEW_H
