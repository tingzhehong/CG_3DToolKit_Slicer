#include "CGGraphicsScene.h"
#include <QGraphicsSceneMouseEvent>

CGGraphicsScene::CGGraphicsScene(QObject *parent) : QGraphicsScene(parent)
{
    PolygonFlg = false;
}

void CGGraphicsScene::startCreate()
{
    PolygonFlg = true;
    Plist.clear();
}

void CGGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (PolygonFlg)
    {
        QPointF p(event->scenePos().x(), event->scenePos().y());
        switch (event->buttons())
        {
        case Qt::LeftButton: {
            Plist.push_back(p);
            emit updatePoint(p, Plist, false);
        } break;
        case Qt::RightButton: {
            if (Plist.size() >= 3) {
                emit updatePoint(p, Plist, true);
                emit createFinished();
                PolygonFlg = false;
                Plist.clear();
            }
        } break;
        default: break;
        }
    }
    else {
        QGraphicsScene::mousePressEvent(event);
    }
}
