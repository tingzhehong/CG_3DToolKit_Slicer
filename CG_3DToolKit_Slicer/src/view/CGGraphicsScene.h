#ifndef CGGRAPHICSSCENE_H
#define CGGRAPHICSSCENE_H

#include <QGraphicsScene>

class CGGraphicsScene : public QGraphicsScene
{
    Q_OBJECT

public:
    explicit CGGraphicsScene(QObject *parent = nullptr);

    void startCreate();

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);

signals:
    void updatePoint(QPointF p, QList<QPointF> list, bool isCenter);
    void createFinished();

protected:
    QList<QPointF> Plist;
    bool PolygonFlg;
};

#endif // CGGRAPHICSSCENE_H
