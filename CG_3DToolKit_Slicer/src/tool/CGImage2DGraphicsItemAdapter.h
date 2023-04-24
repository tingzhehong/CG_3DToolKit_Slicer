#ifndef CGIMAGE2DGRAPHICSITEMADAPTER_H
#define CGIMAGE2DGRAPHICSITEMADAPTER_H

#include <QObject>
#include <QLineF>
#include <QRectF>

class CGImage2DGraphicsItemAdapter : public QObject
{
    Q_OBJECT

private:
    explicit CGImage2DGraphicsItemAdapter(QObject *parent = nullptr);
    ~CGImage2DGraphicsItemAdapter() = default;

public:
    static CGImage2DGraphicsItemAdapter *getInstance();

    void SendLine(QLineF line);
    void SendRect(QRectF rect);
    void SendAngle(qreal angle);

    QLineF GetLine() const;
    QRectF GetRect() const;
    qreal  GetAngle() const;

    bool m_Status = false;
    int  m_SendType = 0;   // 0 = line, 1 = rect, 2 = arc

private:
    static CGImage2DGraphicsItemAdapter *m_CGImage2DGraphicsItemAdapter;

public:
    QLineF  m_Line;
    QRectF  m_Rect;
    qreal   m_Angle;

};

#endif // CGIMAGE2DGRAPHICSITEMADAPTER_H
