#include "CGImage2DGraphicsItemAdapter.h"


CGImage2DGraphicsItemAdapter *CGImage2DGraphicsItemAdapter:: m_CGImage2DGraphicsItemAdapter = nullptr;


CGImage2DGraphicsItemAdapter::CGImage2DGraphicsItemAdapter(QObject *parent) : QObject(parent)
{

}

CGImage2DGraphicsItemAdapter *CGImage2DGraphicsItemAdapter::getInstance()
{
    if (!m_CGImage2DGraphicsItemAdapter)
    {
        m_CGImage2DGraphicsItemAdapter = new CGImage2DGraphicsItemAdapter();
    }
    return m_CGImage2DGraphicsItemAdapter;
}

void CGImage2DGraphicsItemAdapter::SendLine(QLineF line)
{
    m_Line = line;
}

void CGImage2DGraphicsItemAdapter::SendRect(QRectF rect)
{
    m_Rect = rect;
}

void CGImage2DGraphicsItemAdapter::SendAngle(qreal angle)
{
    m_Angle = angle;
}

QLineF CGImage2DGraphicsItemAdapter::GetLine() const
{
    return m_Line;
}

QRectF CGImage2DGraphicsItemAdapter::GetRect() const
{
    return m_Rect;
}

qreal CGImage2DGraphicsItemAdapter::GetAngle() const
{
    return m_Angle;
}
