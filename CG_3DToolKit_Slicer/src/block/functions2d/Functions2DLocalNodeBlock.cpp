#include "Functions2DLocalNodeBlock.h"
#include "CGMetaType.h"
#include <QTextCodec>

Functions2DLocalNodeBlock::Functions2DLocalNodeBlock(NodeView *nodeview, QWidget *parent) : NodeBlock(nodeview, parent)
{
    m_NodeView = nodeview;
    m_IDCounter = nodeview->m_IDCounter;

    NodeItemFactory(tr(u8"2D本地图像"), 0, 1);

    m_NodeItem->m_portList.at(0)->setColor(Qt::yellow);
}

void Functions2DLocalNodeBlock::Run()
{
    m_NodeItem->PortClass();

    if (m_NodeItem->m_OutPortItem.size() != 1) return;

    QVariant var = m_NodeItem->m_Parameters.value(u8"文件");
    QString name = var.toString();
    QTextCodec *code = QTextCodec::codecForName("GB2312");
    std::string filename = code->fromUnicode(name).data();
    m_Image = cv::imread(filename, -1);

    m_NodeItem->m_OutPortItem.at(0)->setValue(QVariant::fromValue(m_Image));
    m_IsRuned = true;
}
