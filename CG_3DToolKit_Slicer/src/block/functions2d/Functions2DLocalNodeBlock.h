#ifndef FUNCTIONS2DLOCALNODEBLOCK_H
#define FUNCTIONS2DLOCALNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>
#include <opencv2/opencv.hpp>

class Functions2DLocalNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit Functions2DLocalNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~Functions2DLocalNodeBlock() = default;

public:
    void Run() override;

private:
    cv::Mat m_Image;

};

#endif // FUNCTIONS2DLOCALNODEBLOCK_H
