#ifndef FUNCTIONS3DLOCALNODEBLOCK_H
#define FUNCTIONS3DLOCALNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>
#include <CGPCLHeader.h>

class Functions3DLocalNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit Functions3DLocalNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~Functions3DLocalNodeBlock() = default;

public:
    void Run() override;

private:
    PointCloudT::Ptr m_PointCloud;

};

#endif // FUNCTIONS3DLOCALNODEBLOCK_H
