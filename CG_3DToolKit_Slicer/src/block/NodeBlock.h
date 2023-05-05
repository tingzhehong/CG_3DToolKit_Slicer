#ifndef NODEBLOCK_H
#define NODEBLOCK_H

#include <QWidget>
#include <NodeItem.h>
#include <NodeView.h>


class NodeBlock : public QWidget
{
    Q_OBJECT

public:
    explicit NodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~NodeBlock() = default;

public:
    NodeView *m_NodeView;
};

#endif // NODEBLOCK_H
