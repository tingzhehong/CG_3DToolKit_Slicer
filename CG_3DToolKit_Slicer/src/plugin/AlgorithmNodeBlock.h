#ifndef ALGORITHMNODEBLOCK_H
#define ALGORITHMNODEBLOCK_H

#include <QObject>
#include <NodeBlock.h>
#include <AlgorithmInterface.h>

class AlgorithmNodeBlock : public NodeBlock
{
    Q_OBJECT

public:
    explicit AlgorithmNodeBlock(NodeView *nodeview, QWidget *parent = nullptr);
    ~AlgorithmNodeBlock() = default;

    void Run() override;

public:
    void SetPlugin(AlgorithmInterface *plugin);
    AlgorithmInterface *GetPlugin() const;

private:
    AlgorithmInterface *m_plugin;

};

#endif // ALGORITHMNODEBLOCK_H
