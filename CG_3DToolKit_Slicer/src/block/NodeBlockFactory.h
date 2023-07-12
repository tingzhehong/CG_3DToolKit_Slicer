#ifndef NODEBLOCKFACTORY_H
#define NODEBLOCKFACTORY_H

#include <QObject>
#include <NodeBlock.h>
#include <NodeBlockWidget.h>
#include <AlgorithmInterface.h>

class NodeBlockFactory : public QObject
{
    Q_OBJECT

public:
    explicit NodeBlockFactory(QObject *parent = nullptr);
    ~NodeBlockFactory() = default;

public:
    NodeBlock *CreatNodeBlock(AlgorithmInterface *plugin, CG_NODEBLOCK *pluginobj, NodeView *nodeview);

private:

};

#endif // NODEBLOCKFACTORY_H
