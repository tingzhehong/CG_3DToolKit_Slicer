#ifndef NODEBLOCKMANAGER_H
#define NODEBLOCKMANAGER_H

#include <QObject>
#include <QRunnable>
#include <NodeBlock.h>

#define RUNMAXLOOP 100

class NodeBlockManager : public QObject , public QRunnable
{
    Q_OBJECT

public:
    explicit NodeBlockManager(QObject *parent = nullptr);

public:
    void run() override;

public:
    QList<NodeBlock*> m_NodeBlockList;
    QList<NodeBlock*> m_RunBlockList;

};

#endif // NODEBLOCKMANAGER_H
