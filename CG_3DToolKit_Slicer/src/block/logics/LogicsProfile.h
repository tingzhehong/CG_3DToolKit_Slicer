#ifndef LOGICSPROFILE_H
#define LOGICSPROFILE_H

#include <QObject>
#include <NodeBlock.h>

class LogicsProfile : public NodeBlock
{
    Q_OBJECT

public:
    explicit LogicsProfile(NodeView *nodeview, QWidget *parent = nullptr);
    ~LogicsProfile() = default;

signals:
    void SignalShow2D();
    void SignalShow3D();

public:
    void Run() override;

protected:
    NodeItem *CreatNodeItem20(const QString nodename);

};

#endif // LOGICSPROFILE_H
