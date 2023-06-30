#ifndef LOGICSGROUP_H
#define LOGICSGROUP_H

#include <QObject>
#include <NodeBlock.h>
#include <GroupItem.h>

class QPushButton;
class QLineEdit;
class LogicsGroup : public NodeBlock
{
    Q_OBJECT

public:
    explicit LogicsGroup(NodeView *nodeview, QWidget *parent = nullptr);
    ~LogicsGroup() = default;

public:
    void Connections();
    void Run() override;

private slots:
    void AddNodeBlock();
    void DelNodeBlock();

protected:
    NodeItem *CreatNodeItem00(const QString nodename) override;

private:
    GroupItem *group;
    NodeItem *mainnode;
    QPushButton *addNode;
    QPushButton *delNode;
    QLineEdit *lineEdit;
};

#endif // LOGICSGROUP_H
