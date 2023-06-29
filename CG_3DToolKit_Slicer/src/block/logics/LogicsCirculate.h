#ifndef LOGICSCIRCULATE_H
#define LOGICSCIRCULATE_H

#include <QObject>
#include <NodeBlock.h>
#include <GroupItem.h>

class QPushButton;
class QLineEdit;
class LogicsCirculate : public NodeBlock
{
    Q_OBJECT

public:
    explicit LogicsCirculate(NodeView *nodeview, QWidget *parent = nullptr);
    ~LogicsCirculate() = default;

public:
    void Connections();
    void Run() override;

private slots:
    void AddNodeBlock();
    void DelNodeBlock();

protected:
    NodeItem *CreatNodeItem10(const QString nodename) override;

private:
    GroupItem *group;
    NodeItem *mainnode;
    QPushButton *addNode;
    QPushButton *delNode;
    QLineEdit *lineEdit;
};

#endif // LOGICSCIRCULATE_H
