#ifndef CGNODEVIEW_H
#define CGNODEVIEW_H

#include <CGBaseWidget.h>
#include <NodeView.h>
#include <NodeItem.h>

class NodeBlock;
class CGNodeView : public CGBaseWidget
{
    Q_OBJECT

public:
    explicit CGNodeView(QWidget *parent = nullptr);
    ~CGNodeView();

public:
    void InitUi() override;
    void InitConnections() override;
    void CreateMathsNodeItem(const QString toolname);
    void CreateLogicsNodeItem(const QString toolname);

    void Run();
    void RunBlockUpdate();

private:
    void Test();
    void Verify();

public slots:
    void OnRemoveNodeBlock(unsigned int nodeId);

protected:
    QStringList m_MathsNames{u8"加", u8"减", u8"乘", u8"除", u8"数值/输入", u8"数值/输出"};
    QStringList m_LogicsNames{u8"条件", u8"循环"};
    QStringList m_2DFuctionNames{u8"2D数据源"};
    QStringList m_3DFuctionNames{u8"3D数据源"};

    QList<NodeBlock*> m_NodeBlockList;
    QList<NodeBlock*> m_RunBlockList;

public:
    NodeView *m_NodeView;
};

#endif // CGNODEVIEW_H
