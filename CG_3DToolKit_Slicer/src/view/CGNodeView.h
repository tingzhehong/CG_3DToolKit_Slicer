#ifndef CGNODEVIEW_H
#define CGNODEVIEW_H

#include <CGBaseWidget.h>
#include <CGConsoleView.h>
#include <NodeView.h>
#include <NodeItem.h>

class NodeBlock;
class NodeBlockFactory;
class NodeBlockManager;
class AlgorithmInterface;
class PluginManager;
class CGAlgorithmArgumentsDialog;

class CGNodeView : public CGBaseWidget
{
    Q_OBJECT

public:
    explicit CGNodeView(QWidget *parent = nullptr);
    ~CGNodeView();

signals:
    void Signal2DRequest(const int type = 2);
    void Signal3DRequest(const int type = 3);
    void SignalAlgorithmPlugin(QPair<QStringList, QStringList> names);

public:
    void InitUi() override;
    void InitConnections() override;
    void InitPluginManager();
    void CreateMathsNodeItem(const QString toolname);
    void CreateLogicsNodeItem(const QString toolname);
    void Create2DFuctionNodeItem(const QString toolname);
    void Create3DFuctionNodeItem(const QString toolname);

    void Run();
    void RunBlockUpdate();

private:
    void Test();
    void Verify();

public slots:
    void OnRemoveNodeBlock(unsigned int nodeId);
    void OnLoadAlgorithmArguments(bool b, unsigned int nodeId);

protected:
    QStringList m_MathsNames{u8"加", u8"减", u8"乘", u8"除", u8"数值/输入", u8"数值/输出"};
    QStringList m_LogicsNames{u8"条件", u8"循环", u8"组"};
    QStringList m_2DFuctionNames{u8"2D数据源", u8"2D数据终端"};
    QStringList m_3DFuctionNames{u8"3D数据源", u8"3D数据终端"};

    //Deprecated
    //QList<NodeBlock*> m_NodeBlockList;
    //QList<NodeBlock*> m_RunBlockList;

public:
    NodeView *m_NodeView;
    NodeBlockFactory *m_NodeBlockFactory;
    NodeBlockManager *m_NodeBlockManager;
    PluginManager *m_PluginManager;
    CGAlgorithmArgumentsDialog *m_CGAlgorithmArgumentsDialog;
};

#endif // CGNODEVIEW_H
