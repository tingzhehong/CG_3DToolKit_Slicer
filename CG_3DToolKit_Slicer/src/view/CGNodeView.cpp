#include "CGNodeView.h"
#include <QLabel>
#include <QLineEdit>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QIcon>
#include <QDebug>
#include <QThreadPool>
#include <memory>
#include "NumberInputNodeBlock.h"
#include "NumberOutputNodeBlock.h"
#include "MathAddNodeBlock.h"
#include "MathDivNodeBlock.h"
#include "MathMulNodeBlock.h"
#include "MathSubNodeBlock.h"
#include "LogicsCondition.h"
#include "LogicsCirculate.h"
#include "LogicsGroup.h"
#include "Functions2DLocalNodeBlock.h"
#include "Functions3DLocalNodeBlock.h"
#include "Functions2DSourceNodeBlock.h"
#include "Functions3DSourceNodeBlock.h"
#include "Functions2DTerminalNodeBlock.h"
#include "Functions3DTerminalNodeBlock.h"
#include "NodeBlockFactory.h"
#include "NodeBlockManager.h"
#include "NodeBlockWidget.h"
#include "PluginManager.h"
#include <CGAlgorithmArgumentsDialog.h>
#include <CGLocalDataFileDialog.h>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>


CGNodeView::CGNodeView(QWidget *parent) : CGBaseWidget(parent)
{
    InitUi();
    InitConnections();
    setWindowTitle(tr(u8"流程节点"));
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
}

CGNodeView::~CGNodeView()
{

}

void CGNodeView::InitUi()
{
    m_NodeView = new NodeView(this);
    m_NodeView->setRopeFlexion(50);

    m_NodeBlockFactory = new NodeBlockFactory(this);

    m_NodeBlockManager = new NodeBlockManager(this);
    m_NodeBlockManager->setAutoDelete(false);

    m_CGAlgorithmArgumentsDialog = new CGAlgorithmArgumentsDialog();
    m_CGLocalDataFileDialog = new CGLocalDataFileDialog();

    //Test();
    //Verify();

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_NodeView);

    setLayout(pMainLayout);
}

void CGNodeView::InitConnections()
{
    connect(m_NodeView, &NodeView::signalRemoveNode, this, &CGNodeView::OnRemoveNodeBlock);
    connect(m_NodeView, &NodeView::signalDoubleClick, this, [&](bool b, unsigned int id, QString name) {
            if (b) { OnLoadAlgorithmArguments(b, id); m_CGAlgorithmArgumentsDialog->exec(); } 
            else if (name == u8"2D本地图像" || name == u8"3D本地点云") { OnLoadLocalDataFile(b, id); m_CGLocalDataFileDialog->exec(); } });
    connect(m_CGAlgorithmArgumentsDialog, &CGAlgorithmArgumentsDialog::SignalSetArguments, [this](){NodeBlockWidget::getInstance()->OnSendAlgorithmArguments();});
}

void CGNodeView::InitPluginManager()
{
    m_PluginManager = new PluginManager();

    if (m_PluginManager->LoadPlugin("./Algorithms"))
    {
        m_2DFuctionNames.append(m_PluginManager->m_PluginNames2D);
        m_3DFuctionNames.append(m_PluginManager->m_PluginNames3D);

        emit SignalAlgorithmPlugin(qMakePair(m_PluginManager->m_PluginNames2D, m_PluginManager->m_PluginNames3D));
        CGConsoleView::getInstance()->ConsoleOut(tr(u8"The app init plugins succeed."));
    }
}

void CGNodeView::CreateMathsNodeItem(const QString toolname)
{
         if (toolname == u8"数值/输入") {
            NumberInputNodeBlock *input = new NumberInputNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(input));
         }
    else if (toolname == u8"数值/输出") {
            NumberOutputNodeBlock *output = new NumberOutputNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(output));
         }
    else if (toolname == u8"加") {
            MathAddNodeBlock *add = new MathAddNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(add));
            m_NodeView->m_IDCounter++;
         }
    else if (toolname == u8"减") {
            MathSubNodeBlock *sub = new MathSubNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(sub));
            m_NodeView->m_IDCounter++;
         }
    else if (toolname == u8"乘") {
            MathMulNodeBlock *mul = new MathMulNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(mul));
            m_NodeView->m_IDCounter++;
         }
    else if (toolname == u8"除") {
            MathDivNodeBlock *div = new MathDivNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(div));
            m_NodeView->m_IDCounter++;
         }
    else {
            ;   
         }

}

void CGNodeView::CreateLogicsNodeItem(const QString toolname)
{
    if (toolname == u8"条件") {
        LogicsCondition *condition = new LogicsCondition(m_NodeView);
        m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(condition));
        m_NodeView->m_IDCounter++;
    }
    if (toolname == u8"循环") {
        LogicsCirculate *circulate = new LogicsCirculate(m_NodeView);
        circulate->FillNodeBlock(m_NodeBlockManager->m_NodeBlockList);
        m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(circulate));
        m_NodeView->m_IDCounter++;
    }
    if (toolname == u8"组") {
        LogicsGroup *gp = new LogicsGroup(m_NodeView);
        m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(gp));
    }
}

void CGNodeView::Create2DFuctionNodeItem(const QString toolname)
{
        if (toolname == u8"2D数据源") {
            Functions2DSourceNodeBlock *src2d = new Functions2DSourceNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(src2d));
            m_NodeView->m_IDCounter++;
         }
    else if (toolname == u8"2D数据终端") {
            Functions2DTerminalNodeBlock *terml2d = new Functions2DTerminalNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(terml2d));
            m_NodeView->m_IDCounter++;
            connect(terml2d, &Functions2DTerminalNodeBlock::SignalShow2D, this, [&](){emit Signal2DRequest();});
         }
    else if (toolname == u8"2D本地图像") {
            Functions2DLocalNodeBlock *local2d = new Functions2DLocalNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(local2d));
            m_NodeView->m_IDCounter++;
         }
    else {
            int index = m_2DFuctionNames.indexOf(toolname);
            if (index < 2) return;

            AlgorithmInterface *plugin = m_PluginManager->m_Plugins2D.at(index - 3)->Clone();
            CG_NODEBLOCK *obj = m_PluginManager->m_PluginObjects2D.at(index - 3);
            NodeBlock *algorithmNodeBlock = m_NodeBlockFactory->CreatNodeBlock(plugin, obj, m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(algorithmNodeBlock);
            m_NodeView->m_IDCounter++;
            connect(plugin, &AlgorithmInterface::SignalMessage, this, [=](const QString msg){CGConsoleView::getInstance()->ConsoleOut(msg);});
            m_PluginNodeBlockList[algorithmNodeBlock->m_NodeItem->m_NodeID] = plugin;
         }

}

void CGNodeView::Create3DFuctionNodeItem(const QString toolname)
{
        if (toolname == u8"3D数据源") {
            Functions3DSourceNodeBlock *src3d = new Functions3DSourceNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(src3d));
            m_NodeView->m_IDCounter++;
         }
    else if (toolname == u8"3D数据终端") {
            Functions3DTerminalNodeBlock *terml3d = new Functions3DTerminalNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(terml3d));
            m_NodeView->m_IDCounter++;
            connect(terml3d, &Functions3DTerminalNodeBlock::SignalShow3D, this, [&](){emit Signal3DRequest();});
         }
    else if (toolname == u8"3D本地点云") {
            Functions3DLocalNodeBlock *local3d = new Functions3DLocalNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(local3d));
            m_NodeView->m_IDCounter++;
         }
    else {
            int index = m_3DFuctionNames.indexOf(toolname);
            if (index < 2) return;

            AlgorithmInterface *plugin = m_PluginManager->m_Plugins3D.at(index - 3)->Clone();
            CG_NODEBLOCK *obj = m_PluginManager->m_PluginObjects3D.at(index - 3);
            NodeBlock *algorithmNodeBlock = m_NodeBlockFactory->CreatNodeBlock(plugin, obj, m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(algorithmNodeBlock);
            m_NodeView->m_IDCounter++;
            connect(plugin, &AlgorithmInterface::SignalMessage, this, [=](const QString msg){CGConsoleView::getInstance()->ConsoleOut(msg);});
            m_PluginNodeBlockList[algorithmNodeBlock->m_NodeItem->m_NodeID] = plugin;
         }

}

void CGNodeView::Run()
{
    m_NodeBlockManager->run();  //QThreadPool::globalInstance()->start(m_NodeBlockManager);
}

void CGNodeView::RunBlockUpdate()
{
    m_NodeBlockManager->m_RunBlockList.clear();

    foreach (NodeBlock *block, m_NodeBlockManager->m_NodeBlockList)
    {
        block->m_IsRuned = false;
        if (block->m_NodeItem->m_NodeID != 0)
            m_NodeBlockManager->m_RunBlockList.append(block);
    }
}

void CGNodeView::Test()
{
    PortItem *portIn = Q_NULLPTR;

    QWidget *widget = new QWidget;
    widget->resize(150, 30);

    QLabel *label = new QLabel("Node:",widget);
    label->resize(60, 20);
    label->move(2, 5);

    QLineEdit *lineEdit = new QLineEdit("123456", widget);
    lineEdit->resize(105, 20);
    lineEdit->move(40, 5);

    NodeItem *node = m_NodeView->createNode(widget);
    node->setTitle("Node Item");
    PortItem *portOut = node->createPortOut(-8, QColor(Qt::cyan));
    node->setPos(-250, 90);

    for (int i=0; i<5; ++i)
    {
        QWidget *widget = new QWidget;
        widget->resize(150, 30);

        QLabel *label = new QLabel("Node:",widget);
        label->resize(60, 20);
        label->move(2, 5);

        QLineEdit *lineEdit = new QLineEdit("123456",widget);
        lineEdit->resize(105, 20);
        lineEdit->move(40, 5);

        NodeItem *node = m_NodeView->createNode(widget);
        node->setTitle("Node Item " + QString::number(i + 1));
        node->setPos(i * 15, i * 60);

        portIn = node->createPortIn(-8, QColor(Qt::cyan));
        m_NodeView->createConnection(portOut, portIn);
    }

    GroupItem *group = m_NodeView->createGroup(m_NodeView->nodeList());
    group->setTitle("Group item");
    group->removeNode(node);
}

void CGNodeView::Verify()
{
    m_NodeView->NodeItemFactory("1#", 1, 1);
    m_NodeView->NodeItemFactory("2#", 2, 1);
    m_NodeView->NodeItemFactory("3#", 3, 1);
}

void CGNodeView::Flow2Node(const QString flowname)
{

}

void CGNodeView::Node2Flow(const QString flowname)
{
    QFile JsonFile(flowname);
    if (!JsonFile.open(QIODevice::WriteOnly))
    {
        qDebug() << "Write .flow json file failure!";
        return;
    }
    JsonFile.resize(0);

    QJsonDocument Doc;
    QJsonObject Obj;

    int i = 0;
    foreach (NodeBlock* block, m_NodeBlockManager->m_NodeBlockList)
    {
        QJsonDocument nodeDoc;
        QJsonArray nodeData;
        QJsonObject nodeObj;

        ++i;
        unsigned int id = block->m_NodeItem->m_NodeID;
        QString name = block->m_NodeItem->m_NodeName;
        QString title = block->m_NodeItem->m_title;
        qreal x = block->m_NodeItem->x();
        qreal y = block->m_NodeItem->y();

        nodeObj["id"] = QString::number(id);
        nodeObj["name"] = name;
        nodeObj["title"] = title;
        nodeObj["pos x"] = QString::number(x, 'f', 3);
        nodeObj["pos y"] = QString::number(y, 'f', 3);

        nodeData.append(nodeObj);
        nodeDoc.setArray(nodeData);

        QString Index = QString::number(i);
        Obj["node " + Index] = nodeDoc.toJson(QJsonDocument::Indented).toStdString().c_str();
    }
    Doc.setObject(Obj);

    JsonFile.write(Doc.toJson());
    JsonFile.close();
}

void CGNodeView::OnRemoveNodeBlock(unsigned int nodeId)
{
    foreach (NodeBlock* block, m_NodeBlockManager->m_NodeBlockList)
    {
        if (block->m_NodeItem->m_NodeID == nodeId)
        {
            m_NodeBlockManager->m_NodeBlockList.removeOne(block);
            m_PluginNodeBlockList.remove(nodeId);
            delete block;
            block = nullptr;
        }
    }
}

void CGNodeView::OnLoadAlgorithmArguments(bool b, unsigned int nodeId)
{
    if (!b) return;

    QString nodeName = NULL;
    foreach (NodeBlock* block, m_NodeBlockManager->m_NodeBlockList)
    {
        if (block->m_NodeItem->m_NodeID == nodeId)
        {
            nodeName = block->m_NodeItem->m_NodeName;
            NodeBlockWidget::getInstance()->SetCurrentNodeBlock(block);
            break;
        }
    }
    if (nodeName == NULL) return;

    QVector<CG_ARGUMENT> arguments;
    CG_SHOWDATA data;
    if (m_PluginManager->m_PluginNames2D.contains(nodeName))
    {
        AlgorithmInterface *plugin = m_PluginNodeBlockList.value(nodeId);
        arguments = plugin->GetAlgorithmArguments();
        data = plugin->GetAlgorithmShowData();
        NodeBlockWidget::getInstance()->SetCurrentAlgorithmPlugin(plugin);
    }
    if (m_PluginManager->m_PluginNames3D.contains(nodeName))
    {
        AlgorithmInterface *plugin = m_PluginNodeBlockList.value(nodeId);
        arguments = plugin->GetAlgorithmArguments();
        data = plugin->GetAlgorithmShowData();
        NodeBlockWidget::getInstance()->SetCurrentAlgorithmPlugin(plugin);
    }

    if (arguments.empty()) return;
    NodeBlockWidget::getInstance()->LoadAlgorithmArguments(arguments);

    if (data.Data.isNull()) return;
    NodeBlockWidget::getInstance()->LoadAlgorithmShowData(data);

    NodeBlockWidget::getInstance()->ShowAlgorithmPluginInfomation();
}

void CGNodeView::OnLoadLocalDataFile(bool b, unsigned int nodeId)
{
    if (b) return;

    QString nodeName = NULL;
    NodeBlock *nodeBlock;
    foreach (NodeBlock* block, m_NodeBlockManager->m_NodeBlockList)
    {
        if (block->m_NodeItem->m_NodeID == nodeId)
        {
            nodeName = block->m_NodeItem->m_NodeName;
            nodeBlock = block;
            break;
        }
    }
    m_CGLocalDataFileDialog->SetCurrentNodeBlock(nodeBlock);

    if (nodeName == NULL) {
        return;}
    if (nodeName == u8"2D本地图像") {
        m_CGLocalDataFileDialog->setWindowTitle(tr(u8"本地算子  文件设置  2D本地图像")); m_CGLocalDataFileDialog->m_2d3dfile = 2;}
    if (nodeName == u8"3D本地点云") {
        m_CGLocalDataFileDialog->setWindowTitle(tr(u8"本地算子  文件设置  3D本地点云")); m_CGLocalDataFileDialog->m_2d3dfile = 3;}

    QVariant var = nodeBlock->m_NodeItem->m_Parameters.value(u8"文件");
    QString str = var.toString();
    m_CGLocalDataFileDialog->m_pFilePath->setText(str);
}
