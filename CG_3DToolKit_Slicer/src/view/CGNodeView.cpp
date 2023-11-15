#include "CGNodeView.h"
#include <QLabel>
#include <QLineEdit>
#include <QTextEdit>
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
#include "LogicsScriptCpp.h"
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
#include <CGScriptCppEditor.h>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <QByteArray>
#include <QDateTime>


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
    m_CGScriptCppEditor = new CGScriptCppEditor();

    //Test();
    //Verify();

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_NodeView);

    setLayout(pMainLayout);
}

void CGNodeView::InitConnections()
{
    connect(m_NodeView, &NodeView::signalRemoveNode, this, &CGNodeView::OnRemoveNodeBlock);
    connect(m_NodeView, &NodeView::signalDoubleClick, this, [&](bool b, int id, QString name) {
            if (b) { OnLoadAlgorithmArguments(b, id); m_CGAlgorithmArgumentsDialog->exec(); } 
            else if (name == u8"2D本地图像" || name == u8"3D本地点云") { OnLoadLocalDataFile(b, id); m_CGLocalDataFileDialog->exec(); }
            else if (name == u8"脚本") { OnLoadScriptCpp(b, id); m_CGScriptCppEditor->exec(); } });
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
            m_NodeView->m_IDCounterMinus--;
         }
    else if (toolname == u8"数值/输出") {
            NumberOutputNodeBlock *output = new NumberOutputNodeBlock(m_NodeView);
            m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(output));
            m_NodeView->m_IDCounterMinus--;
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
        m_NodeView->m_IDCounterMinus--;
    }
    if (toolname == u8"脚本") {
       LogicsScriptCpp *cpp = new LogicsScriptCpp(m_NodeView);
       m_NodeBlockManager->m_NodeBlockList.append(dynamic_cast<NodeBlock*>(cpp));
       m_NodeView->m_IDCounter++;
       connect(cpp, &LogicsScriptCpp::SignalMessage, this, [=](const QString msg){CGConsoleView::getInstance()->ConsoleOut(msg);});
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

int CGNodeView::ParentNodeID(PortItem *port)
{
    int id = -9999;

    NodeItem *parentNode = Q_NULLPTR;

    if (port->parentItem())
        parentNode = (NodeItem*)port->parentItem();

    if (parentNode)
       id = parentNode->m_NodeID;

    return id;
}

void CGNodeView::Flow2Node(const QString flowname)
{
    QFileInfo Infor(flowname);
    if (Infor.suffix() != "flow") 
        return;

    QFile JsonFile(flowname);
    if (!JsonFile.open(QIODevice::ReadOnly))
    {
        qDebug() << "Read .flow json file failure!";
        return;
    }
    QByteArray data = JsonFile.readAll();
    JsonFile.close();
    
    QJsonDocument doc = QJsonDocument::fromJson(data);

    if (!doc.isObject())
        return;

     QJsonObject obj = doc.object();

     /// Parse Json
     ///

     QJsonObject infoObj = obj["node infomation"].toObject();
     int num = infoObj["node number"].toString().toInt();
     int mun = infoObj["rope number"].toString().toInt();
     qDebug() << "node number: " << num;
     qDebug() << "rope number: " << mun;
     //
     for (int i = 1; i <= num; ++i)
     {
         QString Index = QString::number(i);
         QJsonObject nodeObj = obj["node block " + Index].toObject();

         int id = nodeObj["id"].toString().toInt();
         QString name = nodeObj["name"].toString();
         QString title = nodeObj["title"].toString();
         qreal x = nodeObj["pos x"].toString().toDouble();
         qreal y = nodeObj["pos y"].toString().toDouble();
         QPointF pos(x, y);

         qDebug() << i << " " << id << " " << name << " " << title << " " << pos;

         /// Create Node
         ///

              if (m_MathsNames.contains(name)) {
                  CreateMathsNodeItem(name);
         }
         else if (m_LogicsNames.contains(name)) {
                  CreateLogicsNodeItem(name);
         }
         else if (m_2DFuctionNames.contains(name)) {
                  Create2DFuctionNodeItem(name);
         }
         else if (m_3DFuctionNames.contains(name)) {
                  Create3DFuctionNodeItem(name);
         }
         else {
                  ;
         }
         NodeBlock *block = m_NodeBlockManager->m_NodeBlockList.back();
         block->m_NodeItem->setNodeID(id);
         block->m_NodeItem->setNodeName(name);
         block->m_NodeItem->setTitle(title);
         block->m_NodeItem->setPos(pos);

         QJsonObject argsObj = nodeObj["arguments"].toObject();
         QStringList argsList = argsObj.keys();

         int n = argsObj.count();
         int m = argsList.count();
         if (n > 0)
         {
             for (int j = 0; j < m; ++j)
             {
                 qreal val = argsObj[argsList.at(j)].toString().toDouble();
                 block->m_NodeItem->m_Parameters[argsList.at(j)] = val;
                 qDebug() << argsList[j] << " " << val;
             }
             //Only algorithm plugin have arguments!
             OnReLoadAlgorithmArguments(id);

             //Number node block input parameters
             if (name == u8"数值/输入" || name == u8"数值/输出") {
                 block->m_NodeItem->m_Parameters[u8"值"] = argsObj[argsList.at(0)].toString();
                 block->m_NodeItem->SendParameters();
             }

             //Group node block parameters
             if (name == u8"循环" || name == u8"组") {
                for (int k = 0; k < m; ++k)
                {
                    qDebug() << "group node block id: " << argsObj[argsList.at(k)].toString();

                    for(GroupItem *group : m_NodeView->m_groupList) {
                        //qDebug() << " node id " << id;
                        //qDebug() << "group id: " << group->GroupID();
                        if (group->GroupID() == id) {
                            group->m_nodeListString.append(argsObj[argsList.at(k)].toString());
                            break;
                        }
                    }
                }
             }

             //Local node block parameters
             if (name == u8"2D本地图像" || name == u8"3D本地点云") {
                 block->m_NodeItem->m_Parameters[u8"文件"] = argsObj[argsList.at(0)].toString();
             }

             //Script Cpp node block parameters
             if (name == u8"脚本") {
                block->m_NodeItem->m_Parameters[u8"代码"] = argsObj[argsList.at(0)].toString();
                block->m_NodeItem->m_Parameters[u8"端口"] = argsObj[argsList.at(1)].toString();

                QStringList IOList = block->m_NodeItem->m_Parameters[u8"端口"].toString().split(",");

                for (int m = 0; m < IOList.count() - 1; ++m)
                {
                    QColor Color = IOList[m];

                    if (Color != Qt::white)
                    {
                        if (m < 3)
                            block->m_NodeItem->createPortIn(8 + m * 16, Color);
                        else
                            block->m_NodeItem->createPortOut(8 + (m - 3) * 16, Color);
                    }
                }
            }
            
         }
     }
     //
     qDebug() << "IDCounter: " << m_NodeView->m_IDCounter - 1;
     qDebug() << "IDCounterMinus: " << m_NodeView->m_IDCounterMinus + 1;
     ///

     /// Group Handle
     ///
     for(GroupItem *group : m_NodeView->m_groupList)
     {
         for(NodeItem *node : m_NodeView->m_nodeList)
         {
             if (group->m_nodeListString.contains(QString::number(node->m_NodeID))) {
                 group->addNode(node);
                 qDebug() << "group add node block: " << node->m_NodeID;
             }
         }
     }
     ///

     /// Create Connection
     ///

     for (int j = 1; j <= mun; ++j)
     {
         QString Index = QString::number(j);
         QJsonObject ropeObj = obj["rope connection " + Index].toObject();

         int nodeOut = ropeObj["node block id out"].toString().toInt();
         int nodeIn = ropeObj["node block id in"].toString().toInt();
         int portOutNum = ropeObj["port number out"].toString().toInt();
         int portInNum = ropeObj["port number in"].toString().toInt();

          PortItem *portOut;
          PortItem *portIn;
          foreach (NodeBlock* block, m_NodeBlockManager->m_NodeBlockList)
          {
             if (block->m_NodeItem->m_NodeID == nodeOut)
                 portOut = block->m_NodeItem->portAt(portOutNum);

             if (block->m_NodeItem->m_NodeID == nodeIn)
                 portIn = block->m_NodeItem->portAt(portInNum);
          }
          if (portOut && portIn)
             m_NodeView->createConnection(portOut, portIn);
     }   
     ///
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

    /// Node Block
    ///

    int i = 0;
    foreach (NodeBlock* block, m_NodeBlockManager->m_NodeBlockList)
    {
        QJsonDocument nodeDoc;
        QJsonArray nodeData;
        QJsonObject nodeObj;
        QJsonObject parametersObj;

        ++i;
        int id = block->m_NodeItem->m_NodeID;
        QString name = block->m_NodeItem->m_NodeName;
        QString title = block->m_NodeItem->m_title;
        qreal x = block->m_NodeItem->x();
        qreal y = block->m_NodeItem->y();

        nodeObj["id"] = QString::number(id);
        nodeObj["name"] = name;
        nodeObj["title"] = title;
        nodeObj["pos x"] = QString::number(x, 'f', 3);
        nodeObj["pos y"] = QString::number(y, 'f', 3);

        QList<QString> keyList = block->m_NodeItem->m_Parameters.keys();
        for (int j = 0; j < keyList.size(); ++j)
        {
            QString key = keyList[j];
            QVariant value = block->m_NodeItem->m_Parameters.value(key);
            parametersObj[key] = value.toString();
        }
        nodeObj.insert("arguments", parametersObj);

        QString Index = QString::number(i);
        Obj.insert("node block " + Index, nodeObj);
    }

    /// Rope Connection
    ///

    int j = 0;
    for (RopeItem *rope : m_NodeView->m_ropeList)
    {
        ++j;
        PortItem *portOut = rope->portOut();
        PortItem *portIn = rope->portIn();

        int ropeOut = ParentNodeID(portOut);
        int ropeIn = ParentNodeID(portIn);

        QJsonObject ropeObj;
        ropeObj["color"] = rope->color().name();
        ropeObj["node block id out"] = QString::number(ropeOut);
        ropeObj["node block id in"] = QString::number(ropeIn);
        ropeObj["port number out"] = QString::number(portOut->number());
        ropeObj["port number in"] = QString::number(portIn->number());

        QString Index = QString::number(j);
        Obj.insert("rope connection " + Index, ropeObj);
    }

    /// Infomation
    ///
    
    QJsonObject infoObj;
    QDateTime CurrentDataTime = QDateTime::currentDateTime();
    infoObj["time"] = CurrentDataTime.toString("yyyy-MM-dd hh:mm:ss");
    infoObj["node number"] = QString::number(i);
    infoObj["rope number"] = QString::number(j);
    Obj.insert("node infomation", infoObj);

    Doc.setObject(Obj);

    JsonFile.write(Doc.toJson());
    JsonFile.close();
}

void CGNodeView::OnRemoveNodeBlock(int nodeId)
{
    foreach (NodeBlock* block, m_NodeBlockManager->m_NodeBlockList)
    {
        if (block->m_NodeItem->m_NodeID == nodeId)
        {
            m_NodeBlockManager->m_NodeBlockList.removeOne(block);
            delete block;
            block = nullptr;

            if (nodeId > 0)
                m_NodeView->m_IDCounter--;
            if (nodeId < 0)
                m_NodeView->m_IDCounterMinus++;
        }
    }
}

void CGNodeView::OnLoadAlgorithmArguments(bool b, int nodeId)
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

void CGNodeView::OnLoadLocalDataFile(bool b, int nodeId)
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

void CGNodeView::OnLoadScriptCpp(bool b, int nodeId)
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
    m_CGScriptCppEditor->SetCurrentNodeBlock(nodeBlock);

    if (nodeName == NULL) {
        return;}

    QVariant var = nodeBlock->m_NodeItem->m_Parameters.value(u8"代码");
    QString str = var.toString();

    if (str.isEmpty())
        m_CGScriptCppEditor->m_pTextEdit->setText("//*******************\n// Script JS Code  //\n//*******************\n\nfunction ScriptMain(input, output, func)\n{\n    str = \"hello script!\";\n\n    return str;\n}");
    else
        m_CGScriptCppEditor->m_pTextEdit->setText(str);
}

void CGNodeView::OnReLoadAlgorithmArguments(int nodeId)
{
    QString nodeName = NULL;
    NodeBlock* nodeBlock;
    foreach (NodeBlock* block, m_NodeBlockManager->m_NodeBlockList)
    {
        if (block->m_NodeItem->m_NodeID == nodeId)
        {
            nodeName = block->m_NodeItem->m_NodeName;
            nodeBlock = block;
            NodeBlockWidget::getInstance()->SetCurrentNodeBlock(block);
            break;
        }
    }
    if (nodeName == NULL) return;

    QVector<CG_ARGUMENT> arguments;
    if (m_PluginManager->m_PluginNames2D.contains(nodeName))
    {
        AlgorithmInterface *plugin = m_PluginNodeBlockList.value(nodeId);
        NodeBlockWidget::getInstance()->SetCurrentAlgorithmPlugin(plugin);

        arguments = plugin->GetAlgorithmArguments();
        for (CG_ARGUMENT &parameter : arguments)
        {
            QString key = parameter.ARG;
            float value = nodeBlock->m_NodeItem->m_Parameters.value(key).toFloat();
            parameter.VALUE = value;
        }
        if (arguments.count() > 0)
            plugin->SetAlgorithmArguments(arguments);
    }
    if (m_PluginManager->m_PluginNames3D.contains(nodeName))
    {
        AlgorithmInterface *plugin = m_PluginNodeBlockList.value(nodeId);
        NodeBlockWidget::getInstance()->SetCurrentAlgorithmPlugin(plugin);
        
        arguments = plugin->GetAlgorithmArguments();
        for (CG_ARGUMENT &parameter : arguments)
        {
            QString key = parameter.ARG;
            float value = nodeBlock->m_NodeItem->m_Parameters.value(key).toFloat();
            parameter.VALUE = value;
        }
        if (arguments.count() > 0)
            plugin->SetAlgorithmArguments(arguments);
    }

    if (arguments.empty()) return;
    NodeBlockWidget::getInstance()->LoadAlgorithmArguments(arguments);
}
