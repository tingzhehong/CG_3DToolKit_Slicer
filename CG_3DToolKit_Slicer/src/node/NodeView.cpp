﻿#include "NodeView.h"
#include <QWheelEvent>
#include <QGraphicsScene>
#include <QPainter>
#include <QDebug>
#include <QScrollBar>
#include <QtMath>
#include <QGraphicsProxyWidget>
#include <QLabel>
#include <QLineEdit>

NodeView::NodeView(QWidget *parent) : QGraphicsView(parent),
    m_scene(new QGraphicsScene),
    m_scenePos(QPointF(0,0)),
    m_pressPos(QPointF(0,0)),
    m_moveScene(false),
    m_currentScale(1.0),
    m_activeRope(0),
    m_isCheckingColor(true),
    m_isOnlyOneInputConnection(true),
    m_ropeFlexion(100.0),
    m_isConnectionDragable(true),
    m_IDCounter(1),
    m_IDCounterMinus(-1)
{
    setRenderHint(QPainter::Antialiasing);
    setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    setBackgroundBrush(QBrush(QColor::fromRgb(80,80,80)));

    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setCacheMode(QGraphicsView::CacheBackground);

    m_scene->setSceneRect(m_scene->itemsBoundingRect());
    setScene(m_scene);
}

NodeView::~NodeView()
{
    delete m_scene;
}

void NodeView::wheelEvent(QWheelEvent *e)
{
    qreal scaleFactor = 1.15;

    if (e->delta() > 0)
    {
        if(m_currentScale < scaleFactor)
        {
            scale(scaleFactor,scaleFactor);
            m_currentScale *= scaleFactor;
        }
    }
    else if(m_currentScale > 0.1)
    {
        scale(1/scaleFactor, 1/scaleFactor);
        m_currentScale /= scaleFactor;
    }

    update();
}

void NodeView::drawBackground(QPainter *painter, const QRectF &r)
{
    QGraphicsView::drawBackground(painter, r);

    QPen pfine(QColor::fromRgb(50,50,50), 0.6);

    painter->setPen(pfine);
    drawGrid(painter,15);

    QPen p(QColor::fromRgb(50,50,50), 2.0);

    painter->setPen(p);
    drawGrid(painter,150);
}

void NodeView::drawGrid(QPainter *painter, double gridStep)
{
    QRect   windowRect = rect();
    QPointF tl = mapToScene(windowRect.topLeft());
    QPointF br = mapToScene(windowRect.bottomRight());

    double left   = qFloor(tl.x() / gridStep - 0.5);
    double right  = qFloor(br.x() / gridStep + 1.0);
    double bottom = qFloor(tl.y() / gridStep - 0.5);
    double top    = qFloor(br.y() / gridStep + 1.0);

    for (int xi = int(left); xi <= int(right); ++xi)
    {
      QLineF line(xi * gridStep, bottom * gridStep,
                  xi * gridStep, top * gridStep );

      painter->drawLine(line);
    }

    for (int yi = int(bottom); yi <= int(top); ++yi)
    {
      QLineF line(left * gridStep, yi * gridStep,
                  right * gridStep, yi * gridStep );
      painter->drawLine(line);
    }
}

void NodeView::mousePressEvent(QMouseEvent *e)
{
    QMouseEvent fake(e->type(), e->pos(), Qt::LeftButton, Qt::LeftButton, e->modifiers());

    m_scenePos = mapToScene(e->pos());
    m_pressPos = m_scenePos;

    if (e->button() == Qt::MiddleButton)
    {
        setDragMode(QGraphicsView::ScrollHandDrag);
        setInteractive(false);

        e = &fake;

        m_moveScene = true;
    }
    else if (e->button() == Qt::LeftButton)
    {
        setDragMode(QGraphicsView::RubberBandDrag);

        if (m_isConnectionDragable)
            checkRopeCreation(mapToScene(e->pos()));
    }
    else if (e->button() == Qt::RightButton)
    {
        NodeItem *node = checkNodeHit(mapToScene(e->pos()));

        if (node)
            emit calledMenuNode(node);
        else emit calledMenuView();
    }

    update();
    QGraphicsView::mousePressEvent(e);
}

void NodeView::mouseMoveEvent(QMouseEvent *e)
{
    m_scenePos = mapToScene(e->pos());
    PortItem *hitPort = checkPortHit(m_scenePos);

    if (m_activeRope)
    {
        if (hitPort)
            m_activeRope->setMousePoint(hitPort->portPos());
        else m_activeRope->setMousePoint(m_scenePos);
    }

    if (m_moveScene)
    {
        QPointF difference = m_pressPos - m_scenePos;
        setSceneRect(sceneRect().translated(difference.x(), difference.y()));
    }

    update();
    QGraphicsView::mouseMoveEvent(e);
}

void NodeView::mouseReleaseEvent(QMouseEvent *e)
{
    QMouseEvent fake(e->type(), e->pos(), Qt::LeftButton, Qt::LeftButton, e->modifiers());

    if (e->button() == Qt::LeftButton)
    {
        checkNodesSelected();
        checkRopeConnection(mapToScene(e->pos()));
    }
    else if (e->button() == Qt::MiddleButton)
    {
        setDragMode(QGraphicsView::NoDrag);
        setInteractive(true);

        e = &fake;
    }

    m_moveScene = false;
    update();
    QGraphicsView::mouseReleaseEvent(e);
}

void NodeView::mouseDoubleClickEvent(QMouseEvent *e)
{
    m_scenePos = mapToScene(e->pos());
    NodeItem *hitNode = checkNodeHit(m_scenePos);

    if (hitNode)
    {
        qDebug() << hitNode->NodeName();
        QString name = hitNode->NodeName();
        bool isAlgorithmPlugin = hitNode->m_AlgorithmNode;
        int id = hitNode->m_NodeID;
        emit signalDoubleClick(isAlgorithmPlugin, id, name);
    }

    update();
    QGraphicsView::mouseDoubleClickEvent(e);
}

void NodeView::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Delete)
    {
        if (m_activeNode && m_selectedNodes.size() == 1)
        {
            removeNode(m_activeNode);
        }
        if (m_selectedNodes.size() > 1)
        {
            foreach (NodeItem *node, m_selectedNodes)
            {
                removeNode(node);
            }
        }
    }

    update();
    QGraphicsView::keyPressEvent(e);
}

void NodeView::checkRopeCreation(const QPointF &point)
{
    NodeItem *hitNode = checkNodeHit(point);

    if (!hitNode)
        return;

    PortItem *hitPort = hitNode->isHoveredPort(point);

    if (!hitPort)
        return;

    createRopeHitPort(hitPort);
}

void NodeView::checkRopeConnection(const QPointF &point)
{
    if (!m_activeRope)
        return;

    NodeItem *hitNode = checkNodeHit(point);

    if (!hitNode)
    {
        removeActiveRope();
        return;
    }

    PortItem *port = hitNode->isHoveredPort(point);

    if (port)
        createConnectionToPort(port);
    else removeActiveRope();
}

void NodeView::checkNodesSelected()
{
    QList<NodeItem*> selectedNodeList;

    foreach (NodeItem *node, m_nodeList)
        if (node->isSelected())
            selectedNodeList.append(node);

    if (m_selectedNodes != selectedNodeList)
    {
        m_selectedNodes = selectedNodeList;
        emit selectionsChanged();
    }
}

NodeItem *NodeView::checkNodeHit(const QPointF &point)
{
    NodeItem *hitNode = Q_NULLPTR;

    foreach (NodeItem *node, m_nodeList)
        if (node->isHovered(point))
        {
            hitNode = node;
            m_activeNode = hitNode;
            break;
        }

    return hitNode;
}

PortItem *NodeView::checkPortHit(const QPointF &point)
{
    PortItem *hitPort = Q_NULLPTR;

    NodeItem *hitNode = checkNodeHit(point);

    if (hitNode)
        hitPort = hitNode->isHoveredPort(point);

    return hitPort;
}

void NodeView::createRopeHitPort(PortItem *port)
{
    if (!port)
        return;

    setDragMode(QGraphicsView::NoDrag);

    if (port->portType() == PortItem::TypeIn)
    {
        if (isPortFree(port))
        {
            m_activeRope = new RopeItem(0,port);
            m_activeRope->setFlexion(m_ropeFlexion);
            m_scene->addItem(m_activeRope);
            m_ropeList.append(m_activeRope);
        }
        else
        {
            m_activeRope = getRopeWithPort(port);
            m_activeRope->removePortIn();
            m_activeRope->setMousePoint(m_scenePos);
        }
    }
    else
    {
        m_activeRope = new RopeItem(port,0);
        m_activeRope->setFlexion(m_ropeFlexion);
        m_scene->addItem(m_activeRope);
        m_ropeList.append(m_activeRope);
    }
}

void NodeView::createConnectionToPort(PortItem *port)
{
    if (!m_activeRope)
        return;

    bool colorPermission = false;
    bool severalConnectionPermission = false;

    if (m_isCheckingColor && m_activeRope->color() == port->color())
        colorPermission = true;
    else if (!m_isCheckingColor)
        colorPermission = true;

    if (m_isOnlyOneInputConnection && isPortFree(port))
        severalConnectionPermission = true;
    else if (!m_isOnlyOneInputConnection)
        severalConnectionPermission = true;

    if (m_activeRope->portIn() &&
       !m_activeRope->portOut() &&
       port->portType() == PortItem::TypeOut &&
       colorPermission)
    {
        m_activeRope->setPortOut(port);
    }

    if (m_activeRope->portOut() &&
       !m_activeRope->portIn() &&
       port->portType() == PortItem::TypeIn &&
       colorPermission &&
       severalConnectionPermission)
    {
        m_activeRope->setPortIn(port);
    }

    if (m_activeRope->isConnected())
        m_activeRope = Q_NULLPTR;
    else removeActiveRope();
}

void NodeView::removeActiveRope()
{
    if (!m_activeRope)
        return;

    m_activeRope->removePortIn();
    m_activeRope->removePortOut();

    m_ropeList.removeOne(m_activeRope);
    m_activeRope->disconnect();
    m_activeRope->deleteLater();
    m_activeRope = Q_NULLPTR;
}

RopeItem *NodeView::getRopeWithPort(PortItem *port)
{
    RopeItem *result = Q_NULLPTR;

    foreach (RopeItem *rope, m_ropeList)
    {
        if (rope->portIn() == port)
        {
            result = rope;
            break;
        }
        else if (rope->portOut() == port)
        {
            result = rope;
            break;
        }
    }

    return result;
}

void NodeView::addNode(NodeItem *node)
{
    node->setPos(m_scenePos);
    m_scene->addItem(node);
    m_nodeList.append(node);
}

NodeItem *NodeView::createNode(QWidget *widget)
{
    NodeItem *node = new NodeItem(widget);
    addNode(node);

    return node;
}

void NodeView::removeNode(NodeItem *node)
{
    foreach (PortItem *port, node->portList())
    {
        QList<RopeItem*> repesWithNode;

        foreach (RopeItem *rope, m_ropeList)
        {
            if (rope->portIn() == port)
                repesWithNode.append(rope);
            else if (rope->portOut() == port)
                repesWithNode.append(rope);
        }

        foreach (RopeItem *rope, repesWithNode)
        {
            rope->removePortIn();
            rope->removePortOut();

            m_ropeList.removeOne(rope);
            rope->disconnect();
            rope->deleteLater();
        }

        port->disconnect();
        port->deleteLater();
    }

    foreach(GroupItem *group, m_groupList)
        group->removeNode(node);

    m_nodeList.removeOne(node);
    m_selectedNodes.removeOne(node);
    node->disconnect();
    node->deleteLater();

    emit signalRemoveNode(node->m_NodeID);
}

NodeItem *NodeView::nodeAt(const QUuid &uuid)
{
    NodeItem *result = Q_NULLPTR;

    foreach (NodeItem *node, m_nodeList)
    {
        if (node->uuid() == uuid)
        {
            result = node;
            break;
        }
    }

    return result;
}

void NodeView::addGroup(GroupItem *group)
{
    group->setPos(m_scenePos);
    m_scene->addItem(group);
    m_groupList.append(group);
    connect(group, &GroupItem::becameEmpty, this, &NodeView::removeGroup);
}

GroupItem *NodeView::createGroup(QList<NodeItem *> list)
{
    GroupItem *group = new GroupItem;
    group->setNodes(list);
    addGroup(group);

    return group;
}

GroupItem *NodeView::getItemGroup(NodeItem *node)
{
    GroupItem *result = Q_NULLPTR;

    foreach (GroupItem *group, m_groupList)
    {
        if(group->nodeList().contains(node))
        {
            result = group;
            break;
        }
    }

    return result;
}

void NodeView::removeGroup(GroupItem *group)
{
    if (!m_groupList.contains(group)) return;

    m_groupList.removeOne(group);

    group->disconnect();
    group->deleteLater();
}

bool NodeView::createConnection(PortItem *portOut, PortItem *portIn)
{
    bool result = false;

    if (isPortFree(portIn))
    {
        RopeItem *rope = new RopeItem(portOut,portIn);
        rope->setFlexion(m_ropeFlexion);
        m_scene->addItem(rope);
        m_ropeList.append(rope);
        result = true;
    }

    return result;
}

void NodeView::removePortConnections(PortItem *port)
{
    QList<RopeItem*> ropesToRemove;
    foreach (RopeItem *rope, m_ropeList)
        if (rope->portIn() == port || rope->portOut() == port)
            ropesToRemove.append(rope);

    foreach (RopeItem *rope, ropesToRemove)
    {
        m_ropeList.removeOne(rope);
        rope->disconnect();
        rope->deleteLater();
    }
}

void NodeView::clearView()
{
    foreach (NodeItem *node, m_nodeList)
    {
        if(node->widget())
            node->widget()->deleteLater();

        removeNode(node);
    }

    foreach (GroupItem *group, m_groupList)
    {
        group->disconnect();
        group->deleteLater();
    }

    m_groupList.clear();
    m_nodeList.clear();
    m_ropeList.clear();
    m_selectedNodes.clear();
    m_scene->clear();
}

bool NodeView::isPortFree(PortItem *port)
{
    bool result = true;
    foreach (RopeItem *rope, m_ropeList)
    {
        if (rope->portIn() == port || rope->portOut() == port)
        {
            result = false;
            break;
        }
    }
    return result;
}

void NodeView::setCheckingColor(bool state)
{
    m_isCheckingColor = state;
    update();
}

void NodeView::setOnlyOneInputConnection(bool state)
{
    m_isOnlyOneInputConnection = state;
    update();
}

void NodeView::setRopeFlexion(qreal value)
{
    m_ropeFlexion = value;
    update();
}

void NodeView::setConnectionDragable(bool state)
{
    m_isConnectionDragable = state;
}

NodeItem *NodeView::NodeItemFactory(QString nodename, int in, int out)
{
    NodeItem *node;

    int index = in * 10 + out;

    switch (index)
    {
    case 01:
        node = CreatNodeItem01(nodename);
        break;
    case 10:
        node = CreatNodeItem10(nodename);
        break;
    case 11:
        node = CreatNodeItem11(nodename);
        ++m_IDCounter;
        break;
    case 21:
        node = CreatNodeItem21(nodename);
        ++m_IDCounter;
        break;
    case 31:
        node = CreatNodeItem31(nodename);
        ++m_IDCounter;
        break;
    case 12:
        node = CreatNodeItem12(nodename);
        ++m_IDCounter;
        break;
    default:
        break;
    }

    return node;
}

NodeItem *NodeView::NodeItemNumberInput(QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *label = new QLabel(tr(u8"数值:"), widget);
    label->resize(60, 20);
    label->move(2, 15);

    QLineEdit *lineEdit = new QLineEdit("", widget);
    lineEdit->resize(105, 20);
    lineEdit->move(40, 15);

    NodeItem *node = this->createNode(widget);
    node->setTitle(nodename);
    node->setNodeName(nodename);
    node->setNodeID(0);

    int x = RandPos();
    int y = RandPos();
    node->setPos(x, y);

    PortItem *portOut = node->createPortOut(8, QColor(Qt::cyan));
    connect(lineEdit, &QLineEdit::textChanged, this, [=](QString str){ portOut->setValue(QVariant::fromValue(str.trimmed().toFloat())); });

    return node;
}

NodeItem *NodeView::NodeItemNumberOutput(QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *label = new QLabel(tr(u8"数值:"), widget);
    label->resize(60, 20);
    label->move(2, 15);

    QLineEdit *lineEdit = new QLineEdit("", widget);
    lineEdit->resize(105, 20);
    lineEdit->move(40, 15);
    lineEdit->setReadOnly(true);

    NodeItem *node = this->createNode(widget);
    node->setTitle(nodename);
    node->setNodeName(nodename);
    node->setNodeID(0);

    int x = RandPos();
    int y = RandPos();
    node->setPos(x, y);

    PortItem *portIn = node->createPortIn(8, QColor(Qt::cyan));
    connect(portIn, &PortItem::valueChanged, this, [=](QVariant value){ lineEdit->setText(QString::asprintf("%.6f", value.toFloat())); });

    return node;
}

NodeItem *NodeView::CreatNodeItem01(const QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *label = new QLabel(tr(u8"算子: ") + nodename, widget);
    label->resize(60, 20);
    label->move(56, 15);

    QLabel *labelId = new QLabel("ID: " + QString::number(m_IDCounter), widget);
    labelId->resize(60, 20);
    labelId->move(2, 15);

    NodeItem *node = this->createNode(widget);
    node->setTitle(nodename);
    node->setNodeName(nodename);
    node->setNodeID(m_IDCounter);

    int x = RandPos();
    int y = RandPos();
    node->setPos(x, y);

    node->createPortOut(8, QColor(Qt::cyan));

    return node;
}

NodeItem *NodeView::CreatNodeItem10(const QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *label = new QLabel(tr(u8"算子: ") + nodename, widget);
    label->resize(60, 20);
    label->move(56, 15);

    QLabel *labelId = new QLabel("ID: " + QString::number(m_IDCounter), widget);
    labelId->resize(60, 20);
    labelId->move(2, 15);

    NodeItem *node = this->createNode(widget);
    node->setTitle(nodename);
    node->setNodeName(nodename);
    node->setNodeID(m_IDCounter);

    int x = RandPos();
    int y = RandPos();
    node->setPos(x, y);

    node->createPortIn(8, QColor(Qt::cyan));

    return node;
}

NodeItem *NodeView::CreatNodeItem11(const QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *label = new QLabel(tr(u8"算子: ") + nodename, widget);
    label->resize(60, 20);
    label->move(56, 15);

    QLabel *labelId = new QLabel("ID: " + QString::number(m_IDCounter), widget);
    labelId->resize(60, 20);
    labelId->move(2, 15);

    NodeItem *node = this->createNode(widget);
    node->setTitle(nodename);
    node->setNodeName(nodename);
    node->setNodeID(m_IDCounter);

    int x = RandPos();
    int y = RandPos();
    node->setPos(x, y);

    node->createPortIn(8, QColor(Qt::cyan));
    node->createPortOut(8, QColor(Qt::cyan));

    return node;
}

NodeItem *NodeView::CreatNodeItem21(const QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *label = new QLabel(tr(u8"算子: ") + nodename, widget);
    label->resize(60, 20);
    label->move(56, 15);

    QLabel *labelId = new QLabel("ID: " + QString::number(m_IDCounter), widget);
    labelId->resize(60, 20);
    labelId->move(2, 15);

    NodeItem *node = this->createNode(widget);
    node->setTitle(nodename);
    node->setNodeName(nodename);
    node->setNodeID(m_IDCounter);

    int x = RandPos();
    int y = RandPos();
    node->setPos(x, y);

    node->createPortIn(8, QColor(Qt::cyan));
    node->createPortIn(24, QColor(Qt::cyan));
    node->createPortOut(8, QColor(Qt::cyan));

    return node;
}

NodeItem *NodeView::CreatNodeItem31(const QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *label = new QLabel(tr(u8"算子: ") + nodename, widget);
    label->resize(60, 20);
    label->move(56, 15);

    QLabel *labelId = new QLabel("ID: " + QString::number(m_IDCounter), widget);
    labelId->resize(60, 20);
    labelId->move(2, 15);

    NodeItem *node = this->createNode(widget);
    node->setTitle(nodename);
    node->setNodeName(nodename);
    node->setNodeID(m_IDCounter);

    int x = RandPos();
    int y = RandPos();
    node->setPos(x, y);

    node->createPortIn(8, QColor(Qt::cyan));
    node->createPortIn(24, QColor(Qt::cyan));
    node->createPortIn(40, QColor(Qt::cyan));
    node->createPortOut(8, QColor(Qt::cyan));

    return node;
}

NodeItem *NodeView::CreatNodeItem12(const QString nodename)
{
    QWidget *widget = new QWidget;
    widget->resize(150, 50);

    QLabel *label = new QLabel(tr(u8"算子: ") + nodename, widget);
    label->resize(60, 20);
    label->move(56, 15);

    QLabel *labelId = new QLabel("ID: " + QString::number(m_IDCounter), widget);
    labelId->resize(60, 20);
    labelId->move(2, 15);

    NodeItem *node = this->createNode(widget);
    node->setTitle(nodename);
    node->setNodeName(nodename);
    node->setNodeID(m_IDCounter);

    int x = RandPos();
    int y = RandPos();
    node->setPos(x, y);

    node->createPortIn(8, QColor(Qt::cyan));
    node->createPortOut(8, QColor(Qt::cyan));
    node->createPortOut(24, QColor(Qt::cyan));

    return node;
}

int NodeView::RandPos()
{
    return rand() % 100;
}
