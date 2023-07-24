#ifndef NODEVIEW_H
#define NODEVIEW_H

#include <QGraphicsView>
#include "NodeItem.h"
#include "RopeItem.h"
#include "GroupItem.h"

class NodeView : public QGraphicsView
{
    Q_OBJECT

public:
    explicit NodeView(QWidget *parent = Q_NULLPTR);
    ~NodeView();

private:

    QGraphicsScene *m_scene;
    QPointF m_scenePos;
    QPointF m_pressPos;
    bool m_moveScene;
    qreal m_currentScale;

public:
    RopeItem *m_activeRope;
    NodeItem *m_activeNode;
    QList<NodeItem*> m_nodeList;
    QList<RopeItem*> m_ropeList;
    QList<GroupItem*> m_groupList;
    QList<NodeItem*> m_selectedNodes;

private:
    bool m_isCheckingColor;
    bool m_isOnlyOneInputConnection;
    qreal m_ropeFlexion;
    bool m_isConnectionDragable;

    void wheelEvent(QWheelEvent *event);
    void drawBackground(QPainter *painter, const QRectF &r);
    void drawGrid(QPainter *painter, double gridStep);

    void mousePressEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void mouseDoubleClickEvent(QMouseEvent *e);
    void keyPressEvent(QKeyEvent *e);

    void checkRopeCreation(const QPointF &point);
    void checkRopeConnection(const QPointF &point);
    void checkNodesSelected();

    NodeItem *checkNodeHit(const QPointF &point);
    PortItem *checkPortHit(const QPointF &point);

    void createRopeHitPort(PortItem *port);
    void createConnectionToPort(PortItem *port);

    void removeActiveRope();

    RopeItem *getRopeWithPort(PortItem *port);

signals:
    void selectionsChanged();
    void calledMenuNode(NodeItem *node);
    void calledMenuView();
    void signalRemoveNode(unsigned int nodeId);
    void signalDoubleClick(bool b, unsigned int nodeID, QString name);

public slots:
    void addNode(NodeItem *node);
    NodeItem *createNode(QWidget *widget);
    void removeNode(NodeItem *node);
    NodeItem *nodeAt(const QUuid &uuid);

    void addGroup(GroupItem *group);
    GroupItem *createGroup(QList<NodeItem*> list);
    GroupItem *getItemGroup(NodeItem* node);
    void removeGroup(GroupItem *group);

    bool createConnection(PortItem *portOut, PortItem *portIn);
    void removePortConnections(PortItem *port);

    void clearView();

    QList<NodeItem*> nodeList(){return m_nodeList;}
    QList<RopeItem*> ropeList(){return m_ropeList;}
    QList<GroupItem*> groupList(){return m_groupList;}
    QList<NodeItem*> selectedNodeList(){return m_selectedNodes;}

    bool isPortFree(PortItem *port);

    void setCheckingColor(bool state = true);
    void setOnlyOneInputConnection(bool state = true);
    void setRopeFlexion(qreal value = 100.0);
    void setConnectionDragable(bool state = true);

public:
    unsigned int m_IDCounter;


//Deprecated

public:
    NodeItem *NodeItemFactory(QString nodename, int in, int out);
    NodeItem *NodeItemNumberInput(QString nodename);
    NodeItem *NodeItemNumberOutput(QString nodename);

private:
    NodeItem *CreatNodeItem01(const QString nodename);
    NodeItem *CreatNodeItem10(const QString nodename);
    NodeItem *CreatNodeItem11(const QString nodename);
    NodeItem *CreatNodeItem21(const QString nodename);
    NodeItem *CreatNodeItem31(const QString nodename);
    NodeItem *CreatNodeItem12(const QString nodename);

    int RandPos();

};

#endif // NODEVIEW_H
