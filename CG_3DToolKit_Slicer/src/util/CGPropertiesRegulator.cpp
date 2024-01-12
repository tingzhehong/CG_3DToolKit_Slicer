#include "CGPropertiesRegulator.h"
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QDebug>
#include <CGPropertiesForm1.h>
#include <CGPropertiesForm2.h>
#include <CGVTKHeader.h>
#include <vtkPlane.h>
#include <vtkPlanes.h>


CGPropertiesRegulator *CGPropertiesRegulator::m_CGPropertiesRegulator = nullptr;

CGPropertiesRegulator::CGPropertiesRegulator(QObject *parent) : QObject(parent)
{

}

CGPropertiesRegulator *CGPropertiesRegulator::getInstance()
{
    if (!m_CGPropertiesRegulator)
    {
        m_CGPropertiesRegulator = new CGPropertiesRegulator();
    }
    return m_CGPropertiesRegulator;
}

void CGPropertiesRegulator::SetCGPropertiesForms(CGPropertiesForm1 *&form1, CGPropertiesForm2 *&form2)
{
    m_Form1 = form1;
    m_Form2 = form2;
}

void CGPropertiesRegulator::ShowPropertiesLine(QLineF line)
{
    m_Form1->m_PropertiesTree->clear();

    QTreeWidgetItem *pToolItem = new QTreeWidgetItem(QStringList{tr(u8"工具属性")});
    QStringList propertiesList;
    qreal x1 = line.x1();   QString X1 = QString::number(x1);
    qreal y1 = line.y1();   QString Y1 = QString::number(y1);
    qreal x2 = line.x2();   QString X2 = QString::number(x2);
    qreal y2 = line.y2();   QString Y2 = QString::number(y2);
    propertiesList.append(tr(u8"LINE: "));
    propertiesList.append("X1 = " + X1 + "  Y1 = " + Y1);
    propertiesList.append("X2 = " + X2 + "  Y2 = " + Y2);

    for (int i = 0; i < propertiesList.size(); ++i) {
        QTreeWidgetItem *pItem = new QTreeWidgetItem(QStringList() << propertiesList.at(i));
        pItem->setFlags(pItem->flags() | Qt::ItemFlag::ItemIsEditable);
        pToolItem->addChild(pItem);
    }
    CGPropertiesRegulator::getInstance()->m_Form1->m_PropertiesTree->addTopLevelItem(pToolItem);
    CGPropertiesRegulator::getInstance()->m_Form1->m_PropertiesTree->expandAll();
}

void CGPropertiesRegulator::ShowPropertiesRect(QRectF rect)
{
    m_Form1->m_PropertiesTree->clear();

    QTreeWidgetItem *pToolItem = new QTreeWidgetItem(QStringList{tr(u8"工具属性")});
    QStringList propertiesList;
    qreal x = rect.x();        QString X = QString::number(x);
    qreal y = rect.y();        QString Y = QString::number(y);
    qreal w = rect.width();    QString W = QString::number(w);
    qreal h = rect.height();   QString H = QString::number(h);
    propertiesList.append(tr(u8"RECT: "));
    propertiesList.append("X = " + X);
    propertiesList.append("Y = " + Y);
    propertiesList.append("Width = " + W);
    propertiesList.append("Height = " + H);

    for (int i = 0; i < propertiesList.size(); ++i) {
        QTreeWidgetItem *pItem = new QTreeWidgetItem(QStringList() << propertiesList.at(i));
        pItem->setFlags(pItem->flags() | Qt::ItemFlag::ItemIsEditable);
        pToolItem->addChild(pItem);
    }
    CGPropertiesRegulator::getInstance()->m_Form1->m_PropertiesTree->addTopLevelItem(pToolItem);
    CGPropertiesRegulator::getInstance()->m_Form1->m_PropertiesTree->expandAll();
}

void CGPropertiesRegulator::ShowPropertiesAngle(qreal angle)
{
    m_Form1->m_PropertiesTree->clear();

    QTreeWidgetItem *pToolItem = new QTreeWidgetItem(QStringList{tr(u8"工具属性")});
    QStringList propertiesList;
    QString Angle = QString::number(angle);
    propertiesList.append(tr(u8"ANGLE: "));
    propertiesList.append("Angle = " + Angle);

    for (int i = 0; i < propertiesList.size(); ++i) {
        QTreeWidgetItem *pItem = new QTreeWidgetItem(QStringList() << propertiesList.at(i));
        pItem->setFlags(pItem->flags() | Qt::ItemFlag::ItemIsEditable);
        pToolItem->addChild(pItem);
    }
    CGPropertiesRegulator::getInstance()->m_Form1->m_PropertiesTree->addTopLevelItem(pToolItem);
    CGPropertiesRegulator::getInstance()->m_Form1->m_PropertiesTree->expandAll();
}

void CGPropertiesRegulator::ShowPropertiesVTKSphere(double *sphere)
{
    m_Form1->m_PropertiesTree->clear();

    QTreeWidgetItem *pToolItem = new QTreeWidgetItem(QStringList{tr(u8"工具属性")});
    QStringList propertiesList;
    QString X = QString::number(sphere[0]);
    QString Y = QString::number(sphere[1]);
    QString Z = QString::number(sphere[2]);
    QString R = QString::number(sphere[3]);
    propertiesList.append(tr(u8"SPHERE "));
    propertiesList.append("X = " + X);
    propertiesList.append("Y = " + Y);
    propertiesList.append("Z = " + Z);
    propertiesList.append("Radius = " + R);

    for (int i = 0; i < propertiesList.size(); ++i) {
        QTreeWidgetItem *pItem = new QTreeWidgetItem(QStringList() << propertiesList.at(i));
        pItem->setFlags(pItem->flags() | Qt::ItemFlag::ItemIsEditable);
        pToolItem->addChild(pItem);
    }
    CGPropertiesRegulator::getInstance()->m_Form1->m_PropertiesTree->addTopLevelItem(pToolItem);
    CGPropertiesRegulator::getInstance()->m_Form1->m_PropertiesTree->expandAll();
}

void CGPropertiesRegulator::ShowPropertiesVTKPlane(vtkPlane *plane)
{
    m_Form1->m_PropertiesTree->clear();

    QTreeWidgetItem *pToolItem = new QTreeWidgetItem(QStringList{tr(u8"工具属性")});
    QStringList propertiesList;
    double normal[3];
    double origin[3];
    plane->GetNormal(normal);
    plane->GetOrigin(origin);
    QString A = QString::number(normal[0]);
    QString B = QString::number(normal[1]);
    QString C = QString::number(normal[2]);
    QString X = QString::number(origin[0]);
    QString Y = QString::number(origin[1]);
    QString Z = QString::number(origin[2]);
    propertiesList.append(tr(u8"PLANE: "));
    propertiesList.append(tr(u8"normal: "));
    propertiesList.append("A = " + A);
    propertiesList.append("B = " + B);
    propertiesList.append("C = " + C);
    propertiesList.append(tr(u8"origin: "));
    propertiesList.append("X = " + X);
    propertiesList.append("Y = " + Y);
    propertiesList.append("Z = " + Z);

    for (int i = 0; i < propertiesList.size(); ++i) {
        QTreeWidgetItem *pItem = new QTreeWidgetItem(QStringList() << propertiesList.at(i));
        pItem->setFlags(pItem->flags() | Qt::ItemFlag::ItemIsEditable);
        pToolItem->addChild(pItem);
    }
    CGPropertiesRegulator::getInstance()->m_Form1->m_PropertiesTree->addTopLevelItem(pToolItem);
    CGPropertiesRegulator::getInstance()->m_Form1->m_PropertiesTree->expandAll();
}

void CGPropertiesRegulator::ShowPropertiesVTKPlanes(vtkPlanes *planes)
{
    m_Form1->m_PropertiesTree->clear();

    QTreeWidgetItem *pToolItem = new QTreeWidgetItem(QStringList{tr(u8"工具属性")});
    QStringList propertiesList;
    double bounds[6];
    planes->GetPoints()->GetBounds(bounds);
    QString Xmin = QString::number(bounds[0]);
    QString Xmax = QString::number(bounds[1]);
    QString Ymin = QString::number(bounds[2]);
    QString Ymax = QString::number(bounds[3]);
    QString Zmin = QString::number(bounds[4]);
    QString Zmax = QString::number(bounds[5]);
    propertiesList.append(tr(u8"BOUNDS: "));
    propertiesList.append("Xmin  = " + Xmin);
    propertiesList.append("Xmax = " + Xmax);
    propertiesList.append("Ymin  = " + Ymin);
    propertiesList.append("Ymax = " + Ymax);
    propertiesList.append("Zmin  = " + Zmin);
    propertiesList.append("Zmax = " + Zmax);

    for (int i = 0; i < propertiesList.size(); ++i) {
        QTreeWidgetItem *pItem = new QTreeWidgetItem(QStringList() << propertiesList.at(i));
        pItem->setFlags(pItem->flags() | Qt::ItemFlag::ItemIsEditable);
        pToolItem->addChild(pItem);
    }
    CGPropertiesRegulator::getInstance()->m_Form1->m_PropertiesTree->addTopLevelItem(pToolItem);
    CGPropertiesRegulator::getInstance()->m_Form1->m_PropertiesTree->expandAll();
}
