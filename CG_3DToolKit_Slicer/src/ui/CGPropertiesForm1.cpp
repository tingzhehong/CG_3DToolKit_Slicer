#include "CGPropertiesForm1.h"
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QDebug>

CGPropertiesForm1::CGPropertiesForm1(QWidget *parent) : QWidget(parent)
{
    InitUi();
}

void CGPropertiesForm1::InitUi()
{
    m_PropertiesTree = new QTreeWidget(this);
    m_PropertiesTree->setHeaderLabel(tr(u8"属性列表"));

    QVBoxLayout *pTreeLayout = new QVBoxLayout();
    pTreeLayout->addWidget(m_PropertiesTree);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addLayout(pTreeLayout);

    setLayout(pMainLayout);
}

void CGPropertiesForm1::CreateImageProperties()
{
    m_PropertiesTree->clear();

    QTreeWidgetItem *pImageItem = new QTreeWidgetItem(QStringList{tr(u8"图像属性")});

    QStringList propertiesList;
    QString strDepthImageSize = QString::number(g_Image.DepthImage.cols) + " , " + QString::number(g_Image.DepthImage.rows);
    QString strColorImageSize = QString::number(g_Image.ColorImage.cols) + " , " + QString::number(g_Image.ColorImage.rows);
    QString strGrayImageSize = QString::number(g_Image.GrayImage.cols) + " , " + QString::number(g_Image.GrayImage.rows);
    QString strIntensityImageSize = QString::number(g_Image.IntensityImage.cols) + " , " + QString::number(g_Image.IntensityImage.rows);
    propertiesList.append(tr(u8"Depth Image Size: ") + strDepthImageSize);
    propertiesList.append(tr(u8"Color Image Size: ") + strColorImageSize);
    propertiesList.append(tr(u8"Gray Image Size: ") + strGrayImageSize);
    propertiesList.append(tr(u8"Intensity Image Size: ") + strIntensityImageSize);

    for (int i = 0; i < propertiesList.size(); ++i) {
        QTreeWidgetItem *pItem = new QTreeWidgetItem(QStringList() << propertiesList.at(i));
        pImageItem->addChild(pItem);
    }
    m_PropertiesTree->addTopLevelItem(pImageItem);
    m_PropertiesTree->expandAll();
}

void CGPropertiesForm1::CreatePointCloudProperties()
{
    if (g_PointCloud == nullptr) return;

    m_PropertiesTree->clear();

    QTreeWidgetItem *pPointCloudItem = new QTreeWidgetItem(QStringList{tr(u8"点云属性")});

    QStringList propertiesList;
    QString strPointCloudSize = QString::number(g_PointCloud->size());
    QString strXPitch = QString::number(g_XPitch);
    QString strYPitch = QString::number(g_YPitch);
    propertiesList.append(tr(u8"Point Cloud Size: ") + strPointCloudSize);
    propertiesList.append(tr(u8"X Pitch: ") + strXPitch);
    propertiesList.append(tr(u8"Y Pitch: ") + strYPitch);

    for (int i = 0; i < propertiesList.size(); ++i) {
        QTreeWidgetItem *pItem = new QTreeWidgetItem(QStringList() << propertiesList.at(i));
        pPointCloudItem->addChild(pItem);
    }
    m_PropertiesTree->addTopLevelItem(pPointCloudItem);
    m_PropertiesTree->expandAll();
}

void CGPropertiesForm1::SelectPointCloudProperties(const long long number)
{
    m_PropertiesTree->clear();

    QTreeWidgetItem *pPointCloudItem = new QTreeWidgetItem(QStringList{tr(u8"点云属性")});

    QStringList propertiesList;
    QString strPointCloudSize = QString::number(number);
    propertiesList.append(tr(u8"Point Cloud Size: ") + strPointCloudSize);

    for (int i = 0; i < propertiesList.size(); ++i) {
        QTreeWidgetItem *pItem = new QTreeWidgetItem(QStringList() << propertiesList.at(i));
        pPointCloudItem->addChild(pItem);
    }
    m_PropertiesTree->addTopLevelItem(pPointCloudItem);
    m_PropertiesTree->expandAll();
}
