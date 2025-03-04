#ifndef CGPROPERTIESFORM1_H
#define CGPROPERTIESFORM1_H

#include <QWidget>
#include <CGOCVHeader.h>
#include <CGPointCloud.h>

class QTreeWidget;
class QTreeWidgetItem;

class CGPropertiesForm1 : public QWidget
{
    Q_OBJECT

public:
    explicit CGPropertiesForm1(QWidget *parent = nullptr);
    ~CGPropertiesForm1() = default;

    void InitUi();
    void CreateImageProperties();
    void CreatePointCloudProperties();
    void SelectPointCloudProperties(const long long number);

public:
    QTreeWidget *m_PropertiesTree;

};

#endif // CGPROPERTIESFORM1_H
