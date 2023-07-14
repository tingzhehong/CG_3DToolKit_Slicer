#ifndef CGPROPERTIESFORM2_H
#define CGPROPERTIESFORM2_H

#include <QWidget>

class QTreeWidget;
class QTreeWidgetItem;

class CGPropertiesForm2 : public QWidget
{
    Q_OBJECT

public:
    explicit CGPropertiesForm2(QWidget *parent = nullptr);
    ~CGPropertiesForm2() = default;

    void InitUi();

public:
    QTreeWidget *m_PropertiesTree;

};

#endif // CGPROPERTIESFORM2_H
