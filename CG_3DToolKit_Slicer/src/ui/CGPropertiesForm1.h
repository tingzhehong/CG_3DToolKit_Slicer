#ifndef CGPROPERTIESFORM1_H
#define CGPROPERTIESFORM1_H

#include <QWidget>

class QTreeWidget;
class QTreeWidgetItem;

class CGPropertiesForm1 : public QWidget
{
    Q_OBJECT

public:
    explicit CGPropertiesForm1(QWidget *parent = nullptr);
    ~CGPropertiesForm1() = default;

    void InitUi();

public:
    QTreeWidget *m_NullTree;

};

#endif // CGPROPERTIESFORM1_H
