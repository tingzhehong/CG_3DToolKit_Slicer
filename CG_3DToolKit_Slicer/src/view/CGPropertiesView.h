﻿#ifndef CGPROPERTIESVIEW_H
#define CGPROPERTIESVIEW_H

#include <QTabWidget>

class CGPropertiesForm1;
class CGPropertiesForm2;
class CGPropertiesView : public QTabWidget
{
    Q_OBJECT

public:
    explicit CGPropertiesView();
    ~CGPropertiesView() = default;

    void InitForms();
    void InitConnections();

public:
    CGPropertiesForm1 *m_Form1;
    CGPropertiesForm2 *m_Form2;
};

#endif // CGPROPERTIESVIEW_H