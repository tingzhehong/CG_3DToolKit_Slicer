#include "CGPropertiesView.h"
#include <CGPropertiesForm1.h>
#include <CGPropertiesForm2.h>
#include <QWidget>
#include <QDebug>

CGPropertiesView::CGPropertiesView()
{
    InitForms();
    InitConnections();
    setTabPosition(QTabWidget::South);
}

void CGPropertiesView::InitForms()
{
    m_Form1 = new CGPropertiesForm1();
    m_Form2 = new CGPropertiesForm2();
    insertTab(0, m_Form1, tr(u8"属性 1"));
    insertTab(1, m_Form2, tr(u8"属性 2"));

    CGPropertiesRegulator::getInstance()->SetCGPropertiesForms(m_Form1, m_Form2);
}

void CGPropertiesView::InitConnections()
{

}

void CGPropertiesView::OnPointsNumber(const long long number)
{
    m_Form1->SelectPointCloudProperties(number);
}
