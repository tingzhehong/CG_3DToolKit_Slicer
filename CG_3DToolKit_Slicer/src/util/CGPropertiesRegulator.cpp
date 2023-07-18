#include "CGPropertiesRegulator.h"
#include <QDebug>


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

void CGPropertiesRegulator::SetCGPropertiesForms(CGPropertiesForm1 *form1, CGPropertiesForm2 *form2)
{
    m_Form1 = form1;
    m_Form2 = form2;
}
