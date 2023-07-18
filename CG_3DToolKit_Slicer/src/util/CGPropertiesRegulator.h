#ifndef CGPROPERTIESREGULATOR_H
#define CGPROPERTIESREGULATOR_H

#include <QObject>

class CGPropertiesForm1;
class CGPropertiesForm2;
class CGPropertiesRegulator : public QObject
{
    Q_OBJECT

private:
    explicit CGPropertiesRegulator(QObject *parent = nullptr);
    ~CGPropertiesRegulator() = default;

public:
    static CGPropertiesRegulator *getInstance();

    void SetCGPropertiesForms(CGPropertiesForm1 *&form1, CGPropertiesForm2 *&form2);

public:
    CGPropertiesForm1 *m_Form1;
    CGPropertiesForm2 *m_Form2;

private:
    static CGPropertiesRegulator *m_CGPropertiesRegulator;

};

#endif // CGPROPERTIESREGULATOR_H
