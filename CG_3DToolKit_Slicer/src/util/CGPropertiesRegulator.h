#ifndef CGPROPERTIESREGULATOR_H
#define CGPROPERTIESREGULATOR_H

#include <QObject>
#include <QLineF>
#include <QRectF>

class CGPropertiesForm1;
class CGPropertiesForm2;
class vtkPlane;
class vtkPlanes;
class CGPropertiesRegulator : public QObject
{
    Q_OBJECT

private:
    explicit CGPropertiesRegulator(QObject *parent = nullptr);
    ~CGPropertiesRegulator() = default;

public:
    static CGPropertiesRegulator *getInstance();

    void SetCGPropertiesForms(CGPropertiesForm1 *&form1, CGPropertiesForm2 *&form2);
    
    void ShowPropertiesLine(QLineF line);
    void ShowPropertiesRect(QRectF rect);
    void ShowPropertiesAngle(qreal angle);
    void ShowPropertiesVTKPlane(vtkPlane *plane);
    void ShowPropertiesVTKSphere(double *sphere);
    void ShowPropertiesVTKPlanes(vtkPlanes *planes);

public:
    CGPropertiesForm1 *m_Form1;
    CGPropertiesForm2 *m_Form2;

private:
    static CGPropertiesRegulator *m_CGPropertiesRegulator;

};

#endif // CGPROPERTIESREGULATOR_H
