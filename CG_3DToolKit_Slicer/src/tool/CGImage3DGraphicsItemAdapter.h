#ifndef CGIMAGE3DGRAPHICSITEMADAPTER_H
#define CGIMAGE3DGRAPHICSITEMADAPTER_H

#include <QObject>
#include <vtkSmartPointer.h>

class vtkPlane;
class vtkPlanes;
class CGImage3DGraphicsItemAdapter : public QObject
{
    Q_OBJECT

private:
    explicit CGImage3DGraphicsItemAdapter(QObject *parent = nullptr);
    ~CGImage3DGraphicsItemAdapter() = default;

public:
    static CGImage3DGraphicsItemAdapter *getInstance();

public slots:
    void SendPlane(vtkPlane *plane);
    void SendSphere(double* sphere);
    void SendPlanes(vtkPlanes *planes);
    vtkPlane* GetPlane() const;
    vtkPlanes* GetPlanes() const;

public:
    bool m_Status = false;

private:
    static CGImage3DGraphicsItemAdapter *m_CGImage3DGraphicsItemAdapter;

public:
    vtkSmartPointer<vtkPlane> m_plane;
    vtkSmartPointer<vtkPlanes> m_planes;

};

#endif // CGIMAGE3DGRAPHICSITEMADAPTER_H
