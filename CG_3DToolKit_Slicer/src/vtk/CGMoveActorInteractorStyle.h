#ifndef MOVEACTORINTERACTORSTYLE_H
#define MOVEACTORINTERACTORSTYLE_H

#include <QObject>
#include <CGVTKHeader.h>
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>

class MoveActorInteractorStyle : public QObject, public vtkInteractorStyleTrackballCamera
{
    Q_OBJECT

public:
    static MoveActorInteractorStyle *New();
    vtkTypeMacro(MoveActorInteractorStyle, vtkInteractorStyleTrackballCamera)

    virtual void OnLeftButtonDown();
    virtual void OnLeftButtonUp();
	virtual void OnMouseMove();
    virtual void Rotate();
    virtual void Spin();
    virtual void OnChar();

signals:
    void planeMoved(double* pos_1, double* pos_2);
    void mouseReleased();

private:
    void OnPlaneMove();

	double *lastPos_1;
	double *lastPos_2;
	double *lastPos_3;

protected:
    explicit MoveActorInteractorStyle(QObject *parent = 0);

protected:
    vtkSmartPointer<vtkActor> m_pickedActor;

public:
	double *bounds;
	vtkSmartPointer<vtkActor> m_sphereActor_1;
	vtkSmartPointer<vtkActor> m_sphereActor_2;
	vtkSmartPointer<vtkActor> m_sphereActor_3;
	vtkSmartPointer<vtkActor> m_plane;

};
#endif // MOVEACTORINTERACTORSTYLE_H
