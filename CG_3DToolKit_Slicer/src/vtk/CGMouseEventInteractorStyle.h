﻿#ifndef MOUSEEVENTINTERACTORSTYLE_H
#define MOUSEEVENTINTERACTORSTYLE_H

#include <QObject>
#include <CGVTKHeader.h>
#include <vtkInteractorStyleTrackballCamera.h>

class MouseEventInteractorStyle : public QObject, public vtkInteractorStyleTrackballCamera
{
    Q_OBJECT

public:
    static MouseEventInteractorStyle *New();
    vtkTypeMacro(MouseEventInteractorStyle, vtkInteractorStyleTrackballCamera)

    void OnRightButtonDown();
    void OnRightButtonUp();
    void OnLeftButtonDown();
    void OnLeftButtonUp();
    void OnMouseMove();
    void OnKeyDown();
    void OnKeyUp();

    void setMoveActor(bool move);
    bool moveActor() const;

signals:
    void ctrlPressed(bool pressed);
    void mousePressed(double* pos);
    void mouseMoved(double* pos);
    void mouseReleased(double* pos);

protected:
    explicit MouseEventInteractorStyle(QObject *parent = 0);

public:
    void updateCurrentPos();
    void getPressedActor();

    vtkActor* m_pickedActor = nullptr;
    double m_pos[3];
    bool m_moveActor = false;

    enum SectionType
    {
        SectionItemDefault,
        SectionItemHorizontal,
        SectionItemVertical
    };
    SectionType m_SectionType = SectionItemDefault;

private:
    double _pos[3];
    double _bounds[6];

};

#endif // MOUSEEVENTINTERACTORSTYLE_H
