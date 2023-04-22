#ifndef CGVIEWREGULATOR_H
#define CGVIEWREGULATOR_H

#include <QObject>

class CG2DImageView;
class CG3DImageView;
class CGNodeView;
class CGProfileView;
class vtkActor;

class CGViewRegulator : public QObject
{
    Q_OBJECT

public:
    explicit CGViewRegulator(QObject *parent = nullptr);
    ~CGViewRegulator();

public slots:
    void On2DImageViewRequest();
    void On3DImageViewRequest();
    void OnNodeViewRequest();
    void OnProfileViewRequest();

public:
    CG2DImageView *m_CG2DImageView;
    CG3DImageView *m_CG3DImageView;
    CGNodeView *m_CGNodeView;
    CGProfileView *m_CGProfileView;

};

#endif // CGVIEWREGULATOR_H
