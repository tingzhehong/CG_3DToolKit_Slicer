#ifndef CG3DIMAGEVIEW_H
#define CG3DIMAGEVIEW_H

#include <CGBaseWidget.h>
#include <CGVTKWidget.h>

class vtkActor;
class vtkCamera;
class vtkMatrix4x4;
class CG3DImageView : public CGBaseWidget
{
    Q_OBJECT

public:
    explicit CG3DImageView(QWidget *parent = nullptr);
    ~CG3DImageView();

public:
    void InitUi() override;
    void InitConnections() override;
    void LoadPCD(const std::string filename);
    void LoadCSV(const std::string filename);
    void LoadTXT(const std::string filename);
    void ClearPointCloud();

    vtkCamera* GetCamera();
    void ResetCameraParameter();
    void GetCameraParameter();
    void SetCameraParameter(double pos_x, double pos_y, double pos_z,
                            double up_x, double up_y, double up_z);

    vtkActor* GetActor() const;

public:
    double* GetCameraFocalPoint();
    double* GetCameraPosition();
    double* GetCameraViewUp();
    double* GetCameraClippingRange();
    double  GetCameraViewAngle();

public:
    CGVTKWidget *m_CGVTKWidget = nullptr;
    vtkCamera *m_CGVTKCamera = nullptr;

private:
    double *pCameraPosition = nullptr;
    double *pCameraFocalPoint = nullptr;
    double *pCameraViewUp = nullptr;
    double *pCameraClippingRange = nullptr;
    double fovy = 0;

};

#endif // CG3DIMAGEVIEW_H
