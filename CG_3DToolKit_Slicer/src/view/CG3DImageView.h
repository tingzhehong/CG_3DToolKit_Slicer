#ifndef CG3DIMAGEVIEW_H
#define CG3DIMAGEVIEW_H

#include <CGBaseWidget.h>
#include <CGVTKUtils.h>
#include <CGVTKWidget.h>

class vtkActor;
class vtkCamera;
class vtkMatrix4x4;
class vtkDistanceWidget;
class vtkAngleWidget;
class vtkBoxWidget;
class vtkDistanceRepresentation3D;
class vtkAngleRepresentation3D;

class CG3DImageView : public CGBaseWidget
{
    Q_OBJECT

public:
    explicit CG3DImageView(QWidget *parent = nullptr);
    ~CG3DImageView();

signals:

public slots:
    void OnUseTool();
    void OnDelTool();

public:
    void InitUi() override;
    void InitConnections() override;
    void ShowPCD();
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

    void InitTools();
    void RemoveTools();

public:
    enum ToolType
    {
        DistanceTool,
        AngleTool,
        BoxTool
    };

    ToolType  m_CurrentToolType;
    ToolType  m_LastToolType;

    double* GetCameraFocalPoint();
    double* GetCameraPosition();
    double* GetCameraViewUp();
    double* GetCameraClippingRange();
    double  GetCameraViewAngle();

private:
    void InitDistanceTool();
    void InitAngleTool();
    void InitBoxTool();

public:
    CGVTKWidget *m_CGVTKWidget = nullptr;
    vtkCamera *m_CGVTKCamera = nullptr;
    vtkSmartPointer<vtkActor> m_Actor;

private:
    double *pCameraPosition = nullptr;
    double *pCameraFocalPoint = nullptr;
    double *pCameraViewUp = nullptr;
    double *pCameraClippingRange = nullptr;
    double  fovy = 0;

    vtkSmartPointer<vtkDistanceWidget> m_pDistanceWidgetTool;
    vtkSmartPointer<vtkAngleWidget> m_pAngleWidgetTool;
    vtkSmartPointer<vtkBoxWidget> m_pBoxWidgetTool;

    vtkSmartPointer<vtkDistanceRepresentation3D> m_pDistanceRep;
    vtkSmartPointer<vtkAngleRepresentation3D> m_pAngleRep;

    bool IsTool = false;

};

#endif // CG3DIMAGEVIEW_H
