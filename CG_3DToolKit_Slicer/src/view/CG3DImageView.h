#ifndef CG3DIMAGEVIEW_H
#define CG3DIMAGEVIEW_H

#include <CGBaseWidget.h>
#include <CGVTKUtils.h>
#include <CGVTKWidget.h>
#include <CGPointPickObserver.h>

class vtkActor;
class vtkCamera;
class vtkMatrix4x4;
class vtkDistanceWidget;
class vtkAngleWidget;
class vtkBoxWidget;
class vtkDistanceRepresentation3D;
class vtkAngleRepresentation3D;
class vtkTextActor;
class vtkLineSource;

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
    void OnUpdatePoint(float x, float y, float z);

public:
    void InitUi() override;
    void InitConnections() override;

    void LoadPCD(const std::string filename);
    void LoadCSV(const std::string filename);
    void LoadTXT(const std::string filename);
    void ClearPointCloud();

    vtkCamera* GetCamera();
    vtkActor* GetActor() const;

    void ResetCameraParameter();
    void GetCameraParameter();
    void SetCameraParameter(double pos_x, double pos_y, double pos_z,
                            double up_x, double up_y, double up_z);

    void InitActors();
    void InitTools();
    void InitPointPick();
    void RemoveTools();

    void ShowText2D();
    void ShowText3D();
    void ShowPCD();
    void ShowPointPickInfo(const bool enable);

public:
    enum ToolType
    {
        DistanceTool,
        AngleTool,
        BoxTool
    };

    ToolType  m_CurrentToolType;
    ToolType  m_LastToolType;

    enum PickType
    {
        SinglePoint,
        DoublePoint
    };

    PickType m_PickType;

    double* GetCameraFocalPoint();
    double* GetCameraPosition();
    double* GetCameraViewUp();
    double* GetCameraClippingRange();
    double  GetCameraViewAngle();

private:
    void InitDistanceTool();
    void InitAngleTool();
    void InitBoxTool();

    void HandlePickPointCoordinate(float x, float y, float z);
    void HandlePickPointDistance(float x, float y, float z);

    float LineDistance(float x1, float y1, float z1,
                       float x2, float y2, float z2);

public:
    CGVTKWidget *m_CGVTKWidget = nullptr;
    CGVTKUtils::CGPointPickObserver *m_CGPointPicker = nullptr;

    vtkSmartPointer<vtkCamera> m_CGVTKCamera;
    vtkSmartPointer<vtkActor> m_Actor;
    vtkSmartPointer<vtkTextActor> m_TextActor_X;
    vtkSmartPointer<vtkTextActor> m_TextActor_Y;
    vtkSmartPointer<vtkTextActor> m_TextActor_Z;
    vtkSmartPointer<vtkTextActor> m_TextActor_Distance;
    vtkSmartPointer<vtkActor> m_PickSphere_1;
    vtkSmartPointer<vtkActor> m_PickSphere_2;
    vtkSmartPointer<vtkLineSource> m_PickLineSouce;
    vtkSmartPointer<vtkPolyDataMapper> m_PickLineMapper;
    vtkSmartPointer<vtkActor> m_PickLine;

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
