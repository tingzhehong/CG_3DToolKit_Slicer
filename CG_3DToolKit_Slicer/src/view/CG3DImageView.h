#ifndef CG3DIMAGEVIEW_H
#define CG3DIMAGEVIEW_H

#include <CGBaseWidget.h>
#include <CGVTKUtils.h>
#include <CGVTKWidget.h>
#include <CGPointPickObserver.h>
#include <CGBoxWidgetObserver.h>
#include <CGPlaneWidgetObserver.h>
#include <CGImage3DGraphicsItemAdapter.h>
#include <CGPropertiesRegulator.h>

class vtkActor;
class vtkCubeAxesActor;
class vtkCamera;
class vtkMatrix4x4;
class vtkDistanceWidget;
class vtkAngleWidget;
class vtkBoxWidget;
class vtkPlaneWidget;
class vtkDistanceRepresentation3D;
class vtkAngleRepresentation3D;
class vtkTextActor;
class vtkLineSource;
class vtkInteractorStyleTrackballCamera;
class CGAreaPickerInteractorStyle;

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
    void OnBoxWidgetPlaneChanged(vtkPlanes* planes);
    void OnPlaneWidgetPlaneChanged(vtkPlane* plane);

public:
    void InitUi() override;
    void InitConnections() override;

    void LoadPCD(const std::string filename);
    void LoadCSV(const std::string filename);
    void LoadTXT(const std::string filename);
    void LoadPLY(const std::string filename);
    void LoadSTL(const std::string filename);
    void LoadOBJ(const std::string filename);
    void ClearPointCloud();

    vtkCamera* GetCamera();
    vtkActor* GetActor() const;
    void SetCamera(vtkCamera* camera);
    void SetActor(vtkActor* actor);

    void ResetCameraParameter();
    void GetCameraParameter();
    void SetCameraParameter(double pos_x, double pos_y, double pos_z,
                            double up_x, double up_y, double up_z);
    void SetPointPickSize();
    void SetRepresentationToPoints();
    void SetRepresentationToWireframe();
    void SetRepresentationToSurface();
    void SaveMesh(const std::string filename);
    
    bool ReconstructionDepthImage2Mesh(vtkSmartPointer<vtkActor> actor);
    bool HasMeshStructure(vtkSmartPointer<vtkActor> actor);
    int  CalcSparse();

    void InitActors();
    void InitTools();
    void InitPointPick();
    void InitAreaPick();
    void InitInteractorStyle();
    void RemoveTools();

    void ShowText2D();
    void ShowText3D();
    void ShowPCD();
    void ShowPointPickInfo(const bool enable);
    void ChangeInteractorStyle(const int style);

protected:
    void keyPressEvent(QKeyEvent *event);

public:
    enum ToolType
    {
        DistanceTool,
        AngleTool,
        BoxTool,
        PlaneTool
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
    void InitPlaneTool();

    void HandlePickPointCoordinate(float x, float y, float z);
    void HandlePickPointDistance(float x, float y, float z);

    float LineDistance(float x1, float y1, float z1,
                       float x2, float y2, float z2);

public:
    void CreatCubeAxes();
    void CreatXYGrids(double* bounds);

public:
    CGVTKWidget *m_CGVTKWidget = nullptr;
    CGVTKUtils::CGPointPickObserver *m_CGPointPicker = nullptr;
    CGVTKUtils::CGBoxWidgetObserver *m_CGBoxWidgeter = nullptr;
    CGVTKUtils::CGPlaneWidgetObserver *m_CGPlaneWidgeter = nullptr;

    vtkSmartPointer<vtkCamera> m_CGVTKCamera;
    vtkSmartPointer<vtkActor> m_Actor;
    vtkSmartPointer<vtkActor> m_GridsActor;
    vtkSmartPointer<vtkCubeAxesActor> m_CubeAxesActor;

    vtkSmartPointer<vtkTextActor> m_TextActor_X;
    vtkSmartPointer<vtkTextActor> m_TextActor_Y;
    vtkSmartPointer<vtkTextActor> m_TextActor_Z;
    vtkSmartPointer<vtkTextActor> m_TextActor_Distance;
    vtkSmartPointer<vtkActor> m_PickSphere_1;
    vtkSmartPointer<vtkActor> m_PickSphere_2;
    vtkSmartPointer<vtkLineSource> m_PickLineSouce;
    vtkSmartPointer<vtkPolyDataMapper> m_PickLineMapper;
    vtkSmartPointer<vtkActor> m_PickLine;

    vtkSmartPointer<vtkInteractorStyleTrackballCamera> m_DefaultStyle;
    vtkSmartPointer<CGAreaPickerInteractorStyle> m_AreaPickerStyle;

private:
    double *pCameraPosition = nullptr;
    double *pCameraFocalPoint = nullptr;
    double *pCameraViewUp = nullptr;
    double *pCameraClippingRange = nullptr;
    double  fovy = 0;

    vtkSmartPointer<vtkDistanceWidget> m_pDistanceWidgetTool;
    vtkSmartPointer<vtkAngleWidget> m_pAngleWidgetTool;
    vtkSmartPointer<vtkBoxWidget> m_pBoxWidgetTool;
    vtkSmartPointer<vtkPlaneWidget> m_pPlaneWidgetTool;

    vtkSmartPointer<vtkDistanceRepresentation3D> m_pDistanceRep;
    vtkSmartPointer<vtkAngleRepresentation3D> m_pAngleRep;

    bool IsTool = false;
    bool IsGrids = true;
};

#endif // CG3DIMAGEVIEW_H
