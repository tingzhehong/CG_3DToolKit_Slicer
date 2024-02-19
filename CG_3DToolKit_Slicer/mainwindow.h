#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <CGCommon.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QMdiArea;
class QMdiSubWindow;
class QStackedWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    void InitUi();
    void InitConnections();
    void InitPlugins();
    void QSS(const int Style);

    bool HandleDepthImage(const std::string filename);
    bool HandleOrderPointCloud();
    bool HandleDisOrderPointCloud();

private:
    CGProjectTreeView *m_pCGProjectTreeView;
    CGPropertiesView *m_pCGPropertiesView;
    CGDataTreeView *m_pCGDataTreeView;
    CGSubWindowWidget *m_pCGSubWindowWidget;
    CG2DImageView *m_pCG2DImageView;
    CG3DImageView *m_pCG3DImageView;
    CGProfileView *m_pCGProfileView;
    CGNodeView *m_pCGNodeView;
    CGFullScreenView *m_pCGFullScreenView;
    CGWebView *m_pCGWebView;

    QMdiArea *m_pMdiArea;
    QStackedWidget *m_pStackedWidget;

    CGUsersLoginDialog *m_pCGUsersLoginDialog;
    CGWaitingDialog *m_pCGWaitingDialog;
    CGAboutDialog *m_pCGAboutDialog;
    CGDisOrderDialog *m_pCGDisOrderDialog;
    CGDepthImageDialog *m_pCGDepthImageDialog;

    CGViewRegulator *m_pCGViewRegulator;

private:
    void Create2DTool(const QString toolname);
    void Create3DTool(const QString toolname);
    void CreateProfileTool(const QString toolname);
    void CreateMaths(const QString toolname);
    void CreateLogics(const QString toolname);
    void Create2DFuction(const QString toolname);
    void Create3DFuction(const QString toolname);

protected:
    void keyPressEvent(QKeyEvent *event);
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);

private slots:
    void OnProjectTreeItemSelected(QTreeWidgetItem *item, int column);
    void OnDataTreeItemSelected(QTreeWidgetItem *item, int column);
    void OnGraphicsItemValue(const QString msg);
    void OnUsersLogin(const QString user);

    void on_action_new_triggered();
    void on_action_open_triggered();
    void on_action_exit_triggered();

    void on_action_open_Image_triggered();
    void on_action_open_PointCloud_triggered();
    void on_action_save_PointCloud_triggered();

    void on_action_ClearAll_triggered();
    void on_action_FullScreen_triggered();

    void on_action_Elevation_triggered();
    void on_action_Depth_triggered();
    void on_action_Intensity_triggered();
    void on_action_Points_triggered();
    void on_action_Wire_triggered();
    void on_action_Surface_triggered();

    void on_action_Login_triggered();
    void on_action_about_triggered();

    void on_action_Aqua_triggered();
    void on_action_MacOS_triggered();
    void on_action_Ubuntu_triggered();
    void on_action_Windows_triggered();

    void on_action_SnapShot_triggered();
    void on_action_GlobalZoom_triggered();
    void on_action_ZoomIn_triggered();
    void on_action_ZoomOut_triggered();
    void on_action_console_triggered(bool checked);

    void on_action_SetViewTop_triggered();
    void on_action_SetViewFront_triggered();
    void on_action_SetViewLeft_triggered();
    void on_action_SetViewBack_triggered();
    void on_action_SetViewRight_triggered();
    void on_action_SetViewBottom_triggered();
    void on_action_SetViewIso1_triggered();
    void on_action_SetViewIso2_triggered();

    void on_action_dock_project_triggered();
    void on_action_dock_properties_triggered();
    void on_action_dock_data_triggered();
    void on_action_dock_console_triggered();

    void on_action_PickPointCoordinate_triggered(bool checked);
    void on_action_PickPointDistance_triggered(bool checked);
    void on_action_PickArea_triggered(bool checked);

    void on_action_trigger_triggered();
    void on_action_save_Flow_triggered();
    void on_action_open_Flow_triggered();

private:
    enum QSSStyle
    {
        WindowStyle,
        AquaStyle,
        MacOSStyle,
        UbuntuStyle
    };

    QFont font;
    QPalette pe;
    QString dirPath;

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
