#ifndef CGFULLSCREENVIEW_H
#define CGFULLSCREENVIEW_H

#include <QWidget>
#include <CG3DImageView.h>

class CGFullScreenView : public QWidget
{
    Q_OBJECT

public:
    explicit CGFullScreenView(QWidget *parent = nullptr);
    ~CGFullScreenView();

public:
    void InitUi();
    void InitConnections();

    void SetSceneView(CG3DImageView* scene);
    CG3DImageView* GetSceneView() const;

    void ClearSceneView();

protected:
    void keyPressEvent(QKeyEvent *event);
    void closeEvent(QCloseEvent *event);

private:
    CG3DImageView *m_CG3DImageView;

};

#endif // CGFULLSCREENVIEW_H
