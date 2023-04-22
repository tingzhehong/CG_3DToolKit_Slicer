#ifndef CGPROFILEFORM2D_H
#define CGPROFILEFORM2D_H

#include <QWidget>
#include <CGGraphicsView.h>

class CGProfileForm2D : public QWidget
{
    Q_OBJECT

public:
    explicit CGProfileForm2D(QWidget *parent = nullptr);
    ~CGProfileForm2D() = default;

    void InitUi();
    void InitConnections();

public:
    QPixmap *m_pPixmap;
    QGraphicsPixmapItem *m_pItem;
    QGraphicsScene *m_pScene;
    CGGraphicsView *m_pGraphicsView;

    bool bGraphicsScene = false;

};

#endif // CGPROFILEFORM2D_H
